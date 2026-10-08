# -*- coding: utf-8 -*-
"""FastAPI 路由: 多地面站连接管理 / 遥测查询 / 外部坐标持续引导 /
单点飞行 / 航点任务 / 急停·立即RTL·立即LAND.
"""
import itertools
import os
import sys
import threading
import time
from contextlib import asynccontextmanager
from typing import Optional

from fastapi import FastAPI, HTTPException

from . import config, tasks as taskmod
from .guidance import GuidanceStreamer
from .link import Link, get_available_ports
from .models import (ArmReq, CorrectionTargetReq, DisarmReq, EmergencyReq,
                     GuidanceGlobalReq, GotoReq, LinkConnectReq, MissionReq,
                     ModeReq, TakeoffReq)
from .tasklog import TaskLogRouter

_PX4 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PX4 not in sys.path:
    sys.path.insert(0, _PX4)

import px4_control_test as tct

_links = {}
_links_lock = threading.Lock()
_id_seq = itertools.count(1)
_router = None


def _install_router():
    global _router
    if _router is None:
        _router = TaskLogRouter(mirror=True).install()
        tct.ARM_CONFIRM_HOOK = (
            lambda reason='', all_ready=True: True) if config.AUTO_CONFIRM_ARM \
            else (lambda reason='', all_ready=True: False)


@asynccontextmanager
async def lifespan(app: FastAPI):
    _install_router()
    yield
    with _links_lock:
        items = list(_links.values())
    for link in items:
        try:
            link.close()
        except Exception:
            pass


app = FastAPI(title='PX4 多地面站控制 API',
              description='外部修正坐标持续引导 + 单点/航点飞行 + 急停/RTL/LAND',
              version='1.0.0', lifespan=lifespan)


def _new_id():
    while True:
        cid = 'gcs%d' % next(_id_seq)
        with _links_lock:
            if cid not in _links:
                return cid


def _get(link_id: str) -> Link:
    with _links_lock:
        link = _links.get(link_id)
    if link is None:
        raise HTTPException(status_code=404, detail='地面站不存在: %s' % link_id)
    if not link.connected:
        raise HTTPException(status_code=409,
                            detail='地面站 %s 未连接' % link_id)
    return link


def _task_dict(rec):
    return rec.to_dict()


# ---------------- 基础 ----------------

@app.get('/health')
def health():
    with _links_lock:
        n = len(_links)
    return {'ok': True, 'version': '1.0.0', 'links': n,
            'auto_confirm_arm': config.AUTO_CONFIRM_ARM}


@app.get('/api/v1/ports')
def list_ports():
    return {'ports': [{'device': d, 'description': s}
                      for d, s in get_available_ports()]}


# ---------------- 地面站连接 ----------------

@app.post('/api/v1/links', status_code=201)
def connect_link(req: LinkConnectReq):
    with _links_lock:
        if any(l.port == req.port and l.connected for l in _links.values()):
            raise HTTPException(status_code=409, detail='该串口/连接串已连接: %s'
                                % req.port)
        link_id = req.id or _new_id()
        if link_id in _links:
            raise HTTPException(status_code=409, detail='id 已存在: %s' % link_id)
        link = Link(link_id, req.port, req.baud, req.name)
        link.tasks = taskmod.TaskManager(link)
        link.guidance = GuidanceStreamer(link)
        _links[link_id] = link
    try:
        info = link.connect()
    except Exception as e:
        with _links_lock:
            _links.pop(link_id, None)
        raise HTTPException(status_code=502, detail=str(e))
    return info


@app.get('/api/v1/links')
def list_links():
    with _links_lock:
        items = list(_links.values())
    return {'links': [l.info() for l in items]}


@app.get('/api/v1/links/{link_id}')
def get_link(link_id: str):
    with _links_lock:
        link = _links.get(link_id)
    if link is None:
        raise HTTPException(status_code=404, detail='地面站不存在: %s' % link_id)
    return link.info()


@app.delete('/api/v1/links/{link_id}')
def disconnect_link(link_id: str):
    with _links_lock:
        link = _links.pop(link_id, None)
    if link is None:
        raise HTTPException(status_code=404, detail='地面站不存在: %s' % link_id)
    link.close()
    return {'id': link_id, 'closed': True}


@app.get('/api/v1/links/{link_id}/telemetry')
def telemetry(link_id: str):
    return _get(link_id).telemetry()


# ---------------- 外部修正坐标持续引导 ----------------

def _submit_guidance(link: Link, req) -> dict:
    if link.tasks is not None:
        link.tasks.cancel()
    link.stop_offboard_stream()
    try:
        accepted, detail = link.guidance.submit(req)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    if not accepted:
        raise HTTPException(status_code=422, detail=detail)
    return {'link_id': link.id, 'accepted': True, 'guidance': detail}


@app.post('/api/v1/links/{link_id}/guidance')
def post_guidance(link_id: str, req: CorrectionTargetReq):
    return _submit_guidance(_get(link_id), req)


@app.post('/api/v1/guidance')
def post_guidance_global(req: GuidanceGlobalReq):
    with _links_lock:
        items = list(_links.values())
    if req.link_id == 'all':
        targets = [l for l in items if l.connected]
        if not targets:
            raise HTTPException(status_code=409, detail='没有已连接的地面站')
    else:
        targets = [_get(req.link_id)]
    return {'results': [_submit_guidance(l, req) for l in targets]}


@app.get('/api/v1/links/{link_id}/guidance')
def get_guidance(link_id: str):
    return _get(link_id).guidance.status()


@app.delete('/api/v1/links/{link_id}/guidance')
def stop_guidance(link_id: str):
    link = _get(link_id)
    stopped = link.guidance.stop('api_stop')
    return {'link_id': link.id, 'stopped': stopped,
            'guidance': link.guidance.status()}


# ---------------- 单点飞行 / 航点任务 ----------------

def _start_task(link: Link, kind, params, fn):
    try:
        return link.tasks.start(kind, params, fn)
    except taskmod.BusyError as e:
        raise HTTPException(status_code=409, detail=str(e))


@app.post('/api/v1/links/{link_id}/goto', status_code=202)
def post_goto(link_id: str, req: GotoReq):
    link = _get(link_id)
    if link.guidance is not None and link.guidance.status()['active']:
        link.guidance.stop('goto_takeover')
    params = {'lat': req.lat, 'lon': req.lon, 'alt': req.alt, 'speed': req.speed}
    with link.tasks.lock:
        cur = link.tasks.current
        retarget = (cur is not None and cur.status in ('pending', 'running')
                    and cur.kind == 'goto')
    if retarget:
        link.tasks.set_update(params)
        return {'link_id': link.id, 'retarget': True, 'target': params}
    rec = _start_task(link, 'goto', params,
                      lambda ctx: taskmod.do_goto(ctx, req))
    return {'link_id': link.id, 'task': rec.to_dict()}


@app.post('/api/v1/links/{link_id}/mission', status_code=202)
def post_mission(link_id: str, req: MissionReq):
    link = _get(link_id)
    if link.guidance is not None and link.guidance.status()['active']:
        link.guidance.stop('mission_takeover')
    params = {'waypoints': len(req.waypoints), 'end_action': req.end_action}
    rec = _start_task(link, 'mission', params,
                      lambda ctx: taskmod.do_mission(ctx, req))
    return {'link_id': link.id, 'task': rec.to_dict()}


@app.get('/api/v1/links/{link_id}/tasks')
def list_tasks(link_id: str, tail: int = 200):
    return {'link_id': link_id, 'tasks': _get(link_id).tasks.list(log_tail=tail)}


@app.get('/api/v1/links/{link_id}/tasks/{task_id}')
def get_task(link_id: str, task_id: str):
    rec = _get(link_id).tasks.get(task_id)
    if rec is None:
        raise HTTPException(status_code=404, detail='任务不存在: %s' % task_id)
    return rec


# ---------------- 急停 / 立即动作 ----------------

def _emergency(link: Link, mode: str, body: Optional[EmergencyReq]):
    cancel_task = True if body is None else body.cancel_task
    stop_guid = True if body is None else body.stop_guidance
    if cancel_task and link.tasks is not None:
        link.tasks.cancel()
        link.stop_offboard_stream()
    if stop_guid and link.guidance is not None:
        link.guidance.stop('emergency_%s' % mode)
    try:
        sent = link.immediate_mode(mode)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    settle = None
    t = link.tel_copy()
    if sent in ('RTL', 'LAND') and cancel_task and t.get('armed'):
        link.tasks.preempt('settle_land')
        try:
            settle = link.tasks.start('settle_land', {'mode': sent},
                                      taskmod.do_settle_land,
                                      exclusive=False).to_dict()
        except taskmod.BusyError as e:
            settle = {'error': str(e)}
    t = link.tel_copy()
    return {'link_id': link.id, 'mode': sent, 'sent': True,
            'current_mode': t.get('mode'), 'armed': t.get('armed'),
            'task_cancelled': cancel_task, 'guidance_stopped': stop_guid,
            'settle_task': settle, 'at': time.time()}


@app.post('/api/v1/links/{link_id}/stop')
def emergency_stop(link_id: str, body: Optional[EmergencyReq] = None):
    """急停: 立即切 LOITER 悬停 (直发不排队)"""
    mode = 'LOITER' if body is None else body.mode
    return _emergency(_get(link_id), mode, body)


@app.post('/api/v1/links/{link_id}/rtl')
def emergency_rtl(link_id: str, body: Optional[EmergencyReq] = None):
    """立即 RTL 返航"""
    return _emergency(_get(link_id), 'RTL', body)


@app.post('/api/v1/links/{link_id}/land')
def emergency_land(link_id: str, body: Optional[EmergencyReq] = None):
    """立即 LAND 降落"""
    return _emergency(_get(link_id), 'LAND', body)


# ---------------- 基础控制 ----------------

@app.post('/api/v1/links/{link_id}/arm', status_code=202)
def post_arm(link_id: str, req: ArmReq):
    link = _get(link_id)
    rec = _start_task(link, 'arm', {'mode': req.mode},
                      lambda ctx: taskmod.do_arm(ctx, req))
    return {'link_id': link.id, 'task': rec.to_dict()}


@app.post('/api/v1/links/{link_id}/disarm', status_code=202)
def post_disarm(link_id: str, req: DisarmReq):
    link = _get(link_id)
    rec = _start_task(link, 'disarm', {'force': req.force},
                      lambda ctx: taskmod.do_disarm(ctx, req))
    return {'link_id': link.id, 'task': rec.to_dict()}


@app.post('/api/v1/links/{link_id}/takeoff', status_code=202)
def post_takeoff(link_id: str, req: TakeoffReq):
    link = _get(link_id)
    if link.guidance is not None and link.guidance.status()['active']:
        link.guidance.stop('takeoff_takeover')
    rec = _start_task(link, 'takeoff', {'alt': req.alt, 'mode': req.mode},
                      lambda ctx: taskmod.do_takeoff(ctx, req))
    return {'link_id': link.id, 'task': rec.to_dict()}


@app.post('/api/v1/links/{link_id}/mode', status_code=202)
def post_mode(link_id: str, req: ModeReq):
    link = _get(link_id)
    rec = _start_task(link, 'mode', {'mode': req.mode.upper()},
                      lambda ctx: taskmod.do_set_mode(ctx, req))
    return {'link_id': link.id, 'task': rec.to_dict()}


@app.post('/api/v1/links/{link_id}/rc/handover', status_code=202)
def post_rc_handover(link_id: str):
    link = _get(link_id)
    rec = _start_task(link, 'rc_handover', {},
                      lambda ctx: taskmod.do_rc_handover(ctx, None))
    return {'link_id': link.id, 'task': rec.to_dict()}
