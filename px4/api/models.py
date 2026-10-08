# -*- coding: utf-8 -*-
"""接口请求模型."""
from typing import List, Literal, Optional

from pydantic import BaseModel, Field


class LinkConnectReq(BaseModel):
    id: Optional[str] = Field(None, pattern=r'^[A-Za-z0-9_\-]{1,32}$',
                              description='地面站标识, 缺省自动生成 gcs1/gcs2...')
    port: str = Field(..., description='串口号(COM3, /dev/ttyUSB0) 或连接串'
                                       '(udp:127.0.0.1:14550, tcp:127.0.0.1:5760)')
    baud: int = Field(57600, ge=1200, le=921600, description='串口波特率')
    name: Optional[str] = Field(None, max_length=64, description='地面站备注名')


class CorrectionTargetReq(BaseModel):
    id: Optional[int] = Field(None, description='外部系统数据包编号(仅回显)')
    lat: float = Field(..., ge=-90, le=90, description='纬度(度)')
    lon: float = Field(..., ge=-180, le=180, description='经度(度)')
    alt: float = Field(0.0, description='高度(米), 语义由 alt_frame 决定')
    alt_frame: Literal['relative', 'amsl'] = Field(
        'relative', description='relative=相对起飞点高度(默认); '
                                'amsl=绝对高程, 服务用 home 高程换算成相对高度')
    vx: float = Field(0.0, description='速度 x (ned 时为北向, enu 时为东向) m/s')
    vy: float = Field(0.0, description='速度 y (ned 时为东向, enu 时为北向) m/s')
    vz: float = Field(0.0, description='速度 z (ned 时为向下, enu 时为向上) m/s')
    vel_frame: Literal['ned', 'enu'] = Field(
        'ned', description='速度坐标系, MAVLink 内部使用 NED')
    type: int = Field(0, description='目标类型(透传记录, 不参与控制)')
    track: bool = Field(True, description='true=持续引导流(默认); '
                                          'false=只下发一次最终目标点')
    confidence: Optional[int] = Field(None, ge=0, le=100, description='置信度 0~100')
    min_confidence: Optional[int] = Field(
        None, ge=0, le=100, description='低于该置信度的数据直接丢弃(覆盖全局配置)')
    rate_hz: Optional[float] = Field(None, ge=1.0, le=30.0,
                                     description='持续引导下发频率, 默认 10Hz')
    auto_takeoff: bool = Field(False, description='飞机未解锁时先自动解锁起飞')
    takeoff_alt: Optional[float] = Field(None, gt=0, description='自动起飞高度(米)')
    ensure_offboard: bool = Field(True, description='引导期间把飞机维持在 OFFBOARD 模式')
    ttl: Optional[float] = Field(None, gt=0,
                                 description='引导流存活秒数, 到期自动停止; '
                                             '缺省用配置(0=不限)')


class GuidanceGlobalReq(CorrectionTargetReq):
    link_id: str = Field('all', description='目标地面站 id, 或 all=全部已连接地面站')


class WaypointReq(BaseModel):
    lat: float = Field(..., ge=-90, le=90)
    lon: float = Field(..., ge=-180, le=180)
    alt: float = Field(20.0, gt=0, description='相对起飞点高度(米)')
    drop: Optional['DropReq'] = Field(
        None, description='抛投项(该航点执行 DO_SET_SERVO+NAV_DELAY+回位)')


class DropReq(BaseModel):
    channel: int = Field(..., ge=1, le=16, description='舵机通道号')
    pwm_on: int = Field(2000, ge=500, le=2500)
    pwm_off: Optional[int] = Field(1000, ge=500, le=2500)
    hold_ms: int = Field(1500, ge=0)


class GotoReq(BaseModel):
    lat: float = Field(..., ge=-90, le=90, description='目的地纬度')
    lon: float = Field(..., ge=-180, le=180, description='目的地经度')
    alt: float = Field(20.0, gt=0, description='目的地相对高度(米)')
    speed: Optional[float] = Field(
        None, ge=0.2, le=20.0, description='巡航速度 m/s, 用 DO_CHANGE_SPEED 下发, 不写参数')
    auto_takeoff: bool = Field(True, description='未解锁时先自动解锁起飞')
    threshold: Optional[float] = Field(None, gt=0, description='到达判定水平距离(米)')
    timeout: Optional[float] = Field(None, gt=0, description='飞行超时(秒)')


class MissionReq(BaseModel):
    waypoints: List[WaypointReq] = Field(..., min_length=1)
    end_action: Literal['none', 'rtl', 'land'] = Field(
        'none', description='任务结束动作: none=停末航点保持, rtl=返航, land=末点降落')
    auto_takeoff: bool = Field(True, description='未解锁时先自动解锁起飞')
    takeoff_alt: Optional[float] = Field(None, gt=0, description='起飞高度, 缺省取最高航点')
    timeout: Optional[float] = Field(None, gt=0, description='任务总超时(秒)')


class TakeoffReq(BaseModel):
    alt: float = Field(5.0, gt=0, le=120)
    mode: Literal['POSCTL', 'ALTCTL'] = 'POSCTL'


class ModeReq(BaseModel):
    mode: str = Field(..., min_length=2, max_length=16)


class ArmReq(BaseModel):
    mode: Literal['POSCTL', 'ALTCTL'] = Field('POSCTL')


class DisarmReq(BaseModel):
    force: bool = Field(False, description='true=强制上锁(21196)')


class EmergencyReq(BaseModel):
    mode: Literal['LOITER', 'RTL', 'LAND'] = Field('LOITER')
    cancel_task: bool = Field(True, description='同时取消正在执行的单点/航点任务')
    stop_guidance: bool = Field(True, description='同时停止外部持续引导流')
