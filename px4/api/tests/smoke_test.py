# -*- coding: utf-8 -*-
"""px4.api 无硬件冒烟测试 (内置假飞控).

运行: python -m px4.api.tests.smoke_test
      python px4/api/tests/smoke_test.py
覆盖: 连接/遥测/外部修正坐标持续引导/置信度过滤/急停/单点飞行/航点任务/
      RTL落地自动上锁/基础控制(切模式·解锁·起飞·降落)/断开
"""
import os
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_PX4 = os.path.dirname(os.path.dirname(_HERE))
_ROOT = os.path.dirname(_PX4)
for _p in (_PX4, _ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from fastapi.testclient import TestClient               # noqa: E402

from px4.api.tests.fake_fc import FakeFC                # noqa: E402

PORT = 15761
CONN = 'tcp:127.0.0.1:%d' % PORT


def wait_task(client, link_id, task_id, timeout=120.0):
    t_end = time.time() + timeout
    while time.time() < t_end:
        r = client.get('/api/v1/links/%s/tasks/%s' % (link_id, task_id))
        assert r.status_code == 200, r.text
        data = r.json()
        if data['status'] not in ('pending', 'running'):
            return data
        time.sleep(0.5)
    raise AssertionError('任务超时未结束: %s' % task_id)


def wait_armed(client, link_id, want, timeout=60.0):
    t_end = time.time() + timeout
    armed = None
    while time.time() < t_end:
        tel = client.get('/api/v1/links/%s/telemetry' % link_id).json()
        armed = tel['flight_status']['armed']
        if armed is want:
            return armed
        time.sleep(0.5)
    return armed


def wait_mode(client, link_id, want, timeout=30.0):
    """等遥测模式变成 want (模式切换依赖心跳上报, 需要轮询)"""
    t_end = time.time() + timeout
    mode = None
    while time.time() < t_end:
        tel = client.get('/api/v1/links/%s/telemetry' % link_id).json()
        mode = tel['flight_status']['mode']
        if mode == want:
            return mode
        time.sleep(0.5)
    return mode


def main():
    from px4.api.app import app

    fc = FakeFC('tcpin:127.0.0.1:%d' % PORT)
    th = threading.Thread(target=fc.run, daemon=True)
    th.start()
    time.sleep(0.3)

    ok = []
    with TestClient(app) as client:
        r = client.get('/health')
        assert r.status_code == 200 and r.json()['ok'], r.text
        ok.append('health')

        r = client.post('/api/v1/links', json={'id': 'gcs1', 'port': CONN,
                                               'baud': 57600})
        assert r.status_code == 201, r.text
        assert r.json()['connected'] is True, r.text
        ok.append('connect gcs1 (PX4)')

        r = client.post('/api/v1/links', json={'id': 'gcs1', 'port': CONN})
        assert r.status_code == 409, r.text
        ok.append('duplicate id rejected')

        r = client.post('/api/v1/links', json={'id': 'nope', 'port': 'tcp:127.0.0.1:1'})
        assert r.status_code == 502, r.text
        ok.append('bad link rejected')

        time.sleep(2.0)
        r = client.get('/api/v1/links/gcs1/telemetry')
        assert r.status_code == 200, r.text
        tel = r.json()
        for sec in ('flight_status', 'gps', 'velocity_attitude',
                    'battery_system', 'control_signals', 'labels'):
            assert sec in tel, '缺少遥测段: %s' % sec
        assert tel['link_ok'] is True, tel
        assert tel['gps']['satellites'] == 14, tel['gps']
        assert tel['battery_system']['pos_flags'] == 1, tel['battery_system']
        assert tel['battery_system']['position_estimate']['ready'] is True, \
            tel['battery_system']['position_estimate']
        assert tel['flight_status']['mode'] == 'POSCTL', tel['flight_status']
        assert tel['labels']['rc'].startswith('1500'), tel['labels']
        ok.append('telemetry sections (pos ready, no EKF)')

        r = client.post('/api/v1/links/gcs1/guidance', json={
            'link_id': 'gcs1', 'lat': 31.2309, 'lon': 121.4742, 'alt': 15.0,
            'confidence': 90, 'track': True, 'auto_takeoff': False})
        assert r.status_code == 422, r.text
        ok.append('unarmed guidance rejected')

        r = client.post('/api/v1/guidance', json={
            'link_id': 'gcs1', 'lat': 31.2309, 'lon': 121.4742, 'alt': 12.0,
            'vx': 2.0, 'vy': 0.0, 'vz': 0.0, 'confidence': 90, 'track': True,
            'auto_takeoff': True, 'takeoff_alt': 12.0, 'rate_hz': 10})
        assert r.status_code == 200, r.text
        t_end = time.time() + 120
        while time.time() < t_end and fc.received['pos_target'] < 5:
            time.sleep(1.0)
        assert fc.received['pos_target'] >= 5, \
            '引导未开始发帧: armed=%s rel=%.1f' % (fc.armed, fc.rel)
        time.sleep(1.0)
        r = client.get('/api/v1/links/gcs1/guidance')
        g = r.json()
        assert g['active'] is True and g['state'] == 'running', g
        assert g['sent'] >= 5 and g['target']['lat'] == 31.2309, g
        assert g['armed'] is True, g
        mode = wait_mode(client, 'gcs1', 'OFFBOARD', timeout=30)
        assert mode == 'OFFBOARD', '引导未切入 OFFBOARD: %s' % mode
        g = client.get('/api/v1/links/gcs1/guidance').json()
        assert g['mode'] == 'OFFBOARD', g
        assert fc.main == 6, (fc.mode_name, fc.main)
        ok.append('guidance streaming with auto takeoff (%d frames)'
                  % fc.received['pos_target'])

        last = fc.last_pos_target
        assert last['frame'] == 6, last
        assert last['mask'] in (0x7F8, 0x7C0), last
        ok.append('MAV_FRAME_GLOBAL_RELATIVE_ALT_INT + mask 0x%03X'
                  % last['mask'])

        r = client.post('/api/v1/links/gcs1/guidance', json={
            'lat': 31.2310, 'lon': 121.4743, 'alt': 15.0,
            'confidence': 10, 'min_confidence': 50})
        assert r.status_code == 422, r.text
        ok.append('confidence filter')

        r = client.post('/api/v1/links/gcs1/guidance', json={
            'lat': 31.2310, 'lon': 121.4743, 'alt': 25.0,
            'alt_frame': 'amsl'})
        assert r.status_code == 200, r.text
        time.sleep(0.6)
        assert abs(fc.last_pos_target['alt'] - 15.0) < 3.0, fc.last_pos_target
        ok.append('amsl->relative conversion (%.1f m)'
                  % fc.last_pos_target['alt'])

        r = client.post('/api/v1/links/gcs1/stop')
        assert r.status_code == 200 and r.json()['mode'] == 'LOITER', r.text
        mode = wait_mode(client, 'gcs1', 'LOITER', timeout=15)
        assert mode == 'LOITER', '急停后未进入 LOITER: %s' % mode
        g = client.get('/api/v1/links/gcs1/guidance').json()
        assert g['active'] is False, g
        ok.append('emergency stop -> LOITER')

        r = client.post('/api/v1/links/gcs1/goto', json={
            'lat': 31.2310, 'lon': 121.4745, 'alt': 12.0, 'auto_takeoff': True})
        assert r.status_code == 202, r.text
        task = wait_task(client, 'gcs1', r.json()['task']['id'], timeout=120)
        assert task['status'] == 'success', task
        assert task['result']['arrived'] is True, task
        assert any('已到达目的地' in line for line in task.get('log', []))
        ok.append('goto (OFFBOARD position) with auto takeoff')

        r = client.post('/api/v1/links/gcs1/mission', json={
            'waypoints': [{'lat': 31.2312, 'lon': 121.4747, 'alt': 12.0},
                          {'lat': 31.2314, 'lon': 121.4749, 'alt': 12.0}],
            'end_action': 'none', 'auto_takeoff': False})
        assert r.status_code == 202, r.text
        task = wait_task(client, 'gcs1', r.json()['task']['id'], timeout=120)
        assert task['status'] == 'success', task
        ok.append('waypoint mission')

        r = client.post('/api/v1/links/gcs1/rtl')
        assert r.status_code == 200 and r.json()['mode'] == 'RTL', r.text
        armed = wait_armed(client, 'gcs1', False, timeout=180)
        assert armed is False, 'RTL 后未上锁: fc mode=%s rel=%.1f lat=%.6f ' \
                               'lon=%.6f' % (fc.mode_name, fc.rel, fc.lat, fc.lon)
        ok.append('RTL -> landed & disarmed')

        time.sleep(2.0)
        r = client.post('/api/v1/links/gcs1/mode', json={'mode': 'POSCTL'})
        assert r.status_code == 202, r.text
        task = wait_task(client, 'gcs1', r.json()['task']['id'], timeout=30)
        assert task['status'] == 'success' and task['result']['mode'] == 'POSCTL', \
            task
        ok.append('mode switch POSCTL')

        r = client.post('/api/v1/links/gcs1/arm', json={'mode': 'POSCTL'})
        assert r.status_code == 202, r.text
        task = wait_task(client, 'gcs1', r.json()['task']['id'], timeout=60)
        assert task['status'] == 'success', task
        assert task['result']['armed'] is True, task
        ok.append('arm POSCTL')

        r = client.post('/api/v1/links/gcs1/takeoff',
                        json={'alt': 8.0, 'mode': 'ALTCTL'})
        assert r.status_code == 202, r.text
        task = wait_task(client, 'gcs1', r.json()['task']['id'], timeout=90)
        assert task['status'] == 'success', task
        assert task['result']['mode'] == 'ALTCTL', task
        t_end = time.time() + 30
        rel = 0.0
        while time.time() < t_end:
            rel = client.get('/api/v1/links/gcs1/telemetry').json()[
                'gps']['rel_alt'] or 0.0
            if rel >= 7.0:
                break
            time.sleep(0.5)
        assert rel >= 7.0, 'ALTCTL 起飞未到位 rel=%.1f' % rel
        ok.append('takeoff ALTCTL (rel=%.1f m)' % rel)

        r = client.post('/api/v1/links/gcs1/land')
        assert r.status_code == 200 and r.json()['mode'] == 'LAND', r.text
        armed = wait_armed(client, 'gcs1', False, timeout=120)
        assert armed is False, 'LAND 后未上锁: fc mode=%s rel=%.1f' % (
            fc.mode_name, fc.rel)
        ok.append('LAND -> disarmed')

        r = client.delete('/api/v1/links/gcs1')
        assert r.status_code == 200, r.text
        r = client.get('/api/v1/links/gcs1/telemetry')
        assert r.status_code == 404, r.text
        ok.append('disconnect')

    print('\n✅ 冒烟测试通过:')
    for s in ok:
        print('   - %s' % s)
    return 0


if __name__ == '__main__':
    sys.exit(main())
