import time
from pymavlink import mavutil

# 드론에 연결
print('Wating for the drone to connect...')
connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# 드론의 상태 확인
connection.wait_heartbeat()
print('Drone is connected!')

# 비행 모드 설정 함수
def set_mode(connection, mode):
    if mode not in connection.mode_mapping():
        print(f"Unknown mode: {mode}")
        return

    mode_id = connection.mode_mapping()[mode]
    connection.mav.command_long_send(
        connection.target_system, 
        connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
        0,
        0, mode_id, 0, 0, 0, 0, 0
    )

    while True:
        ack_msg = connection.recv_match(type='COMMAND_ACK', blocking=True)
        if ack_msg and ack_msg['command'] == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            print(f"Mode changed to {mode}")
            break

# 현재 위치 확인 함수
def get_current_position(connection):
    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            latitude = msg.lat / 1e7
            longitude = msg.lon / 1e7
            altitude = msg.relative_alt / 1000.0
            return latitude, longitude, altitude

# 이륙 및 착륙 함수
def arm_and_takeoff(connection, target_altitude):
    print('Waiting for the drone to arm...')
    connection.arducopter_arm()
    connection.motors_armed_wait()
    print("Armed!")

    print(f"Taking off to {target_altitude} meters")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, target_altitude
    )
    time.sleep(5)
    '''
    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg and msg.relative_alt / 1000 >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
    '''

def land(connection):
    print("Landing")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )

    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg and msg.relative_alt / 1000 <= 0.1:
            print("Landed")
            break
        time.sleep(1)

# 특정 위치로 이동 함수
def goto_position_target_global_int(connection, latitude, longitude, altitude):
    # MAV_CMD_NAV_WAYPOINT 명령을 사용하여 특정 위치로 이동
    connection.mav.mission_item_send(
        connection.target_system,
        connection.target_component,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2, 0, 0, 0, 0, 0,
        latitude,
        longitude,
        altitude
    )

    print(f"Moving to position: lat={latitude}, lon={longitude}, alt={altitude}")
    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            current_alt = msg.relative_alt / 1000
            print(f"Current position: lat={current_lat}, lon={current_lon}, alt={current_alt}")
            if (abs(current_lat - latitude / 1e7) < 0.00001 and
                abs(current_lon - longitude / 1e7) < 0.00001 and
                abs(current_alt - altitude) < 0.1):
                print("Reached target position")
                break
        time.sleep(1)

# 비행 모드 설정
set_mode(connection, 'OFFBOARD')

# 이륙
current_lat, current_lon, current_alt = get_current_position(connection)
print(current_lat, current_lon, current_alt)

if current_alt is not None:
    print(f"Current altitude: {current_alt} meters")
    target_alt = max(current_alt + 5, 10)
else:
    target_alt = 10

arm_and_takeoff(connection, target_alt)

# 특정 GPS 위치로 이동 (예: 37.7749 N, -122.4194 W, 10m)
target_lat = current_lat * 1e7 + 500
target_lon = current_lon * 1e7 + 500
print(target_lat, target_lon, target_alt)
goto_position_target_global_int(connection, target_lat, target_lon, target_alt)

# 호버링 시간 (10초)
time.sleep(10)

# 착륙
land(connection)

# 모터 비활성화
print("Disarming motors")
connection.arducopter_disarm()
connection.motors_disarmed_wait()
print("Motors disarmed")

connection.close()