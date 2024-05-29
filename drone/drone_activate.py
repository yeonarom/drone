'''
print('Wating for the drone to connect')
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print('Drone is connected!')

#master.set_mode('OFFBOARD')

master.arducopter_arm()
print('Waiting for the drone to arm')
master.motors_armed_wait()
print('Armed!')

def takeoff(altitude):
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)
    print('Taking off')

takeoff(20)
time.sleep(10)

master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
print('Landing')

while True:
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
        print("Landed")
        break

master.arducopter_disarm()
print('Waiting for the drom to disarm')
master.motors_disarmed_wait()
print('Disarmed!')

master.close()
'''

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

    mode_id = connection.mode_mapping()[mode][1]
    connection.mav.command_long_send(
        connection.target_system, 
        connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
        0,
        0, mode_id, 0, 0, 0, 0, 0
    )

    while True:
        ack_msg = connection.recv_match(type='COMMAND_ACK', blocking=True)
        if ack_msg and ack_msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            print(f"Mode changed to {mode}")
            break

# 드론의 현재 고도
def get_current_altitude(connection):
    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            current_alt = msg.relative_alt / 1000.0
            return current_alt

# 모터 활성 및 이륙 함수
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
    
    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg and msg.relative_alt / 1000 >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
    

# 착륙 함수
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

# 비행 모드 설정
print(connection.mode_mapping())
set_mode(connection, 'OFFBOARD')
#connection.set_mode('OFFBOARD')

# 이륙
current_altitude = get_current_altitude(connection)

if current_altitude is not None:
    print(f"Current altitude: {current_altitude} meters")
    target_altitude = max(current_altitude + 10, 10)
else:
    target_altitude = 10

arm_and_takeoff(connection, target_altitude)

# 호버링 시간 (10초)
time.sleep(10)

# 착륙
land(connection)

# 모터 비활성화
print('Waiting for the drone to disarm...')
connection.arducopter_disarm()
connection.motors_disarmed_wait()
print("Disarmed!")

connection.close()