from pymavlink import mavutil
import time

print('Wating for the drone to connect')
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print('Drone is connected!')

master.arducopter_arm()
print('Waiting for the drone to arm')
master.motors_armed_wait()
print('Armed!')

def check_system_health():
    # 시스템 상태 메시지 수신
    msg = master.recv_match(type='SYS_STATUS', blocking=True)
    msg = msg.to_dict()
    print(f"System status: {msg}")

    # 센서 상태 체크 (개별 센서)
    sensors_ok = (
        msg['onboard_control_sensors_present'] & 
        msg['onboard_control_sensors_enabled'] & 
        msg['onboard_control_sensors_health']
    )

    required_sensors = (
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO |
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS
    )

    if (sensors_ok & required_sensors) == required_sensors:
        print("All required sensors are healthy")
        return True
    else:
        print(sensors_ok)
        print("Sensor health issue detected")
        return False

if check_system_health():
    master.set_mode('MANUAL')

    #print('Start!')
    #master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    master.arducopter_arm()
    print('Waiting for the drone to arm')
    master.motors_armed_wait()
    print('Armed!')

    def takeoff(altitude):
        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)
        print('Taking off')

    takeoff(10)
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

else:
    print("System health issue detected, cannot arm the drone")

def calibrate_all_sensors():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
        0,
        1, # Accelerometer
        1, # Magnetometer
        1, # Ground Pressure
        0, 0, 0, 0)
    print("All sensors calibration requested")

# 캘리브레이션 요청 보내기
calibrate_all_sensors()

# 캘리브레이션 완료 대기
while True:
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()
    if ack_msg['command'] == mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION and ack_msg['result'] == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("All sensors calibration successful")
        break
    elif ack_msg['command'] == mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION:
        print("All sensors calibration failed")
        break

def check_sensor_health(present, enabled, health):
    required_sensors = [
        ('3D Gyro', mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO),
        ('3D Accel', mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL),
        ('Absolute Pressure', mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE),
        ('GPS', mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS)
    ]

    all_healthy = True
    for sensor_name, sensor_flag in required_sensors:
        present_status = bool(present & sensor_flag)
        enabled_status = bool(enabled & sensor_flag)
        health_status = bool(health & sensor_flag)
        
        if present_status and enabled_status and health_status:
            print(f"{sensor_name}: OK")
        else:
            all_healthy = False
            print(f"{sensor_name}: Issue detected (Present: {present_status}, Enabled: {enabled_status}, Health: {health_status})")
    
    return all_healthy

# 현재 시스템 상태 값을 이용하여 센서 상태 확인
present = 35684397
enabled = 35749933
health = 840990783

if check_sensor_health(present, enabled, health):
    print("All required sensors are healthy")
else:
    print("Some sensors have issues")

master.close()