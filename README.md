# drone
드론 동작 구현을 위한 repository
## catkin_ws/src
#### `drone_camera_node` : 카메라가 연결된 드론 동작 테스트를 위한 Node

- `image_subscriber.py` : 카메라 이미지를 받아와 시각화

- `drone_control.py` : 드론의 기본적인 동작과 카메라 이미지 시각화를 동시에 수행

- `object_detection.py` : 드론에서 Object Detection 수행

- `object_tracking.py` : 드론에서 Object Tracking 수행

#### `drone_operation` : 드론 기능 구현 및 연결을 위한 Node
- `main_node.py` : 최상위 Node

- `drone_camera.py` : detection-tracking-reidentification 테스트를 위한 Node

- `drone_controller.py` : 드론 arm, takeoff, land, disarm의 기능 Service call 제공을 위한 Node

- `object_detection.py` : 객체 detection 클래스

- `object_tracking.py` : 객체 tracking 클래스

- `object_reidentification.py` : 객체 reidentification 클래스

- `visualize.py` : detection 및 tracking의 결과 시각화 클래스

## drone
`drone_activate.py` : 드론 이착륙

`drone_location.py` : 드론 이착륙 및 특정 GPS로 이동

`drone_state.py` : 드론 상태 확인

# Use
## PX4_Autopilot
<https://docs.px4.io/main/en/simulation/ros_interface.html>
```sh
cd <PX4-Autopilot_clone>
DONT_RUN=1 make px4_sitl_default gazebo-classic
source ~/catkin_ws/devel/setup.bash
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
roslaunch px4 posix_sitl.launch
```
```sh
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

## catkin_ws
```sh
catkin build
source ~/catkin/devel/setup.bash
chmod +x ~/catkin_ws/src/drone_camera_node/scripts/image_subscriber.py
rosrun drone_camera_node image_subscriber.py
roslaunch drone_operation main_node.py
```
## Rviz
```sh
rosrun rviz rviz
```
