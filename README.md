# olin-aerial-robotics-chess
An Olin Aerial Robotics Team project to develop an autonomous chess-playing drone.

## Build Instructions

```bash
# Clone this repository into a working catkin_ws of a ROS Kinetic install, then...

roscd iarc_main/.. && rosdep install -iy --from-paths .

sudo apt-get install python python-all-dev python-pip build-essential swig git libpulse-dev libasound2-dev

sudo apt-get install portaudio19-dev python-all-dev python3-all-dev && sudo pip install pyaudio

roscd iarc_main/.. && pip install --user -r requirements.txt
```


## Run Instructions

```bash
roslaunch iarc_main onboarding_ardrone.launch
```
