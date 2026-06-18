# Autonomous Tunnel Robot: Obstacle Avoidance & Detection

This project was based on the [BARN Navigation Challenge](https://github.com/ccwss-maker/ccwss-barn-2025), with some enhancements to obstacle avoidance and the addition of obstacle detection using You Only Look Once (YOLO).

## Prerequisites
- Docker installation ([Install Docker Engine](https://docs.docker.com/engine/install/))

## Docker Setup

```
git clone https://github.com/GREYHAT262/manifold_ws.git
cd manifold_ws

docker build -t barn:noetic .

xhost +
docker run -it   
    --network host   
    --name barn   
    --gpus all   
    --privileged   
    --env DISPLAY=$DISPLAY   
    --env QT_X11_NO_MITSHM=1   
    --env NVIDIA_DRIVER_CAPABILITIES=all   
    --env NVIDIA_VISIBLE_DEVICES=all   
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw   
    barn:noetic 
```

## Getting Started

```
cd src/the-barn-challenge
python3 run_enhanced.py --world_idx 371 --gui
```

## Acknowledgements
Here's a list of the Github repos used:
- [ccwss-maker/ccwss-barn-2025](https://github.com/ccwss-maker/ccwss-barn-2025)
- [Livox-SDK/livox_laser_simulation](https://github.com/Livox-SDK/livox_laser_simulation)
- [pal-robotics/realsense_gazebo_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)
- [leggedrobotics/darknet_ros](https://github.com/leggedrobotics/darknet_ros)