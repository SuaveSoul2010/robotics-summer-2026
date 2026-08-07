Build from dockerfile:
cd ~/AllCode/jackal_ws
docker build -t jackal_ros_foxy .

Allow GUI access: 
xhost +local:docker

Create container: 
docker run -it \
    --name jackal_container \
    --network host \
    --privileged \
    --gpus all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v ~/AllCode/GitHub/robotics-summer-2026/jackal_ws:/jackal_ws \
    jackal_ros_foxy

Build workspace:
cd /jackal_ws

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

Sourcing: 
source install/setup.bash

Start container: 
docker start -ai jackal_container # or name

Another terminal:
docker exec -it jackal_container bash
