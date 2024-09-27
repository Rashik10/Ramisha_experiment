xhost +

docker run -it --rm --net=host --privileged \
    -e DISPLAY=$DISPLAY \
    -v /home/erablab/.Xauthority:/root/.Xauthority \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd)/../linked_volumes/test_pkg:/ros2_ws/src/test_pkg \
    -v $(pwd)/../linked_volumes/init_and_run:/init_and_run \
    -v $(pwd)/../linked_volumes/run_experiment:/run_experiment \
    -v $(pwd)/../linked_volumes/run_simulation:/run_simulation \
    passive_teleoperation \
    /bin/bash
