docker run \
    --name name_this_container \
    -it \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    --device /dev/video0 \
    djnighti/ucsd_robocar:x86
