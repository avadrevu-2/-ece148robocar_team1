docker run \
    --name name_this_container \
    -it \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    --device /dev/video0 \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    djnighti/ucsd_robocar:latest
