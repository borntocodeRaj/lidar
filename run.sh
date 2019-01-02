#docker run -d -it kineticros:latest /bin/bash
docker run -it --rm --user root -e DISPLAY=$DISPLAY -v/tmp/.X11-unix:/tmp/.X11-unix kineticros
