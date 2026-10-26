#!/bin/bash
# xstartup for ROS 2 Mock Controller VNC
openbox &
xterm -geometry 220x60 \
      -title "FOROS ROS 2 Mock Controller — ros2bridge node" \
      -bg "#0d0d0d" -fg "#a855f7" \
      -fa "Monospace" -fs 10 \
      -e "python3 -u /app/server.py" &
wait
