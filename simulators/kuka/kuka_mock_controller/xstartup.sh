#!/bin/bash
# xstartup for KUKA Mock Controller VNC
openbox &
xterm -geometry 220x60 \
      -title "FOROS KUKA Mock Controller — KRL Interpreter" \
      -bg "#0d0d0d" -fg "#f97316" \
      -fa "Monospace" -fs 10 \
      -e "python3 -u /app/server.py" &
wait
