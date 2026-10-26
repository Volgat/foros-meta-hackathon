#!/bin/bash
openbox &
xterm -geometry 220x60 -title "FOROS Fanuc Karel Mock Controller" \
      -bg "#1a1400" -fg "#eab308" -fa "Monospace" -fs 10 \
      -e "python3 -u /app/server.py" &
wait
