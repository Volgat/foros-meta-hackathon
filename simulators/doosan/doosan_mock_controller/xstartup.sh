#!/bin/bash
openbox &
xterm -geometry 220x60 -title "FOROS Doosan DRL Mock Controller" \
      -bg "#000d1a" -fg "#3b82f6" -fa "Monospace" -fs 10 \
      -e "python3 -u /app/server.py" &
wait
