#!/bin/bash
openbox &
xterm -geometry 220x60 -title "FOROS ABB RAPID Mock Controller" \
      -bg "#1a0000" -fg "#ef4444" -fa "Monospace" -fs 10 \
      -e "python3 -u /app/server.py" &
wait
