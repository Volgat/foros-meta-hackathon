#!/bin/bash
# Start VNC server for FOROS ROS1 simulator
export DISPLAY=:1
mkdir -p /root/.vnc
echo "foros2024" | vncpasswd -f > /root/.vnc/passwd
chmod 600 /root/.vnc/passwd
cat > /root/.vnc/xstartup << 'EOF'
#!/bin/bash
export DISPLAY=:1
openbox &
xterm -geometry 220x60 -title "FOROS ROS1 Console" \
      -bg "#090d16" -fg "#06b6d4" -fa "Monospace" -fs 10 \
      -e "bash -c 'source /opt/ros/noetic/setup.bash; \
          echo -e \"\e[36m[FOROS ROS1 Simulator]\e[0m\"; \
          rostopic list 2>/dev/null || echo waiting...; \
          bash'" &
EOF
chmod +x /root/.vnc/xstartup
vncserver :1 -geometry 1280x720 -depth 24 -rfbport 5901 -rfbauth /root/.vnc/passwd -fg
