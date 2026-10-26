# =============================================================================
#  FOROS KUKA Mock Controller Container
#  Python-based KRL interpreter + VNC terminal + noVNC
# =============================================================================
FROM python:3.11-slim

ENV DEBIAN_FRONTEND=noninteractive \
    DISPLAY=:1 \
    VNC_PORT=5902 \
    NOVNC_PORT=6083

RUN apt-get update && apt-get install -y --no-install-recommends \
    tigervnc-standalone-server tigervnc-common tigervnc-tools \
    xterm openbox \
    websockify novnc \
    supervisor \
    curl \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY server.py /app/server.py

# VNC startup
RUN mkdir -p /root/.vnc /root/.config/tigervnc && \
    echo "foros2024" | vncpasswd -f > /root/.vnc/passwd && \
    cp /root/.vnc/passwd /root/.config/tigervnc/passwd && \
    chmod 600 /root/.vnc/passwd /root/.config/tigervnc/passwd

COPY xstartup.sh /root/.vnc/xstartup
RUN cp /root/.vnc/xstartup /root/.config/tigervnc/xstartup && \
    chmod +x /root/.vnc/xstartup /root/.config/tigervnc/xstartup

COPY supervisord.conf /etc/supervisor/conf.d/kuka.conf

EXPOSE 5902 6083 7000 8766

CMD ["/usr/bin/supervisord", "-n", "-c", "/etc/supervisor/supervisord.conf"]
