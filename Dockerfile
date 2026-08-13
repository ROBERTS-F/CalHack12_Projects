# ---- Base: Python + build tools ----
FROM python:3.11-slim

ENV DEBIAN_FRONTEND=noninteractive
ARG UID=1000
ARG GID=1000
ARG USER=dev

# OS deps for MuJoCo + builds + serial
RUN apt-get update && apt-get install -y --no-install-recommends \
    git curl ca-certificates \
    build-essential cmake pkg-config \
    libgl1 libgl1-mesa-dev libglu1-mesa libosmesa6 libosmesa6-dev \
    libglfw3 libglfw3-dev libglew-dev freeglut3-dev \
    libx11-6 libxext6 libxrender1 libxrandr2 libxi6 libxinerama1 libxcursor1 \
    udev usbutils \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user with dialout-like permissions for serial
RUN groupadd -g ${GID} ${USER} && useradd -m -u ${UID} -g ${GID} ${USER}
USER ${USER}
WORKDIR /workspace

# ---- Python deps ----
# Split to speed up caching
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# MuJoCo needs MUJOCO_GL set:
# - "egl" (headless on GPU/servers), "osmesa" (portable headless), or "glfw" (GUI, Linux/X11)
ENV MUJOCO_GL=osmesa

# Add convenient tools
RUN pip install --no-cache-dir ptpython ipython

# Default command: just drop into bash
CMD ["/bin/bash"]
