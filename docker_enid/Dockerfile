FROM navigoat:dev

RUN apt-get update && apt-get install -y \
    python3-venv \
    python3-pip \
    git \
    libgl1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Keep access to system ROS 2 Python packages
RUN python3 -m venv --system-site-packages /opt/people_env

ENV PATH="/opt/people_env/bin:${PATH}"

RUN pip install --no-cache-dir --upgrade pip

# CUDA 13.2 PyTorch for JetPack 7.2
RUN pip install --no-cache-dir \
    torch torchvision \
    --index-url https://download.pytorch.org/whl/cu132

# Detection and tracking dependencies
RUN pip install --no-cache-dir \
    "numpy<2" \
    scipy \
    ultralytics \
    git+https://github.com/ultralytics/CLIP.git

WORKDIR /workspace

# Orbbec Femto Bolt ROS 2 driver
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-orbbec-camera \
    ros-jazzy-orbbec-description \
    usbutils \
    && rm -rf /var/lib/apt/lists/*
