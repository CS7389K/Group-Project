FROM nvcr.io/nvidia/pytorch:23.10-py3

ENV DEBIAN_FRONTEND=noninteractive
ENV CUDA_HOME=/usr/local/cuda
ENV PYTHONUNBUFFERED=1

# System dependencies
RUN apt-get update && apt-get install -y \
    git wget vim htop nvtop tmux \
    build-essential cmake \
    libopencv-dev libglib2.0-0 libsm6 libxext6 libxrender-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages in correct order
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir \
    "numpy<1.25" \
    scipy \
    pandas \
    scikit-learn \
    && pip install --no-cache-dir \
    Pillow \
    opencv-python \
    timm \
    einops \
    && pip install --no-cache-dir \
    transformers \
    accelerate \
    bitsandbytes \
    && pip install --no-cache-dir \
    ultralytics \
    psutil \
    gputil \
    pynvml

WORKDIR /workspace
COPY . /workspace/
CMD ["/bin/bash"]
