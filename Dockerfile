from sunxiaojay123/10.2-cudnn7-devel-ubuntu18.04-pytorch1.7.1:v1

env NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES},display

workdir /viper

RUN rm /etc/apt/sources.list.d/cuda.list
RUN rm /etc/apt/sources.list.d/nvidia-ml.list

run apt-get update --yes
run apt-get install --yes \
    g++ \
    cmake \
    xorg-dev \
    libboost-all-dev \
    libglew-dev \
    libcgal-dev \
    libtbb-dev \
    libgl1-mesa-glx \
    libgl1-mesa-dev \
    libgl1-mesa-dri
