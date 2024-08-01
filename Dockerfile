#BASE IMAGE
FROM ubuntu:20.04

#DEPENDENCIES DEC
RUN apt-get update && apt-get install -y \
    #DEPENDENCIES LIST
    python3 \
    python3-pip \
    gcc-9 \
    g++-9 \
    libgl1-mesa-glx \
    libosmesa6-dev \
    patchelf \
    && rm -rf /var/lib/apt/lists/*

#DEFINE DEFAULT COMPILER (gcc9 for mujoco)
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 100 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 100

RUN pip3 install --upgrade pip \
    && pip3 install \
    numpy \
    cffi \
    glfw

#LOGISIC SETTING
COPY . /app
WORKDIR /app
RUN pip3 install mujoco-py

#EXECUTION COMMAND
CMD ["python3", "your_script.py"]
