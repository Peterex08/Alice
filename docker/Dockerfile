FROM ros:humble-ros-base

ENV LANG=C.UTF-8

# ros-humble-desktop-full
RUN apt update && apt upgrade -y && \
    apt install -y \
    ros-humble-desktop-full \
    ros-humble-rqt* \
    python3-colcon-common-extensions \
    build-essential \
    git \
    vim && \
    rm -rf /var/lib/apt/lists/*

# Sourcing do setup.bash
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Usuário não root
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID rosuser && \
    useradd -m -s /bin/bash -u $UID -g $GID rosuser

# Define o usuário padrão
USER rosuser

# Define o diretório de trabalho
WORKDIR /home/rosuser