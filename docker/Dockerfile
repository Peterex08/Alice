FROM ros:humble-ros-base

ENV LANG=C.UTF-8

# Instala pacotes
RUN apt update && apt upgrade -y && \
    apt install -y \
    sudo \
    ros-humble-desktop-full \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rqt* \
    liburdfdom-dev \
    ros-humble-urdf-tutorial \
    python3-colcon-common-extensions \
    build-essential \
    git \
    vim && \
    rm -rf /var/lib/apt/lists/*

# Usuário não root
ARG UID=1000
ARG GID=1000

RUN groupadd -g $GID rosuser && \
    useradd -m -s /bin/bash -u $UID -g $GID rosuser && \
    usermod -aG sudo rosuser && \
    echo "rosuser ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/rosuser && \
    chmod 440 /etc/sudoers.d/rosuser

# Configuração do bash para o usuário
RUN echo "source /opt/ros/humble/setup.bash" >> /home/rosuser/.bashrc && \
    chown rosuser:rosuser /home/rosuser/.bashrc

# Define o usuário padrão
USER rosuser

# Define o diretório de trabalho
WORKDIR /home/rosuser