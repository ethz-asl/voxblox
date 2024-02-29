FROM docker.io/osrf/ros:humble-desktop-full

ENV SHELL /bin/bash

# Create a non-root user
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID ${USERNAME} \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m ${USERNAME} \
    && mkdir /home/${USERNAME}/.config && chown $USER_UID:$USER_GID /home/${USERNAME}/.config

# Set up sudo
RUN apt-get update \
    && apt-get install -y sudo \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME}\
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && rm -rf /var/lib/apt/lists/*

# Dependencies
RUN apt-get update \
    && apt-get install -y \
        nano \
        protobuf-compiler \
        autoconf \
        rsync \
        libtool \
        libgoogle-glog-dev \
        gdb \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /home/ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
