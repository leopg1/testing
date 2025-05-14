FROM ros:humble

# Instalare dependențe
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Instalare pachete Python pentru hardware
RUN pip3 install RPi.GPIO gpiozero numpy

# Crearea workspace-ului
WORKDIR /workspace
COPY . /workspace

# Asigură-te că scriptul de pornire este executabil
RUN chmod +x /workspace/start_maze_robot.sh

# Compilare proiect
RUN . /opt/ros/humble/setup.sh && \
    cd /workspace && \
    colcon build --symlink-install

# Sursa ROS2 și workspace
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Comanda de pornire implicită
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && exec bash"]
