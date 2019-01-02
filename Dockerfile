FROM ros:kinetic-ros-base

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-kinetic-gazebo-* \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get upgrade -y 
# Install vi editor
RUN apt-get update
RUN apt-get install vim -y

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-kinetic-perception=1.3.2-0* \
    && rm -rf /var/lib/apt/lists/*
# setup working directory
# ADD /App /App
WORKDIR /App

CMD /usr/bin/firefox


# nvidia-docker hooks
#LABEL com.nvidia.volumes.needed="nvidia_driver"
#ENV PATH /usr/local/nvidia/bin:${PATH}
#ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

CMD [ "gazebo" ]
