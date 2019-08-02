FROM osrf/ros2:nightly
SHELL ["/bin/bash", "-c"]

RUN mkdir -p /opt/ros/dashing/perf_test_ws/src && \
  cd /opt/ros/dashing/perf_test_ws/src && \
  git clone https://github.com/ApexAI/performance_test.git

RUN apt-get update && \
  apt-get install -y \
    nano \
    tmux \
    libboost-all-dev \
    default-jre \
    python-pip && \
  python -m pip install \
    pandas

RUN echo "source /opt/ros/dashing/setup.bash" >> /etc/bash.bashrc && \
  cd /opt/ros/dashing/perf_test_ws && \
  source /opt/ros/dashing/setup.bash && \
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
  echo "source /opt/ros/dashing/perf_test_ws/install/setup.bash" >> /etc/bash.bashrc

RUN mkdir -p /opt/ros/dashing/secTest/src && \
  apt-get install -y -q \
    libgtest-dev \
    googletest

ADD . /opt/ros/dashing/secTest/src
RUN cd /opt/ros/dashing/secTest && \
  source /opt/ros/dashing/setup.bash && \  
  colcon build --cmake-args -DSECURITY=ON && \
  echo "source /opt/ros/dashing/secTest/install/setup.bash"

WORKDIR /opt/ros/dashing/
ENTRYPOINT /bin/bash
CMD /bin/bash -l
