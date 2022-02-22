FROM kristoferb/spbase_ros2:galactic as build

# install any deps here and clone any external repos into the workspace

# Copy the packages and only the rust package and cargo files to build the rust deps.
COPY ./robotiq_2f_msgs/ /build/src/robotiq_2f_msgs
COPY ./robotiq_2f_driver/Cargo.* \
    ./robotiq_2f_driver/package.xml \
    ./robotiq_2f_driver/r2r_cargo.cmake \
    ./robotiq_2f_driver/CMakeLists.txt \
    /build/src/robotiq_2f_driver/
RUN . /opt/ros/$ROS_DISTRO/setup.sh &&\
    cd /build &&\
    mkdir ./src/robotiq_2f_driver/src/ &&\
    echo "fn main() {}" > ./src/robotiq_2f_driver/src/main.rs &&\
    colcon build

# Copy the rest of the rust code and colcon build again
COPY ./robotiq_2f_driver/ /build/src/robotiq_2f_driver/
RUN . /opt/ros/$ROS_DISTRO/setup.sh &&\
    cd /build &&\
    colcon build --packages-select robotiq_2f_driver


# Create a new core image without build tools
FROM kristoferb/spcore_ros2:galactic
COPY --from=build /build/install /install/

# Install some missing deps that is needed bu not included in core. Find them here:
# http://packages.ros.org/ros2/ubuntu/dists/focal/main/binary-amd64/Packages
# RUN apt-get update -qqy \
#  && apt-get install -qqy ros-galactic-geometry2 \
#  && rm -rf /var/lib/apt/lists/*

RUN sed --in-place --expression \
    '$isource "/install/setup.bash"' \
    /entrypoint.sh
