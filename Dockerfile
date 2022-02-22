FROM kristoferb/spbase_ros2:galactic as build

ARG project_name=robotiq_2f_driver

# install any deps here and clone any external repos into the workspace
COPY ./robotiq_2f_msgs/ /build/src/robotiq_2f_msgs

# Copy the packages and only the rust package and cargo files to build the rust deps.
COPY ./$project_name/Cargo.* \
    ./$project_name/package.xml \
    ./$project_name/r2r_cargo.cmake \
    ./$project_name/CMakeLists.txt \
    /build/src/$project_name/
RUN . /opt/ros/$ROS_DISTRO/setup.sh &&\
    cd /build &&\
    mkdir ./src/$project_name/src/ &&\
    echo "fn main() {}" > ./src/$project_name/src/main.rs &&\
    colcon build

# Copy the rest of the rust code and colcon build again
COPY ./$project_name/ /build/src/$project_name/
RUN . /opt/ros/$ROS_DISTRO/setup.sh &&\
    cd /build &&\
    colcon build --packages-select $project_name


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
