container_name="ros_melodic"
image_name="melodic_image"
network_name="ros_network"

# Stop any of the 3 containers if running
RUNNING_CONTAINERS=$( docker container ls -a -q --filter ancestor=melodic_image )
if [ -n "$RUNNING_CONTAINERS" ]; then
    docker rm -f "$RUNNING_CONTAINERS"
fi

# Build docker image from Dockerfile in directory 
directory=$( dirname "$0" )
docker build -t $image_name $directory

docker run \
    -it \
    -v /home/exomy/ExoMy_Software/Noetic:/catkin_ws/src \
    -v "/dev:/dev" \
    --privileged \
    --network "${network_name}" \
    -e ROS_MASTER_URI=http://172.19.0.2:11311\
    --name "${container_name}" \
    "${image_name}" \
    bash
    
