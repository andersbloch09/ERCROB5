container_name="ros_realsense"
image_name="ros-realsense"
network_name="ros_network"

# Stop any of the 3 containers if running
#RUNNING_CONTAINERS=$( docker container ls -a -q --filter ancestor=realsense_image )
#if [ -n "$RUNNING_CONTAINERS" ]; then
#    docker rm -f "$RUNNING_CONTAINERS"
#fi

# Build docker image from Dockerfile in directory 
#directory=$( dirname "$0" )
#docker buildx -t $image_name $directory

docker run -it \
    --network="${network_name}" \
    -e ROS_MASTER_URI=http://172.19.0.2:11311 \
    -v "$PWD/realsense.launch:/ros.launch:ro" \
    --privileged \
    -v "/dev:/dev" \
    --name "${container_name}" \
    "${image_name}" \
    bash