#!/bin/bash

# Define color output
RED='\e[0;31m'
GREEN='\e[0;32m'
YELLOW='\e[1;33m'
NC='\e[0m' # No Color

# Define fixed parameters
ROS_VERSION="jazzy"
ARM_IMAGE="robosense/ac-ros2-sdk:jazzy-arm64"
X86_IMAGE="robosense/ac-ros2-sdk:jazzy-amd64"
ARM_CONTAINER="ac_sdk_jazzy_arm64"
X86_CONTAINER="ac_sdk_jazzy_amd64"

# Get the absolute path of the current working directory
WORKSPACE_DIR=$(pwd)/../../../../..

# Get current user information
HOST_USER=$(whoami)
HOST_UID=$(id -u)
HOST_GID=$(id -g)
HOST_GROUP=$(getent group "${HOST_GID}" | cut -d: -f1)

# Display help information
show_help() {
    echo -e "${GREEN}Super Sensor SDK Container Launch Tool${NC}"
    echo "Usage: $0 [option]"
    echo ""
    echo "Options:"
    echo "  help     - Show this help message"
    echo "  stop     - Stop current container"
    echo "  status   - Show container status"
    echo "  arm      - Use ARM architecture image"
    echo ""
    echo "Examples:"
    echo "  $0 arm        # Start container with ARM image"
    echo "  $0            # Start container with default image"
    echo "  $0 stop       # Stop all sdk compilation containers"
    echo "  $0 status     # Show container statuses"
}

# Check if Docker is installed
check_docker() {
    if ! command -v docker &> /dev/null; then
        echo -e "${RED}Error: Docker is not installed${NC}"
        exit 1
    fi
}

# Check and load Docker image
check_and_load_image() {
    local image_name=$1

    # Check existing image
    if ! docker image inspect "${image_name}" &> /dev/null; then
        echo -e "${YELLOW}Pulling image ${image_name}...${NC}"
        docker pull "${image_name}" || { echo -e "${RED}Failed to pull ${image_name}${NC}"; exit 1; }
    fi
}

# Start container
start_container() {
    local arch=$1
    local image_name container_name

    if [ "$arch" == "arm" ]; then
        image_name="$ARM_IMAGE"
        container_name="$ARM_CONTAINER"
    else
        image_name="$X86_IMAGE"
        container_name="$X86_CONTAINER"
    fi

    check_and_load_image $image_name

    # Check existing container
    if docker ps -a --format '{{.Names}}' | grep -q "^${container_name}$"; then
        if docker ps --format '{{.Names}}' | grep -q "^${container_name}$"; then
            echo -e "${YELLOW}Attaching to running container...${NC}"
            docker exec -it -u $HOST_USER $container_name /bin/bash
            return
        else
            echo -e "${YELLOW}Starting existing container...${NC}"
            docker start $container_name
            docker exec -it $container_name /bin/bash
            return
        fi
    fi

    # Create new container
    echo -e "${YELLOW}Creating new ${container_type} container...${NC}"
    
    common_args=(
        -it --rm --privileged
        -v "$WORKSPACE_DIR:/workspace"
        -v "/etc/timezone:/etc/timezone:ro"
        -v "/etc/localtime:/etc/localtime:ro"
        -v "/etc/passwd:/etc/passwd:ro"
        -v "/etc/group:/etc/group:ro"
        -v "/dev:/dev"
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw"
        --network host
        --name "$container_name"
        --user root
    )

    if [ "$container_type" = "cross" ]; then
        docker run "${common_args[@]}" --platform linux/arm64/v8 \
            $image_name /bin/bash -c "
                groupadd -g $HOST_GID $HOST_GROUP || true
                useradd -m -o -u $HOST_UID -g $HOST_GID -s /bin/bash $HOST_USER || true
                exec su $HOST_USER
            "
    else
        docker run "${common_args[@]}" -w /workspace \
            $image_name /bin/bash -c "
                groupadd -g $HOST_GID $HOST_GROUP || true
                useradd -m -o -u $HOST_UID -g $HOST_GID -s /bin/bash $HOST_USER || true
                exec su $HOST_USER
            "
    fi
}

# Stop container
stop_container() {
    local container_name=$1
    if docker ps -a --format '{{.Names}}' | grep -q "^${container_name}$"; then
        echo -e "${YELLOW}Stopping $container_name...${NC}"
        docker stop $container_name &> /dev/null
        docker rm $container_name &> /dev/null
    else
        echo -e "${YELLOW}Container $container_name not found${NC}"
    fi
}

# Show container status
show_status() {
    echo -e "\n${GREEN}Container Status:${NC}"
    docker ps -a --filter "name=_sdk_compile_" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
}

# Main entry point
main() {
    check_docker
    if [ -z "$1" ]; then
        arch=${2:-$(uname -m)}
        if [[ $arch == "x86_64" ]]; then
            arch="x86"
        elif [[ $arch == "aarch64" ]]; then
            arch="arm"
        fi
        start_container $arch
    else
        case $1 in
            arm)
                start_container "arm"
                ;;
                
            stop)
                stop_container "$ARM_CONTAINER"
                stop_container "$X86_CONTAINER"
                ;;
                
            status)
                show_status
                ;;
                
            -h|--help|help|*)
                show_help
                ;;
        esac
    fi
}

main "$@"