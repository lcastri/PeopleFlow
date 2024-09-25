# Function to check if Docker is running
check_docker_status() {
    if systemctl is-active --quiet docker; then
        echo "Docker already running!"
        return 0  # Docker is running
    else
        return 1  # Docker is not running
    fi
}

# Check Docker status
if ! check_docker_status; then
    # Docker is not running, start Docker
    echo "Starting Docker..."
    sudo systemctl start docker
    if [ $? -eq 0 ]; then
        echo "Docker started successfully."
    else
        echo "Failed to start Docker."
        exit 1
    fi
fi