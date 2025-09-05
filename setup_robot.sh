#!/bin/bash
set -e

REPO_NAME="ros2_nera_bot"
IMAGE="ghcr.io/etogburn/${REPO_NAME}:robot-latest"

echo ">>> Updating apt and installing prerequisites..."
# First install tools needed to refresh keys
sudo apt-get update -y || true
sudo apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg \
    lsb-release \
    git

echo ">>> Fixing GPG keys for ROS 2 and RealSense (if needed)..."

# Refresh ROS 2 repo key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg || true

# Import Intel RealSense repo key (only if repo is configured)
# if grep -q "librealsense.intel.com" /etc/apt/sources.list /etc/apt/sources.list.d/* 2>/dev/null; then
#     sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key C8B3A55A6F3EFCDE || true
# fi

echo ">>> Running apt update/upgrade..."
sudo apt-get update -y || true
sudo apt-get upgrade -y

echo ">>> Installing Docker..."
# Add Docker’s official GPG key
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | \
    sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Add Docker repository
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
  https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update -y

# Install Docker Engine + Compose plugin
sudo apt-get install -y \
    docker-ce \
    docker-ce-cli \
    containerd.io \
    docker-buildx-plugin \
    docker-compose-plugin

echo ">>> Enabling Docker service..."
sudo systemctl enable docker
sudo systemctl start docker

echo ">>> Adding user to docker group (you may need to log out/in)..."
sudo usermod -aG docker $USER

echo ">>> Checking Docker installation..."
docker --version
docker compose version#!/bin/bash
set -e

REPO_NAME="ros2_nera_bot"
IMAGE="ghcr.io/etogburn/${REPO_NAME}:robot-latest"

echo ">>> Updating apt and installing prerequisites..."
sudo apt-get update -y
sudo apt-get upgrade -y
sudo apt-get install -y \
    ca-certificates \
    curl \
    gnupg \
    lsb-release \
    git

echo ">>> Installing Docker..."
# Add Docker’s official GPG key
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Add Docker repository
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update -y

# Install Docker Engine + Compose plugin
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

echo ">>> Enabling Docker service..."
sudo systemctl enable docker
sudo systemctl start docker

echo ">>> Ensuring user is in docker group..."
if ! groups $USER | grep -q "\bdocker\b"; then
    echo ">>> Adding $USER to docker group..."
    sudo usermod -aG docker $USER
    echo ">>> You must log out and back in (or run 'newgrp docker') for group changes to take effect."
    NEED_SUDO=1
else
    NEED_SUDO=0
fi

echo ">>> Checking Docker installation..."
docker --version
docker compose version

echo ">>> Pulling repo image: $IMAGE"
if [ "$NEED_SUDO" -eq 1 ]; then
    sudo docker pull $IMAGE || true
else
    docker pull $IMAGE || true
fi

echo ">>> Starting containers with docker-compose..."
if [ "$NEED_SUDO" -eq 1 ]; then
    sudo docker compose up -d
else
    docker compose up -d
fi

# Call the service installer
if [ -f install_compose_service.sh ]; then
    echo ">>> Installing systemd service..."
    chmod +x install_compose_service.sh
    if [ "$NEED_SUDO" -eq 1 ]; then
        sudo ./install_compose_service.sh
    else
        ./install_compose_service.sh
    fi
else
    echo ">>> install_compose_service.sh not found, skipping service installation."
fi

echo ">>> Setup complete!"
echo "Next steps:"
echo "  - If this was your first run, log out and back in so your user is added to the docker group."
echo "  - Use 'docker ps' to check running containers."
echo "  - Systemd service is installed and will auto-start containers on boot."


echo ">>> Pulling repo image: $IMAGE"
docker pull $IMAGE || true

echo ">>> Starting containers with docker-compose..."
docker compose up -d || true

# Call the service installer
if [ -f install_compose_service.sh ]; then
    echo ">>> Installing systemd service..."
    chmod +x install_compose_service.sh
    ./install_compose_service.sh
else
    echo ">>> install_compose_service.sh not found, skipping service installation."
fi

echo ">>> Setup complete!"
echo "Next steps:"
echo "  - Log out and back in so your user is added to the docker group."
echo "  - Use 'docker ps' to check running containers."
echo "  - Systemd service is installed and will auto-start containers on boot."
