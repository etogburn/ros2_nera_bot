#!/bin/bash
set -e

REPO_NAME="ros2_nera_bot"
IMAGE="ghcr.io/etogburn/${REPO_NAME}:latest"

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

echo ">>> Adding user to docker group (you may need to log out/in)..."
sudo usermod -aG docker $USER

echo ">>> Checking Docker installation..."
docker --version
docker compose version

echo ">>> Pulling repo image: $IMAGE"
docker pull $IMAGE || true

echo ">>> Starting containers with docker-compose..."
docker compose up -d

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
