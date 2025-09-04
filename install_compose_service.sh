#!/bin/bash
set -e

# Service name
SERVICE_NAME="ros2-compose.service"
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"

# Determine the current folder (project folder)
PROJECT_DIR="$(pwd)"

echo "Creating systemd service file for ROS2 Docker Compose..."
echo "Project directory detected as: $PROJECT_DIR"

# Generate the .service content
SERVICE_CONTENT="[Unit]
Description=ROS2 Docker Compose Service
Requires=docker.service
After=docker.service

[Service]
WorkingDirectory=$PROJECT_DIR
ExecStart=/usr/bin/docker compose up -d
ExecStop=/usr/bin/docker compose down
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
"

# Write the service file
echo "$SERVICE_CONTENT" | sudo tee "$SERVICE_PATH" > /dev/null
sudo chmod 644 "$SERVICE_PATH"

# Reload systemd and enable service
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"
sudo systemctl start "$SERVICE_NAME"

echo "$SERVICE_NAME installed and started successfully."
echo "Check status with: sudo systemctl status $SERVICE_NAME"
echo "View logs with: sudo journalctl -u $SERVICE_NAME -f"
