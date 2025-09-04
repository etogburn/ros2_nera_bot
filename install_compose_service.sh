#!/bin/bash
set -e

SERVICE_NAME="ros2-compose.service"
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"

echo "Installing $SERVICE_NAME..."

# Copy service file
if [ ! -f "$SERVICE_NAME" ]; then
    echo "Error: $SERVICE_NAME not found in current directory."
    exit 1
fi

sudo cp "$SERVICE_NAME" "$SERVICE_PATH"
sudo chmod 644 "$SERVICE_PATH"

# Reload systemd so it recognizes new service
sudo systemctl daemon-reload

# Enable service at boot
sudo systemctl enable "$SERVICE_NAME"

# Start service now
sudo systemctl start "$SERVICE_NAME"

echo "$SERVICE_NAME installed and started successfully."
echo "Check status with: sudo systemctl status $SERVICE_NAME"
echo "View logs with: sudo journalctl -u $SERVICE_NAME -f"
