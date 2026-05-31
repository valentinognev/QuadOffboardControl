#!/bin/bash

CONTAINER_NAME="rl_cnn_v4_env"

# Check if the container exists
if ! docker ps -a --format '{{.Names}}' | grep -Eq "^${CONTAINER_NAME}\$"; then
    echo "Creating new container: $CONTAINER_NAME..."
    # -d runs it in the background so we can exec into it repeatedly
    docker run --name "$CONTAINER_NAME" \
        --gpus all \
        --ipc=host \
        -dt \
        -v "$(pwd):/workspace" \
        -w /workspace \
        pytorch/pytorch:latest \
        bash
fi

# Check if the container is stopped; if so, start it
if ! docker ps --format '{{.Names}}' | grep -Eq "^${CONTAINER_NAME}\$"; then
    echo "Starting container: $CONTAINER_NAME..."
    docker start "$CONTAINER_NAME"
fi

# Enter the container
echo "Entering container: $CONTAINER_NAME..."
docker exec -it "$CONTAINER_NAME" bash
