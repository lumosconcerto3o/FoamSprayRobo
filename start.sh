#!/bin/bash

# Build the Docker image
docker-compose build

# Start the container in detached mode
docker-compose up -d

# Attach to the container
docker exec -it 22lts bash
