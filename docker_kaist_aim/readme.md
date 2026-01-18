# How to Use docker_kaist_aim

This guide explains how to build and run the Docker environment for the TEAM_AIM project.

## 1. Build the Docker Image

From the /root/TEAM_AIM/docker_kaist_aim directory, run:

```bash
docker build -t kaist_aim .
```

Or, using docker-compose:

```bash
docker-compose build
```

## 2. Run the Docker Container

To start the container:

```bash
docker run -it --rm \
  --name kaist_aim_container \
  -v /root/TEAM_AIM:/root/TEAM_AIM \
  kaist_aim
```

Or, using docker-compose:

```bash
docker-compose up
```

## 3. Access the Container

If using docker-compose, you can attach to the running container with:

```bash
docker exec -it kaist_aim_container bash
```

## Notes
- Make sure Docker and docker-compose are installed on your system.
- The /root/TEAM_AIM directory is mounted for code access and persistence.
- Adjust volume and port mappings as needed for your use case.
