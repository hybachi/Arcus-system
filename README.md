# ARCUS - Autonomous Racing Car

ARCUS is a ROS 2 based autonomous car platform, inspired by the **F1Tenth** and designed for:

- Developing and testing autonomous driving algorithms.
- Studying robot control, navigation, and perception using a modular stack.
- Running simulations with **Gazebo Sim** and visualizing sensor data with **RViz**.

ARCUS is structured to support both **simulation** and **physical deployment** on the *ARCUS_R1*.


## Key Features
 
- **Modular Hardware Abstraction:** Uses `ros2_control` to separate hardware interfaces from controllers, making it easy to swap actuators, sensors, or control strategies.  
- **Sensor Integration:** Supports LiDAR, cameras, IMU, and other sensors for perception and navigation.  
- **Simulation and Testing:** Can run in simulations using Gazebo Sim, enabling safe testing of autonomous driving algorithms.  


## Development Environment

ARCUS development is containerized using **Docker** and **VS Code Devcontainers**, providing:

- A **consistent development environment** across all systems.
- Automatic setup of ROS 2 Jazzy, Gazebo Sim, and ROS 2 tools.
- GPU acceleration for simulation and perception pipelines via **NVIDIA Container Toolkit**.

The workspace is mounted inside the container at: `/home/arcus_ws`.
This ensures local changes are reflected immediately in the container.


## Requirements

To run ARCUS, the following tools are required:

- **Docker**: Containerization engine.
- **VS Code** with **Dev Containers extension**: For an easy development environment.
- **NVIDIA Container Toolkit**: For GPU acceleration inside containers.
- **NVIDIA GPU** (optional, for accelerated simulation and perception).
- **X11 server** (on Linux/macOS) to forward GUI applications like Gazebo and RViz.

> On Windows, using WSL2 with X server or Remote Desktop is recommended for GUI forwarding.


## Setup and Running

### 1. Install Prerequisites

- Docker: [https://docs.docker.com/get-docker/](https://docs.docker.com/get-docker/)  
- NVIDIA Container Toolkit: [https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)  
- VS Code: [https://code.visualstudio.com/](https://code.visualstudio.com/)  
- Dev Containers Extension: [https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### 2. Open Repository in Devcontainer

1. Clone repository and open in **VSCode**
2. When prompted, select **"Reopen in Container"**.  
   Or use Command Palette: `Dev Containers: Reopen in Container`
3. **VS Code** will build the devcontainer from the included `Dockerfile` and mount your workspace.  
   - The container is pre-configured with ROS 2 Jazzy, Gazebo Sim, and ROS 2 development tools.
   - GPU acceleration is enabled for simulation.

### 3. Connect Host Terminal to Container (Optional)

If you prefer using the host terminal:

```bash
docker ps  # find your devcontainer name
docker exec -it <container_name> bash
```
You can now run ROS 2 commands directly inside the container.

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
