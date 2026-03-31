## ROS2 Workspace Template

This repository is a template for creating ROS2 packages with a CI workflow and devcontainer configuration.

### Getting Started

1. Use this repository as a template (top-right corner → **Use this template**) and specify your owner and package name.
2. Open the cloned repository in VSCode — it will prompt you to **Reopen in Container**.
3. You will be asked to select which version you want to run, use the instructions below to determine which version you need.
4. The devcontainer will install all workspace dependencies and build the workspace automatically.
5. Open the ports section to see which port noVNC is listening on.

| Version | When |
| ------- | ---- |
| Default (CPU-safe) | Recommended first choice. Uses no GPU passthrough and works across laptops, VMs, macOS, and Windows hosts. |
| Generic GPU (`/dev/dri`) | Use when your host exposes `/dev/dri` and you want hardware acceleration without NVIDIA runtime requirements. |
| NVIDIA GPU | Use when you have an NVIDIA GPU and [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#setting-up-nvidia-container-toolkit) installed. This profile layers generic GPU and NVIDIA-specific overrides. |

### Development

- Add ROS2 packages to the `src/` folder using `ros2 pkg create ...`
- The devcontainer runs as `root` using `lcas.lincoln.ac.uk/ros:humble` as the base image
- Dependencies listed in `package.xml` files are installed automatically via `rosdep`

### Useful Resources

- [Get Started with Dev Containers in VS Code](https://youtu.be/b1RavPr_878)
