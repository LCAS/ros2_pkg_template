## ROS2 Workspace Template

This repository is a template for creating ROS2 packages with a CI workflow and devcontainer configuration.

### Getting Started

1. Use this repository as a template (top-right corner → **Use this template**) and specify your owner and package name.
2. Open the cloned repository in VSCode — it will prompt you to **Reopen in Container**.
3. The devcontainer will install all workspace dependencies and build the workspace automatically.

### Development

- Add ROS2 packages to the `src/` folder using `ros2 pkg create ...`
- The devcontainer runs as `root` using `ros:humble` as the base image
- Dependencies listed in `package.xml` files are installed automatically via `rosdep`

### References

- [Get Started with Dev Containers in VS Code](https://youtu.be/b1RavPr_878?si=ADepc_VocOHTXP55)
