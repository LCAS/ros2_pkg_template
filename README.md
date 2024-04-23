## ROS2 Package Template

This repository serves as a template for creating ROS2 packages, equipped with a comprehensive CI workflow and devcontainer configuration.

### Development Environment Setup

To begin development in a containerized environment:

1. **Fork and Clone Repository:**
   Fork this repository into your workspace and clone the forked repository into your environment. Use the following command:

   ```bash
   git clone https://github.com/<YOUR_WORKSPACE>/ros2_pkg_template.git
   ```

2. **Open in Visual Studio Code:**
   Open the cloned repository in VSCode. VSCode will prompt you to "Reopen in Container." Alternatively, you can use the command palette (`Ctrl+Shift+P`) and search for the "reopen in container" command.

   ![Reopen in Container](https://github.com/LCAS/ros2_pkg_template/assets/47870260/52b26ae9-ffe9-4e7c-afb9-88cee88f870f)

3. **Container Setup:**
   Once reopened in the container, VSCode will initiate the building process and pull all necessary dependencies. You can monitor the building log within VSCode.

   ![Devcontainer Log](https://github.com/LCAS/ros2_pkg_template/assets/47870260/4a01e140-972e-4f10-b866-acaabf6b4cfd)

4. **Verify Container Environment:**
   After the build completes, VSCode will connect to the container. You can verify that you are within the container environment.

   ![In Container](https://github.com/LCAS/ros2_pkg_template/assets/47870260/9efec878-5d83-4aed-a9d0-8a1cf6bbf655)

### Devcontainer Features

The devcontainer includes a light desktop interface. To utilize this feature:

1. **Configuration:**
   Add the following features to the devcontainer configuration:

   ```json
   "features": {
       "ghcr.io/LCAS/devcontainer-features/desktop-lite:1": {}
   },
   "forwardPorts": [6080],
   "portsAttributes": {
       "6080": {
           "label": "desktop"
       }
   }
   ```

2. **Accessing the Desktop Interface:**
   Open the user interface by navigating to the PORTS tab in VSCode, selecting port `6080`, and opening it in the browser.

   ![Open in Browser](https://github.com/LCAS/ros2_pkg_template/assets/47870260/b61f4c95-453b-4c92-ad66-5133c91abb05)

3. **Connecting to the Interface:**
   Click on "Connect" and use the password `vscode` to access the desktop interface.

   ![NoVNC](https://github.com/LCAS/ros2_pkg_template/assets/47870260/71246a4c-fd02-4196-b390-b18804f9cd4e)

### Enjoy Development!

By leveraging this setup, you can develop on a remote machine with a lightweight desktop interface. Magic! 

### References

1. [ros2-teaching-ws](https://github.com/LCAS/ros2-teaching-ws)
2. [Get Started with Dev Containers in VS Code](https://youtu.be/b1RavPr_878?si=ADepc_VocOHTXP55)
