# GitHub Copilot Instructions for ROS2 Workspace

## Workspace Overview
This is a **ROS2 workspace** running in a dev container with Ubuntu 22.04.5 LTS. All development should follow ROS2 conventions and use ROS2 tooling exclusively.

## Critical ROS2 Workspace Rules

### 1. Workspace Structure
- **Root Directory**: Consider the root of this repository as the workspace root
- **Source Packages**: All ROS2 packages MUST be placed in `src/` directory
- **Build System**: Use `colcon` build system exclusively at the workspace root
- **Package Structure**: Each package in `src/` should have its own `README.md`

### 2. Build and Development Commands
**Always run these commands from the workspace root:**

```bash
# Build the entire workspace
colcon build

# Build specific package
colcon build --packages-select <package_name>

# Test the workspace
colcon test

# Test specific package
colcon test --packages-select <package_name>

# Source the workspace after building
source install/setup.bash
```

### 3. Python Environment Rules
- **NO virtual environments** (venv, conda, etc.)
- Use ROS2's built-in Python environment management
- Python packages should be managed through `package.xml` dependencies
- Use `rosdep` for system dependencies: `rosdep install --from-paths src --ignore-src -r -y`

### 4. Package Creation
When creating new ROS2 packages:

```bash
# For Python packages
ros2 pkg create --build-type ament_python <package_name>

# For C++ packages  
ros2 pkg create --build-type ament_cmake <package_name>

# Always create in src/ directory
cd src/
ros2 pkg create --build-type ament_python <package_name>
```

### 5. Package Structure Requirements
Each package in `src/` must have:
- `package.xml` - ROS2 package manifest
- `setup.py` or `CMakeLists.txt` - Build configuration
- `README.md` - Package-specific documentation
- Proper Python module structure for Python packages

### 6. Testing Guidelines
- Use `pytest` for Python packages (configured in `pytest.ini`)
- Use `colcon test` instead of running pytest directly
- Test files should be in `test/` directory within each package
- Integration tests should test ROS2 functionality (nodes, topics, services)

### 7. Development Workflow
1. Make changes to source code in `src/`
2. Build with `colcon build` from workspace root
3. Source the workspace: `source install/setup.bash`
4. Test with `colcon test`
5. Run nodes with `ros2 run <package_name> <executable_name>`

### 8. Common ROS2 Development Patterns
- **Nodes**: Create ROS2 nodes using `rclpy` (Python) or `rclcpp` (C++)
- **Launch Files**: Place in `launch/` directory within packages
- **Parameters**: Use ROS2 parameter system, not environment variables
- **Topics/Services**: Follow ROS2 naming conventions
- **Packages**: Keep packages focused and modular

### 9. File Organization
```
src/
├── package_name/
│   ├── package.xml
│   ├── setup.py
│   ├── README.md
│   ├── package_name/
│   │   ├── __init__.py
│   │   └── *.py (Python modules)
│   ├── launch/
│   │   └── *.launch.py
│   ├── test/
│   │   └── test_*.py
│   └── resource/
```

### 10. Dependencies and Installation
- **System dependencies**: Add to `package.xml` and install with `rosdep`
- **Python dependencies**: Add to `package.xml`, not requirements.txt
- **Build dependencies**: Use `<build_depend>` in package.xml
- **Runtime dependencies**: Use `<exec_depend>` in package.xml

### 11. Debugging and Development
- Use `ros2 node list`, `ros2 topic list`, `ros2 service list` for introspection
- Use `ros2 launch` to start multiple nodes
- Use ROS2 logging system: `self.get_logger().info()` in Python nodes

## Important Notes
- This workspace is already configured for ROS2 development
- The dev container has all necessary ROS2 tools pre-installed
- Always work within the ROS2 ecosystem - don't bypass it with standalone Python scripts
- When in doubt, use ROS2 tools and follow ROS2 conventions

## Commands to Avoid
- `python -m venv` or `conda create` (No virtual environments)
- `pip install` for ROS2 dependencies (use package.xml instead)
- Running Python scripts directly (use `ros2 run` instead)
- Building outside the workspace root

## Getting Started
1. Navigate to workspace root
2. Install dependencies: `rosdep install --from-paths src --ignore-src -r -y`
3. Build workspace: `colcon build`
4. Source workspace: `source install/setup.bash`
5. Run your ROS2 nodes: `ros2 run <package> <node>`
