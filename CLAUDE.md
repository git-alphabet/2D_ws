This is an omnidirectional quadruped robot autonomous navigation project based on ROS 2 Humble. All development works are conducted in an Ubuntu 22.04 Docker container.
Tech Stack
- System Environment: Ubuntu 22.04 (Docker Container)
- Middleware: ROS 2 Humble
- Build Tool: Underlying compilation relies on colcon, while dedicated project build scripts are adopted for most scenarios to support multi-branch building
- Programming Language: C++, Python
- Build Framework: CMake
- Development Method: Launch container via VS Code Containers with host machine code mounting
Common Development Commands
All project build, execution and debugging operations must be performed inside the Docker container. The core commands are listed below:
# Enter the project container (all build & run operations execute here)
docker exec -it <container_name> bash

# Execute quick build script
./scripts/quick_build.sh

# Start SLAM mapping mode
./scripts/mapping.sh

# Start relocation & navigation mode (without SLAM mapping)
./scripts/nav.sh
Core Constraints &amp; Specifications
- Container Isolation Rule: All Git operations are executed on the host machine. All other compilation, execution and debugging operations must be completed inside the container with thedocker exec prefix.
- Branch Isolation Rule: The .buildcache directory stores build artifacts of different branches to realize isolated build environments.
- Driver Priority Rule: All ODIN robot driver logics are subject to the source code under src/odin_ros_driver.
- Development Modification Process: Propose and confirm the modification plan before code changes. All speculative inferences must be marked with "Inferred". Consult the team for confirmation in case of any uncertainty.
- Content Restrictions: Prohibit output and submission of key files. No emojis are allowed in code and comments.
- Reading Priority: Read the project README.md first before development and follow the official project specifications.
Coding Standards
- All codes are written in English, and all comments are written in Chinese with uniform and standardized formatting.
- Fully understand the complete business logic and associated codes of the target module before any modification.
- Write unit tests for newly added functions in accordance with the existing project test framework and specifications.
- Use the official existing build scripts for automated building only. Do not repeatedly develop redundant build logic.
