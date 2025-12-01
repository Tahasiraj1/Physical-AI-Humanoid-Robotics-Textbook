---
id: workspace-overview
title: Workspace Overview
sidebar_position: 5
description: Conceptual overview of ROS 2 workspace structure and organization without detailed installation steps.
tags: [ros2, workspace, structure, organization]
learning_objectives: []
---

# Workspace Overview

Understanding ROS 2 workspace structure helps you conceptualize how ROS 2 projects are organized. This section provides a brief overview of workspace concepts without detailed installation procedures.

## What is a ROS 2 Workspace?

A **ROS 2 workspace** is a directory structure where you organize your ROS 2 code, packages, and build artifacts. Think of it as your project's home directory—everything related to your robot's software lives here.

## Workspace Structure

A typical ROS 2 workspace has this structure:

```
workspace/
├── src/                    # Source code directory
│   └── my_robot_package/   # Your ROS 2 packages
│       ├── package.xml     # Package metadata
│       ├── setup.py        # Python package setup
│       └── my_robot_package/
│           └── nodes/      # Your node implementations
├── build/                  # Build artifacts (generated)
├── install/                # Installed packages (generated)
└── log/                    # Build logs (generated)
```

### Key Components

- **`src/`** - Contains your source code and ROS 2 packages
- **`build/`** - Intermediate build files (created during compilation)
- **`install/`** - Installed packages ready to use (created after building)
- **`log/`** - Build and runtime logs

## Packages

A **package** is the fundamental unit of organization in ROS 2. Each package:

- Contains related functionality (e.g., sensor drivers, control algorithms)
- Has a `package.xml` file that describes the package
- Can contain nodes, libraries, configuration files, and more
- Can depend on other packages

### Package Structure Example

```
my_sensor_package/
├── package.xml          # Package metadata and dependencies
├── setup.py             # Python package configuration
└── my_sensor_package/
    ├── __init__.py
    └── nodes/
        └── camera_node.py  # Your node implementation
```

## Build System

ROS 2 uses a build system to:

- **Compile** your code (for C++ packages)
- **Install** Python packages
- **Resolve dependencies** between packages
- **Generate** message and service definitions

The build process creates the `build/` and `install/` directories automatically.

## Workspace Concepts

### Source Space (`src/`)

This is where you write your code. You create packages here, and each package contains:

- Node implementations
- Message/service/action definitions
- Configuration files
- Launch files (for starting multiple nodes)

### Install Space (`install/`)

After building, ROS 2 installs packages to the `install/` directory. This is where:

- Executable nodes are located
- Python packages are installed
- Message/service/action definitions are available
- Configuration files are accessible

### Environment Setup

To use a workspace, you "source" it, which makes the installed packages available to ROS 2. This is typically done with:

```bash
source install/setup.bash  # For bash
# or
source install/setup.zsh  # For zsh
```

This command sets up environment variables so ROS 2 can find your packages.

## Python Package Example

For Python packages in ROS 2:

```python
# setup.py - Package configuration
from setuptools import setup
from glob import glob

setup(
    name='my_sensor_package',
    version='0.0.1',
    packages=['my_sensor_package'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/my_sensor_package']),
        ('share/my_sensor_package', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'camera_node = my_sensor_package.nodes.camera_node:main',
        ],
    },
)
```

This configuration:
- Defines the package name and version
- Specifies which Python modules are included
- Defines entry points (executable nodes)
- Lists dependencies

## Workspace Organization Best Practices

1. **One package per functionality** - Keep packages focused and modular
2. **Clear naming** - Use descriptive package and node names
3. **Dependency management** - Declare dependencies in `package.xml`
4. **Version control** - Track your `src/` directory, ignore `build/`, `install/`, `log/`

## Conceptual Understanding

For this module, the key concepts to understand are:

- **Workspace** = Your project directory
- **Package** = A unit of related functionality
- **Node** = An executable process (lives in a package)
- **Build** = Process that prepares your code for execution
- **Install** = Where ROS 2 finds your built packages

You don't need to know the exact build commands or installation procedures yet—those will be covered in later modules focused on hands-on development.

## Key Takeaways

1. **Workspace** organizes your ROS 2 code and packages
2. **Packages** contain related functionality and nodes
3. **Build system** compiles and installs your packages
4. **Install space** is where ROS 2 finds your packages at runtime
5. **Modular organization** makes complex robot systems manageable

## Next Steps

You've now completed the core content of Module 1! You understand:

- ROS 2's role as the robotic nervous system
- Core concepts: nodes, topics, services, actions
- Communication patterns and when to use each
- How ROS 2 applies to humanoid robotics
- Workspace structure concepts

Review the [Glossary](./glossary.md) for quick reference, or proceed to Module 2 (when available) to dive deeper into ROS 2 development.

