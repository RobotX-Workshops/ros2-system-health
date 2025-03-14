# ROS2 System Health

A package containing a node for monitoring the system health of the robot computer

## Including as a Subrepository (Subrepo)

To include this package in a parent ROS2 workspace:

Add as a submodule:

```bash
cd ~/<your_workspace>/src
git submodule add <this-repo-url>  system_health
```

Install dependencies:

```bash
cd ~/<your_workspace>
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Build the parent workspace:

```bash
cd ~/<your_workspace>
colcon build --symlink-install
source install/setup.bash
```

## Usage
TODO:

### Parameters
TODO:

## License

This project is licensed under the Apache License 2.0.

## Maintainers

[Andrew Johnson](https://github.com/anjrew) – Maintainer – andrewmjohnson549@gmail.com

## Contributing

Contributions are welcome via PR!

## Other Resources

- Can be paired with [this node](https://github.com/RobotX-Workshops/ros2-serial-reader) for getting values from a serial port
