
# Turtlesim Target Chasing Bot – ROS2 Project

This project demonstrates a fun and interactive behavior using ROS2 and Turtlesim. A turtle (turtle1) autonomously navigates the environment to chase, catch, and kill a randomly spawned Target turtle. Once the target is caught, it is automatically respawned at a new random location.


## Features

- Spawns a target turtle at a random location with a random orientation.
- Moves turtle1 towards the target using Euclidean distance and angle correction.
- Kills the target turtle once caught (within a threshold distance).
- Respawns the target automatically, creating a loop of continuous chase.


## Project Structure

```bash
ros2_ws/
├── src/
│   └── catch_turtle/
│       ├── catch_turtle/
│       │   ├── __init__.py
│       │   ├── spawn_turtle.py      # Spawns 'Target' turtle
│       │   └── control_turtle.py    # Moves turtle1 to target and kills it
│       └── package.xml
│       └── setup.py
```
## Dependencies

- ROS 2 (Humble recommended)
- turtlesim package

## Install turtlesim:

```bash
sudo apt install ros-humble-turtlesim
```
## How To Run

1. Source your workspace:

```bash
    source install/setup.bash
```
    
2. Run the Turtlesim node:

```bash
    ros2 run turtlesim turtlesim_node
```

3. Run the spawn node:
```bash
    ros2 run turtle_chase turtle_spawner
```

4. Run the control node:
```bash
    ros2 run turtle_chase turtle_controller
```
## Concepts Demonstrated

- ROS2 Publishers & Subscribers

- ROS2 Clients & Services (/spawn, /kill)

- Geometry and control in 2D space

- Using timers and callbacks in ROS2 Python
## Authors

- Ayush Kumar Sharma
   - [GitHub](https://github.com/a-ksharma)
   - [LinkedIn](https://www.linkedin.com/in/ayush-kumar-sharma-064235317/)


## License

This project is open-source and available under the [Apache-2.0 Licence](http://www.apache.org/licenses/)

