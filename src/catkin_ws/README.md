## ROS Workflow

### Setup
1. Initialize the catkin workspace in the current directory of this README file
   ```bash
   init catkin
   ```
1. Build the catkin workspace with all its included packages
   ```bash
   catkin_make
   ```
1. Link the ros packages (Note must be ran inside each new terminal)
   ```bash
   source devel/setup.bash
   ```
1. Change directory to scripts
   ```bash
   cd src/scion_ros/scripts
   ```
1. Make python scripts executable
   ```bash
   chmod +x sub.py pub.py
   ```

### How to run example
1. Start roscore
   ```bash
   roscore &
   ```
1. Start publisher python node
   ```bash
   rosrun scion_ros pub.py
   ```
1. Open a new terminal and navigate to catkin workspace
   ```bash
   cd ${"This Repo's Directory"}/robosub-2022-software/src/catkin_ws
   ```
1. Resource the packages for new terminal instance
   ```bash
   source devel/setup.bash
   ```
1. Run subscriber python node
   ```bash
   rosrun scion_ros sub.py
   ```
