# linorobot2

Forked from [linorobot2](https://github.com/linorobot/linorobot2).

## Startup Instructions

1. Run `xhost +local:docker` to allow RVIZ to display from Docker containers.
2. (Optional) Plug in a Microsoft Xbox controller (Model 1914) to USB.
3. Run `docker compose build`.
4. Run `docker compose up`. It may take 1-2 minutes for Gazebo to load.
5. Drive the robot using:
   - Xbox controller (Hold the right bumper and toggle the right joystick to move), **or**
   - Run `docker exec -it linorobot2-nav-slam bash` and `ros2 run teleop_twist_keyboard teleop_twist_keyboard` to control the robot via keyboard.

## Expected Results

- RVIZ will open with the robot's view of the world and should begin auto-generating a map using SLAM.
  
  ![RVIZ Map View](https://github.com/user-attachments/assets/4be799ff-4d3a-48c9-b58a-216c756eebee)

- Gazebo will display a view of the robot and the world, including all the lidar scans.
  
  ![Gazebo View](https://github.com/user-attachments/assets/fb57d7c8-b000-4d7e-bef2-c3fcf2c7c44f)
