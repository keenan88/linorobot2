# linorobot2

https://github.com/open-rmf/rmf_demos/tree/humble

sudo ufw allow from any to any port 11345 proto tcp
sudo ufw allow from any to any port 8080 proto tcp
sudo ufw allow from any to any port 55443 proto tcp


sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp to 255.255.255.255

RMF traffic: If a vertex is not connected to a lane, IT WILL NOT BE INCLUDED IN THE NAV GRAPH!!

Nav2 critic scale: https://robotics.stackexchange.com/questions/105749/dwb-planner-in-nav2-does-not-properly-set-scale-for-critics

To get RMF fleet manager to recognize completed task, robot state must have empty path, and mode set to idle or charging


Forked from [linorobot2](https://github.com/linorobot/linorobot2).

## Startup Instructions

1. Run `xhost +local:docker` to allow RVIZ to display from Docker containers.
2. In `.env`, set _SLAM_OR_NAV_ to either _NAV_ or _SLAM_.
3. (Optional) Plug in a Microsoft Xbox controller (Model 1914) to USB.
4. Run `docker compose build`.
5. Run `docker compose up`. It may take 1-2 minutes for Gazebo to load. A `teleop twist` terminal window will open for you to drive the robot.
6. If running NAV, set the robot's starting position in RVIZ2 with the "2D pose estimate" button. Spin the robot a few times until the scans and map line up in RVIZ2.
7. Drive the robot using:
   - Xbox controller (Hold the right bumper and toggle the right joystick to move), **or**
   - Run `docker exec -it linorobot2-nav-slam bash` and `ros2 run teleop_twist_keyboard teleop_twist_keyboard` to control the robot via keyboard.

## Expected Results

- Gazebo will display a view of the robot and the world, including all the lidar scans.
  
  ![Gazebo View](https://github.com/user-attachments/assets/fb57d7c8-b000-4d7e-bef2-c3fcf2c7c44f)

- SLAM: RVIZ will open with the robot's view of the world and should begin auto-generating a map using SLAM.
  
  ![RVIZ Map View](https://github.com/user-attachments/assets/4be799ff-4d3a-48c9-b58a-216c756eebee)

- NAV: RVIZ will open the robots view of the world with the pre-generated map.

![image](https://github.com/user-attachments/assets/ab75c0cb-3a92-4946-96fa-c2bac89121b6)

- NAV: Nav2 goal sent using RVIZ
![image](https://github.com/user-attachments/assets/3130b8f3-9c7c-4197-aa68-da44a7812d5c)



