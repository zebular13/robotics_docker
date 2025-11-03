# Launcher Instructions

1.) Make sure the paths are correct:
- Make sure the path on line 4 of launch_robotics_demo.sh matches the path where you've cloned the [robotics_docker](https://github.com/AlbertaBeef/robotics_docker) repo.
- Make sure the path on line 6 of RoboticsDemo.desktop matches the path where you've cloned the [robotics_docker](https://github.com/AlbertaBeef/robotics_docker) repo.

2.) Move the launch script to your home directory and make it executable:
```mv launch_robotics_demo.sh ~/launch_robotics_demo.sh```
```chmod +x ~/launch_robotics_demo.sh```


3). Move the desktop launcher to /local/share/applications and make it executable:

```mv RoboticsDemo.desktop ~/.local/share/applications/RoboticsDemo.desktop```
```chmod +x ~/.local/share/applications/RoboticsDemo.desktop```

The launcher icon should now appear in your applications menu.
![alt text](launcher_icon.png)
4.) Clicking on the icon will launch part 2 of the ROS demo (the Ubuntu laptop part).
(i.e. the docker container with the ROS environment and the command:
    ```ros2 launch hand_controller demo11_mogiros_car_part2.launch.py use_imshow:=False```)