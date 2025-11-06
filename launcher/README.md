# Launcher Instructions
## Requirements
- Vision AI Kit 6490 flashed with [qcom-robotics-full-image-wifi.zip](https://avtincdeu.sharepoint.com/:u:/s/QualcommTechnology/EbCiL7U5rCJNjVeXC3AkkuEB2TUX_M83ujRFKW2x808Agw?e=qNedpa)
- PC running Ubuntu 22.04 or 24.04 (other versions haven't been tested)

## Instructions
- Clone the robotics docker:
```git clone https://github.com/zebular13/robotics_docker.git```
- Open the launcher folder:
```cd robotics_docker/launcher```

- Edit ```launch_robotics_demo.sh``` to include your board's IP address on **line 9**

- Run the launcher:

```chmod +x install_launcher.sh```

```./install_launcher.sh```

## Launch the demos
You can now:
1. Find **'Robotics Demo'** in your applications menu

![alt text](robotics_icon.png)

2. Run it directly: ```~/launch_robotics_demo.sh"```

While running, it will open a second terminal to launch demo Part 2 (this launches the Gazebo simulation running on the Host PC).

The first terminal will ask for your Vision AI Kit's password and continue setup until it launches Part 1 of the demo (this launches the Flask app running on the Vision AI Kit).

- Open the flask app in a browser window by navigating to ```[Vision AI Kit 6490 IP address]:5000```

- Position the Gazebo environment on top of the flask app. 

## Stop the demos
To stop Part 1 of the demo, simply close the terminal.

To stop Part 2 of the demo, open a new terminal and type:
```docker stop robotics_demo```
