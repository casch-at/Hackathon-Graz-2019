# Infineon Drone Hackathon Project - Box Detector with Time of Flight Camera

This project was implemented during the Infineon Hackathon Drone Track at the #LetCluster festival in Graz 2019.

It implements a simple object detection with the pmdtec time of flight camera using ROS and PCL.

The origin goal of this project was to detect a simple object (a cardboard box) as a whole. However, it turned out that the ROS examples for the Object Recognition Kitchen (ORK) do not work anymore with ROS melodic.

So we moved back to ROS kinetic, but it turned out that the examples for object recognition where designed for 3D cameras with RGB camera included, but the pmdtec camera only delivers a grayscale image.

Moving back again to ROS melodic, (easy thanks to Docker), we decided then to approach a simple object regnoition from scratch.

For this purpose, we then used the PCL library to remove the floor plane and to clean up the pointcloud. The last step was to identify target by choosing the closest point in the leftover pointloud.

## How to use

You can build and start the Docker container with the following command:

```bash
./install_scripts/docker-dev.sh -b
./install_scripts/docker-dev.sh
```

After that, run the following commands to build the ROS workspace.

```bash
source devel/setup.bash
catkin build
```

Then you can start the application:

```bash
source devel/setup.bash
roslaunch royale_in_ros camera_driver.launch
rosrun box_detection box_detection_node input:=/royale_camera_driver/point_cloud
rosrun rviz rviz
rosrun rqt_reconfigure rqt_reconfigure
```

## Notes

You can find more notes in [nodes.md](./notes.md)
