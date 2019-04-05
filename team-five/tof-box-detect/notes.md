## ORK (Object Recognition Kit)

### Links

* http://wg-perception.github.io/tabletop/index.html#tabletop
* https://wg-perception.github.io/object_recognition_core/quickguide.html#quickguide
* https://github.com/TuZZiX/pcl_recognition
* http://wiki.ros.org/vfh_recognition

### Install

https://wg-perception.github.io/object_recognition_core/install.html#install

```bash
curl -L https://couchdb.apache.org/repo/bintray-pubkey.asc | sudo apt-key add -
echo "deb https://apache.bintray.com/couchdb-deb bionic main" | sudo tee -a /etc/apt/sources.list.d/apache_couchdb_bionic.list
sudo apt-get install -y couchdb
```

### set up couchdb

https://wg-perception.github.io/object_recognition_core/infrastructure/couch.html#object-recognition-core-db
hack: https://github.com/wg-perception/object_recognition_core/blob/master/python/object_recognition_core/db/models.py#L115

### config object

```bash
rosrun object_recognition_core object_add.py -n block -d "The master block" --commit
rosrun object_recognition_core mesh_add.py <package_id> src/box_control/hackathon_block.obj --commit
rosrun object_recognition_core training -c `rospack find object_recognition_linemod`/conf/training.ork
roslaunch royale_in_ros camera_driver.launch
```


http://wiki.ros.org/pcl_ros


rosdep install -y --from-paths src --ignore-src --rosdistro melodic -r --os=ubuntu:bionic


### topics
/royale_camera_driver/camera_info
/royale_camera_driver/depth_image
/royale_camera_driver/expo_time_param
/royale_camera_driver/expo_time_value
/royale_camera_driver/gray_image
/royale_camera_driver/init_panel
/royale_camera_driver/point_cloud
/royale_camera_driver/update_fps


### Working with PCL

https://industrial-training-master.readthedocs.io/en/latest/_source/session4/Introduction-to-Perception.html#point-cloud-data-file

Capturing PointCloud data.

```bash
rosrun pcl_ros pointcloud_to_pcd input:=/royale_camera_driver/point_cloud
```

```bash
rosrun nodelet nodelet load pcl/StatisticalOutlierRemoval nodelet_manager __name:=nodelet1 nodelet1/input:=/royale_camera_driver/point_cloud nodelet1/output:=/postpro _stddev:=0.2
```

```bash
roslaunch royale_in_ros camera_driver.launch
rostopic pub /min_filter std_msgs/Float32 1.0
rostopic pub /max_filter std_msgs/Float32 2.5
rosrun box_detection box_detection_node input:=/royale_camera_driver/point_cloud
rosrun rqt_reconfigure rqt_reconfigure
rosrun rviz rviz
```

### Docker RPi

https://medium.freecodecamp.org/the-easy-way-to-set-up-docker-on-a-raspberry-pi-7d24ced073ef
