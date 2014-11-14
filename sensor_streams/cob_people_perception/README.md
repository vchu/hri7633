Build Procedure
===============

Make sure the `cob_perception_common` and `cob_people_perception` are in the ROS package path. Then go to the `cob_people_perception/cob_people_detection` folder, `rosmake cob_people_detection`. 


To Run
======
Please see the original package [wiki](http://wiki.ros.org/cob_people_detection).
Basically, start openni camera driver:
`roslaunch openni_launch openni.launch`

Start tracking and recognition:
`roslaunch cob_people_detection people_detection.launch`

Capture face images:
`rosrun cob_people_detection people_detection_client`
Then input 5 -> 1 -> 20 to change incoming image rate.
Then 1 to capture images. Note that because of the recognition algorithm, more than 1 faces/humans must be trained before recognition. 

After image capturing, run `roslaunch cob_people_detection people_detection.launch`, model will be automatically trained and loaded. 


