# A collection of Tools / Nodes I commonly use with ROS

## PTAM Rectification, Image Preprocessing
This nodes listens to an image topic and publishes an image and camera_info, optionally rectifying the image using the ATAN camera model. Preprocessing such as image filtering, deinterlacing, resizing, cropping, colour encodings, etc as well as PTAM parameters can be configured using dynamic reconfigure in realtime.

#### Dependencies
Todo

#### Example
You either need a webcam with know PTAM distortion parameters or you can use the given bag file.
Both examples spawn:
 * RVIZ with some predefined views and settings
 * Dynamic reconfigure node
 * Rosbag iamge spamming or Camera image grabber
 * 2 static transform publishers that relate the world frame to the camera frame via an object frame
 * The preprocessing node that does image filtering/rectification

Once the example is running, you will need to go ro rviz and uncheck / recheck the camera box to show the processed image. This is a bug in rviz. Feel free to play with the dynamic reconfigure settings and observe the output.

###### Example with given bag file
The example bag file launch/data/example.bag contains a simple 3 second stream of images at 25hz from a 1g nano camera with a wide angle lens. To run the example, do the following:

```roslauch ollieRosTools testBag.launch```

This plays the bag file in a loop, allowing you to play with different options and visualising them in realtime.

###### Using a camera
To run the example with a live camera, you will need to do the following:
<<<<<<< Updated upstream
<<<<<<< Updated upstream

```roslauch ollieRosTools testCam.launch cam:=0```

=======

```roslauch ollieRosTools testCam.launch cam:=0```

>>>>>>> Stashed changes
=======

```roslauch ollieRosTools testCam.launch cam:=0```

>>>>>>> Stashed changes
Make sure you change the video device number ```cam:=0``` to your corresponding one.

#### Deinterlacing
###### Skip odd rows
###### Skip even rows
###### None
###### Interpolation Schemes

#### PTAM Rectification
###### Zoom
###### Cropping
###### Interpolation


#### Image Filters
###### Bilateral Filtering
###### Gaussian Filtering
###### Median Filtering
###### Intensity Normalisation


<<<<<<< Updated upstream
<<<<<<< Updated upstream
=======

>>>>>>> Stashed changes
=======

>>>>>>> Stashed changes
