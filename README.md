# A collection of Tools / Nodes I commonly use with ROS



## PTAM Rectification, Image Preprocessing
This nodes listens to an image topic and publishes an image and camera_info, optionally rectifying the image using the ATAN camera model. Pre-processing such as image filtering, deinterlacing, resizing, cropping, colour encodings, etc as well as PTAM parameters can be configured using dynamic reconfigure in realtime.

An image with the dynamic_reconfigure options:



## Overview
An example video of it in action can be found here:
[Example Operation Video](http://youtu.be/a-ItR_ujeto)




#### Dependencies
For now only tested with ros hydro.

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
The camera takes two successive images and packs them together into a single image before sending it. One image is contained in the even rows, the other in the odd rows. When the camera moves fast, artifacts begin to appear.

###### Skip even rows
The even rows are the older ones, so we just use the newer odd ones. We can just leave them out or interpolate the even rows from the odd ones.

Both the following images were taken under the same conditions: good lighting, medium camera shake
![Interpolating even rows from odd](https://lh6.googleusercontent.com/-6DRt3ZZZemQ/UxiSO4PyaFI/AAAAAAAAfDA/ZsplDrl50Uw/s800/Screenshot-ptamRectify.mkv-half.png)

Notice the following artifacts if one keeps both rows:

![Keeping even and odd rows](https://lh3.googleusercontent.com/-nD4RDKHHrxE/UxiSPX-tULI/AAAAAAAAfDE/EQ8jPWPnhJo/s800/Screenshot-ptamRectify.mkv-full.png)


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
