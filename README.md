# A collection of Tools / Nodes I commonly use with ROS

## PTAM Rectification, Image Preprocessing
This nodes listens to an image topic and publishes an image and camera_info, optionally rectifying the image using the ATAN camera model. Preprocessing such as image filtering, deinterlacing, resizing, cropping, colour encodings, etc as well as PTAM parameters can be configured using dynamic reconfigure in realtime.

#### Dependencies
Todo

#### Example
You either need a webcam with know PTAM distortion parameters or you can use the given bag file 

###### Using the example bag file
The example bag file launch/data/example.bag contains a simple 5 second stream of images at 25hz from a 1g nano camera with a wide angle lens.
```roslauch ollieRosTools testBag.launch```
###### Using a camera


#### PTAM Rectification
###### Zoom
###### Cropping
###### Interpolation


#### Image Filters
###### Bilateral Filtering
###### Gaussian Filtering
###### Median Filtering
###### Intensity Normalisation


#### Deinterlacing
###### Skip odd rows
###### Skip even rows
###### None
###### Interpolation Schemes
