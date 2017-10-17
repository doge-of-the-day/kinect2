#### kinect2
This ros package contains a simple kinect2 driver implementation using *libfreenect2*. 
The contained driver node provides the kinect2 data as is, which means that colored points clouds are only available in the native resolution of the time-of-flight sensor.

All rectifecation and undistortion operations are executed using  solely the camera parameters from the kinect2 device.
The transformation between the rgb and ir camera frames has to be calibrated manually. Since this package does not include calibration tools, it is advised to visit the following project:

	https://github.com/code-iai/iai_kinect2

Otherwise, this ros package provides an implementation to map points from the depth frame to the color frame, also based on the libfreenect code.

#### Parameters
| name | type  | default | what for |
| --------|-----|------| ----------- |
| topic_rgb| string | /kinect2/image_color_raw | full hd rgb image |
|topic_rgb_rectified| string|/kinect2/image_rgb/rectified| rectified color image|
|topic_rgb_registered|string|/kinect2/rgb_registered| registered color image|
| topic_rgb_info | string | /kinect2/image_color/camera_info | color camera parameters |
| topic_depth | string | /kinect2/depth | depth images |
| topic_depth_info | string | kinect2/depth/camera_info | depth camera parameters |
| topic_ir | string | /kinect2/image_ir | intensity image |
| topic_ir_info | string | /kinect2/image_ir/camera_info | same as depth camera parameters |
| topic_kinect2_info | string | /kinect2/camera_info | all coeffiecients and parameters packed together |
| topic_color_registered | string | /kinect2/color_registered | registered color image, not full-hd anymore |
| topic_depth_rectified | string | /kinect2/depth_rectified | rectified depth images |
| topic_pointcloud | string | /kinect2/points | colored point cloud |
| service_name_wakeup | string | /kinect2/wakeup | service name to (re-)start the driver |
| service_name_sleep | string | /kinect2/sleep | service name to stop the driver |
| preferred_publication_rate | double | -1.0 | preferred rate at which the driver front end sould operate |
| frame_id_color | string | kinect2_color_optical_frame | frame name for the color camera |
| frame_id_ir | string | kinect2_depth_optical_frame | frame_id for the ir / depth camera |
| publish_color | bool |  false | publish the color image in full-hd |
| publish_ir |  bool |  false | publish the intensity image |
| publish_depth | bool | false | publish the depth image |
| publish_color_registered | bool |  false | publish the registered color image |
| publish_depth_rectified | bool |  false | publish the rectified depth image |
| pipeline_type | string | CUDA  | set the pipeline type | 

Currently supported pipeline types are:

* "GL"
* "OCL"
* "CUDA"
* "KDE_CUDA"
* "KDE_OCL"

If 'pypeline_type' is set to any other string or in case of missing support for a certain mode by libfreenect2, the driver falls back to CPU executation, which is considerable slower.

#### Services and Topics
All sevices and topics can be configured, at least regarding the name. By setting the publication parameter
the user can choose to process the data and publish it. Therefore the effort for data preparation can be 
reduced to the necessary amount.
Additionally two services allow stopping and restarting the driver. This way the sensor can be deactivated 
when not being used.

#### Dependencies
*libreenect2* and *libusb* have been compiled from source. The projects can be found on GitHub:

	https://github.com/OpenKinect/libfreenect2
	https://github.com/libusb/libusb
