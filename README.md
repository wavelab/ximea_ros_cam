# ximea_ros_cam

# Prerequisites

Tested on:

* Ximea USB 3.0 MQ013CG-E2

**Make sure that the camera firmware (both CPU and FPGA) are updated to the latest stable firmware! See: https://www.ximea.com/support/wiki/allprod/XIMEA_Camera_Firmware_Command-Line_Update for more info**

# Installing Ximea Camera Driver

## Install the Ximea Software Package

Download and install the Ximea Software Package here: https://www.ximea.com/support/documents/4

### Ubuntu 16.04 LTS Installation

Retrieve the Ximea Software Package:
```
cd ~; mkdir tmp; cd tmp
wget https://www.ximea.com/support/attachments/download/271/XIMEA_Linux_SP.tgz
```

Extract and install the Ximea Software Package (Note: `-cam_usb30` is for USB 3.0 cameras):
```
tar -xf XIMEA_Linux_SP.tgz
cd package
./install -cam_usb30
```

(Optional: Cleanup)
```
cd ~
rm -rf tmp
```

## Add user to the plugdev group
```
sudo gpasswd -a $USER plugdev
```

## Setup the USB FS Memory Max Allocation to Infinite 

This is done to make sure that the USB FS buffering size is sufficient for high bandwidth streams through USB 3.0

*Set this with every new shell:*

Put `echo 0 > /sys/module/usbcore/parameters/usbfs_memory_mb` into `/etc/rc.local`

Or

*Apply to current shell:*

`echo "0" | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb`


## Set realtime priority to the /etc/security/limits.conf

Place the following in `/etc/security/limits.conf` to make the Ximea camera driver have real time priority

```
*               -       rtprio          0
@realtime       -       rtprio          81
*               -       nice            0
@realtime       -       nice            -16
```

Then add the current user to the group `realtime`:

```
sudo groupadd realtime
sudo gpasswd -a $USER realtime
```

# Using `ximea_ros_cam`

Examine `launch/example_cam.launch` and `config/example_cam_config.yaml` to get an understanding on how to use the driver.

## Launching a Camera:

`roslaunch ximea_ros_cam example_cam.launch`

To view the stream: `rosrun rqt_image_view rqt_image_view`

## Launching Multiple Cameras:

Each camera can be ran with their own individual launch files (the parameters will be defined in their own individual name spaces, which makes each camera "unique").

Copy the `example_cam.launch` and input another serial number to `serial_no` of the other camera.

**When launching multiple cameras, be sure to launch them at separate times (~1-2 seconds apart), this is potentially due to USB resource hog issues.**

## Parameter Descriptions:

**Note that these parameters are found in either the config file or launch file**

### General

`serial_no` - Serial number of the Ximea camera (used to locate the proper camera)

`cam_name` - Name of the camera used when saving camera images and snapshots under the directory pointed by `image_directory`

`calib_file` - Calibration file used by the camera

`frame_id` - Frame ID of the camera

`num_cams_in_bus` - Number of USB cameras processed by a single USB controller (That is, if a hub has one controller and 4 ports, with 3 ports plugged with USB 3.0 cameras, then `num_cams_in_bus = 3`) (This will divide the total USB bandwidth by `num_cams_in_bus` to ensure equal bandwidth for each camera)

`bw_safetyratio` - Bandwidth safety ratio, a multiplier to the bandwidth allocated for each camera

`poll_time` - Used to set the duration (in seconds) which the camera is attempted to be opened again. When using multiple cameras, a duration of 2 seconds between each camera is recommended. (i.e. `poll_time=0.0` and `poll_time=2.0` for cameras 1 and 2).

`poll_time_frame` - This is the ROS timer loop period for the ximea camera node. It should generally be set to a rate that is a factor higher than the camera capture rate. For example, if the camera runs at 20Hz (or 0.05s period) and `poll_time_frame` set to 0.001s, then the timer will constantly loop every 0.001 seconds which is faster than the 0.05s period of the camera capture time, with an error that is roughly 0.000 to 0.002 between frames. **Warning: if this value is larger than the period of the camera capture rate, then the frame rate of the camera is capped to the rate of the `poll_time_frame`, for example, if `poll_time_frame` is 0.5s, then the maximum rate that the camera can achieve is 2Hz**

`publish_xi_image_info` - Flag for publishing the extra ximea camera information provided with each image acquisition.

### Image Saving

`image_directory` - Directory used by the flag `save_disk` and `calib_mode` (**Note: Must be a valid directory path with an ABSOLUTE path, otherwise the camera fails to launch or will not create the directory properly. **)

`save_disk` - Save images to disk, under the directory `<image_directory>/stream` 

`calib_mode` - Saves images everytime a trigger is pressed, under the director `<image_directory>/calib`

### Compressed Image Transport Parameters

See [compressed_image_transport](http://wiki.ros.org/compressed_image_transport) for more information.

`image_transport_compressed_format` -  Format for the compressed image - `jpg` or `png`

`image_transport_compressed_jpeg_quality` - 1 to 100 (1 = min quality)

`image_transport_compressed_png_level` - 1 to 9 (9 = max compression)

### Colouring

`format` - Image format: 
```
XI_MONO8 - Grayscale 8 bit
XI_MONO16 - Grayscale 16 bit
XI_RGB24 - BGR 24 bit
XI_RGB32 - BGRA 32 bit
XI_RGB_PLANAR - NOT USED
XI_RAW8 - RAW 8 bit
XI_RAW16 - RAW 16 bit
```

`white_balance_mode` - 0 = none, 1 = use coefficients, 2 = auto

`white_balance_coef_red` - manual white balance red coefficient (0 to 8)

`white_balance_coef_green` - manual white balance green coefficient (0 to 8)

`white_balance_coef_blue` - manual white balance blue coefficient (0 to 8)

### Frame Rate Control

`cam_trigger_mode` - 0 = none, 2 = hardware trigger

`hw_trigger_edge` - For hardware trigger mode, 0 = rising-edge trigger, 1 = falling-edge trigger

`frame_rate_control` - Camera frame rate control (Works if no triggering is enabled)

`frame_rate_set` - FPS limiter (0 for none). This is depending on available bandwidth

`img_capture_timeout` - Timeout for getting an image, in milliseconds

### Exposure Settings

`auto_exposure` - Auto exposure

`auto_exposure_priority` - Auto exposure gain ratio (1 = favour only exposure)

`auto_time_limit` - Auto exposure time limit (microseconds)

`auto_gain_limit` - Auto exposure gain limit

`exposure_time` - Manual exposure time (microseconds)

`manual_gain` - Manual exposure gain

### Region of Interest

`roi_top`, `roi_left` - Top left corner in pixels

`roi_width`, `roi_height` - Width, height in pixels

# License

MIT License

Copyright (c) 2017 WAVE Laboratory

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
