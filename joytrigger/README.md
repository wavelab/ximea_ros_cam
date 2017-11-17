# Joy Trigger Script

## How to Use

When loading a camera, set `calib_mode: true` in the config to enable the
functionality to save images via a trigger.

Here the images are saved in the directory set by `image_directory` under a
subfolder `calib`. The camera itself waits for an empty message to be published
to `camera/save_image` in order to trigger an image save.

The following script helps does this. In order to use the script, run:

```
rosrun joy joy_node &
python joy_trigger_node.py
```

And press `A` with an attached usb joystick to trigger an image save.
