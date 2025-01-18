# cwipc_kinect


This project has software for capturing point clouds using Microsoft Azure Kinect cameras, _k4a_ for short. The software turns k4a depth- and colorframes into a cwipc pointcloud.

It is usually built or installed as part of the _cwipc_ suite, <https://github.com/cwi-dis/cwipc>.

This module requires the following software to be installed:

- Azure Kinect SDK
- Azure Kinect Body Tracking SDK
- OpenCV

See the [cwipc readme](../readme.md) file for installation instructions.

## Prepare for use

The k4a capturer needs a `cameraconfig.json` file that specifies the serial numbers of the cameras and their position and orientation.

If you have a single camera in landscape mode looking forward, approximately 1m from the subject at 1m height you can automatically create a config file with

```
cwipc_register --tabletop
```

In the unlikely event that `cwipc_register` does not recognize the fact that you have Kinect camera you can supply the `--kinect` option.

If you have more cameras, or a single camera that is oriented differently (for example in portrait mode, so it is easier to capture a full human body) you should check the documentation on `cwipc_register`.

Inspect the resulting pointcloud view with

```
cwipc_view
```

## cameraconfig.json

You can edit `cameraconfig.json` and modify various settings, such as camera parameters like white balance, various processing options that govern how RGB and Depth images are converted to pointclouds, and bounding box parameters for the pointcloud.

After editing parameters you re-run `cwipc_view` to see the effect of your changes.