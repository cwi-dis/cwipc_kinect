# cwipc_kinect

This project has software for capturing point clouds using Microsoft Azure Kinect (K4A) cameras. The software turns K4A depth- and colorframes into a cwipc pointcloud.

It requires the cwipc_util library.

As of this writing, the cwipc_util utilities that do something with a camera (such as `cwipc_calibrate` or `cwipc_view`) default to using Realsense cameras. Pass the `--kinect` option to use Kinect.

## Installing

For use within VRtogether you can get pre-built zipfiles (or tgzfiles for Mac/Linux) from <https://baltig.viaccess-orca.com:8443/VRT/nativeclient-group/cwipc_kinect/releases>. Download the most recent release with a normal v_X_._Y_._Z_ name. You will also need the accompanying _cwipc\_util_ installer from 
<https://baltig.viaccess-orca.com:8443/VRT/nativeclient-group/cwipc_util/releases>.

[![pipeline status](https://baltig.viaccess-orca.com:8443/VRT/nativeclient-group/cwipc_kinect/badges/master/pipeline.svg)](https://baltig.viaccess-orca.com:8443/VRT/nativeclient-group/cwipc_kinect/commits/master)

### Windows

- Install cwipc_util (and its dependencies PCL and others).

- Install _Azure Kinect SDK_ version 1.4.1 via <https://github.com/microsoft/Azure-Kinect-Sensor-SDK>.
- Extract the `cwipc_util_win1064_vX.Y.zip` file into the same place you extracted cwipc_util. Check that the DLLs are right next to each other in `installed/bin`
- Add the `c:\vrtogether\installed\bin` folder to the `%PATH%` system environment variable.

### OSX

K4A is currently not supported for OSX, because the Microsoft K4A SDK is not available yet.
  
### Ubuntu 20.04

K4A is currently not support for Linux, because the build process needs work.

## Building from source

The directory `cwipc_kinect` that is the root of this repository should be put on your system where you cloned all other repos. The CMake setup should be able to find all dependencies needed.

It is assumed that if you build cwipc_kinect from source you also build cwipc_util from source. So the instructions here assume that you have already followed the cwipc_util instructions.

## Build instructions (all platforms)

- You need everything needed by the cwipc_util build instructions.
- You need the Azure Kinect SDK from <https://github.com/microsoft/Azure-Kinect-Sensor-SDK>
- After you have built cwipc_util you follow the same sequence of commands for cwipc_kinect.
