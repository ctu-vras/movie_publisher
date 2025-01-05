<!-- SPDX-License-Identifier: BSD-3-Clause -->
<!-- SPDX-FileCopyrightText: Czech Technical University in Prague -->

# camera_info_manager_lib

An alternative to [camera_info_manager](https://wiki.ros.org/camera_info_manager) that provides only the C++ API and not
the ROS API (service etc.).

## C++ API

The interface is mostly same as the original [CameraInfoManager](http://docs.ros.org/en/api/camera_info_manager/html/).
This package adds the possibility to dynamically specify focal length and resolve it as a part of the URL.

Each URL can contain several substitution keywords:

- `${NAME}` resolved to the current camera name defined by the device driver.
- `${ROS_HOME}` resolved to the `$ROS_HOME` environment variable if defined, or `~/.ros` if not.
- `${FOCAL_LENGTH}` resolved to the focal length of the camera. By default, the focal length is formatted by
  string `%0.01fmm`, but it can be changed by specifying another format string, e.g. `${FOCAL_LENGTH:%0.04f_mm}`.

## Example Usage

```c++
camera_info_manager_lib::CameraInfoManager manager(log, "test", "package://my_package/calibrations/${NAME}.yaml");
if (manager.isCalibrated())
  return manager.getCameraInfo();
```