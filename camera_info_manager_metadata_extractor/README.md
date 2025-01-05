<!-- SPDX-License-Identifier: BSD-3-Clause -->
<!-- SPDX-FileCopyrightText: Czech Technical University in Prague -->

# camera_info_manager_metadata_extractor

Extractor of image and movie metadata with [camera_info_manager_lib](../camera_info_manager_lib) backend.

See [movie_publisher](../movie_publisher) for more details.

## Camera info

To find corresponding camera info, calibration URLs must be provided to the extractor via ROS parameter.

- `~calibration_urls` (string[], optional): Calibration URLs this extractor searches in order to find camera info.

Each URL can contain several substitution keywords:

- `${NAME}` resolved to the current camera name defined by the device driver.
- `${ROS_HOME}` resolved to the `$ROS_HOME` environment variable if defined, or `~/.ros` if not.
- `${FOCAL_LENGTH}` resolved to the focal length of the camera. By default, the focal length is formatted by
  string `%0.01fmm`, but it can be changed by specifying another format string, e.g. `${FOCAL_LENGTH:%0.04f_mm}`.

If `~calibration_urls` is not specified, the following URLs are used:

- `file://${ROS_HOME}/camera_info/${NAME}.yaml`
- `file://${ROS_HOME}/camera_info/${NAME}-${FOCAL_LENGTH:%0.01fmm}.yaml`