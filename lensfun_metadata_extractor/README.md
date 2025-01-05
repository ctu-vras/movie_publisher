<!-- SPDX-License-Identifier: BSD-3-Clause -->
<!-- SPDX-FileCopyrightText: Czech Technical University in Prague -->

# lensfun_metadata_extractor

Extractor of image and movie metadata with [lensfun](https://lensfun.github.io/) backend.

See [movie_publisher](../movie_publisher) for more details.

## Lensfun Database Handling

This extractor utilizes the camera and lens maker/model (gathered by another extractor) to find the capturing system
in the lensfun database. If it finds it, the extractor can provide information about crop factor, sensor size and
for some lenses, also distortion coefficients.

The camera and lens matching algorithm is completely left to lensfun, with a few additions:

- Aspect ratio of the photo/movie must correspond to the aspect ratio of the lens. If multiple lenses are matched by
  lensfun, then the one with matching aspect ratio is selected. If only one is found and aspect ratio doesn't match,
  the lens doesn't match.
- (Multi-frame) movies need a different definition of camera than still images. Usually, switching the camera to movie
  recording changes crop factor and aspect ratio, so this extractor expects additional entries in the database
  specifically for movie modes.
  [There is currently no standard on how to mark video mode in lensfun DB](
  https://github.com/lensfun/lensfun/issues/2384), so this extractor defines that movie mode is defined by adding a
  \<variant\> with `video` in its name. Ideally, the variant should also contain aspect ratio. So e.g.
    
      <variant>16:9 video</variant>

  Please note that the lensfun database doesn't contain any video mode calibrations by itself and you will have to
  provide them yourself.

It is also suggested to regularly call `lensfun-update-data` to update the system database.

If you need to add another database file, you can pass the path to it (or its directory) as a ROS parameter:

- `~lensfun_extra_db` (string, optional): If nonempty, the specified file or directory will be loaded as an additional
  lensfun database directory.