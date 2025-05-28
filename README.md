<!-- SPDX-License-Identifier: BSD-3-Clause -->
<!-- SPDX-FileCopyrightText: Czech Technical University in Prague -->

# movie_publisher Stack

This stack contains several tools for using movie files in ROS (playback, conversion to bag files etc.).

The most important package is [movie_publisher](movie_publisher).

## Metadata

[movie_publisher](movie_publisher) can also extract various interesting metadata from the image and movie files, like
GNSS location, camera calibration, image orientation etc. To extract the metadata, several extractors are available:

- Builtin into [movie_publisher](movie_publisher):
  - [LibavStreamMetadataExtractor.h](movie_publisher/include/movie_publisher/metadata/LibavStreamMetadataExtractor.h): Extractor of metadata from an open LibAV stream.
  - [FilenameMetadataExtractor.h](movie_publisher/include/movie_publisher/metadata/FilenameMetadataExtractor.h): Extractor of metadata from filename (search for date-time strings like `2025-05-28-14-00-01`).
  - [FileMetadataExtractor.h](movie_publisher/include/movie_publisher/metadata/FileMetadataExtractor.h): Extractor of metadata from filesystem properties of the movie file.
  - [SidecarFileMetadataExtractor.h](movie_publisher/include/movie_publisher/metadata/SidecarFileMetadataExtractor.h): Extractor of metadata from external YAML files ([example file](test_data/ffmpeg_processed/P1320029.MP4.yaml)).
- [exiftool_metadata_extractor](exiftool_metadata_extractor): Extractor of image and movie metadata with [exiftool](https://exiftool.org/) backend.
- [exiv2_metadata_extractor](exiv2_metadata_extractor): Extractor of image and movie metadata with [exiv2](https://exiv2.org/) backend.
- [libexif_metadata_extractor](libexif_metadata_extractor): Extractor of image and movie metadata with [libexif](https://libexif.github.io/) backend.
- [lensfun_metadata_extractor](lensfun_metadata_extractor): Extractor of image and movie metadata with [lensfun](https://lensfun.github.io/) backend.
- [camera_info_manager_metadata_extractor](camera_info_manager_metadata_extractor): Extractor of image and movie metadata with [camera_info_manager_lib](camera_info_manager_lib) backend.
- [gpmf_metadata_extractor](gpmf_metadata_extractor): Extractor of image and movie metadata with [gpmf-parser](https://github.com/gopro/gpmf-parser) backend (GoPro videos).

The extractors are found automatically by pluginlib as all packages specifying
`<export><movie_publisher metadata_plugins="${prefix}/plugins.xml" /></export>`
in `package.xml`.

Each extractor implements interface [metadata_extractor.h](movie_publisher/include/movie_publisher/metadata_extractor.h)
by providing some raw metadata. [metadata_manager.h](movie_publisher/include/movie_publisher/metadata_manager.h) is a
meta-extractor that uses all loaded extractors to get as much metadata as possible, possibly providing
cross-dependencies between individual extractors (e.g. lensfun needs to know camera name, but it cannot figure it
itself, so it relies on some other extractor to provide the name).

[composers](movie_publisher/src/composers) provide ways to combine several metadata from extractors into new metadata.

If an extractor also implements the `TimedMetadataExtractor` interface, it says it can provide time-varying metadata
(like position or orientation of the camera).