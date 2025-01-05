<!-- SPDX-License-Identifier: BSD-3-Clause -->
<!-- SPDX-FileCopyrightText: Czech Technical University in Prague -->

# exiv2_metadata_extractor

Extractor of image and movie metadata with [exiv2](https://exiv2.org/) backend.

See [movie_publisher](../movie_publisher) for more details.

## License

The package code itself is BSD-licensed, but Exiv2 library which it includes and links to is GPL 2.0+ licensed.

You should thus consider this when using the extractor. Although [movie_publisher](../movie_publisher) will load this
extractor dynamically at runtime, some still consider this linking, and thus the final program in memory will also be
GPL-licensed.

You are free to use other EXIF-based plugins if you need something non-copyleft, like
[exiftool](../exiftool_metadata_extractor) or [exiv2](../exiv2_metadata_extractor).