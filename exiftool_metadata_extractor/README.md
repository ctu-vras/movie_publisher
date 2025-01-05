<!-- SPDX-License-Identifier: BSD-3-Clause -->
<!-- SPDX-FileCopyrightText: Czech Technical University in Prague -->

# exiftool_metadata_extractor

Extractor of image and movie metadata with [exiftool](https://exiftool.org/) backend.

See [movie_publisher](../movie_publisher) for more details.

## License

The package code itself is BSD-licensed, but ExiftoolCpp library which it includes is distributed under the following
license:

> This is software is free for non-commercial use.  Permission must be
obtained for using this code in a commercial product, and a license fee may
be required.

You should thus follow this requirement when using this plugin. You are free to use other EXIF-based plugins if you need
something commercially-usable, like [libexif](../libexif_metadata_extractor) or [exiv2](../exiv2_metadata_extractor).