.. SPDX-License-Identifier: BSD-3-Clause
.. SPDX-FileCopyrightText: Czech Technical University in Prague

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package movie_publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.2 (2025-02-19)
------------------
* Fixed buildfarm issues.
* Contributors: Martin Pecka

2.0.1 (2025-02-16)
------------------
* Try fixing build on buildfarm
* Contributors: Martin Pecka

2.0.0 (2025-02-13)
------------------
* Cleaned up dependencies.
* Refactor out MovieProcessorBase.
* CI: Add license linting.
* Compatibility with Melodic.
* Improved documentation, rewritten movie_to_bag to C++, added tests.
* Added support for ImgGPSDirection and GPSTrack expressed towards magnetic North.
* Big rewrite. Moved movie_publisher to a subfolder. Added C++/libav implementation and metadata extraction plugins.
* Contributors: Martin Pecka

1.4.0 (2022-06-08)
------------------
* Compatibility with Noetic
* Contributors: Martin Pecka

1.3.1 (2021-04-27)
------------------
* Fix pixel format of moviepy videos.
* Contributors: Martin Pecka

1.3.0 (2019-03-10)
------------------
* Updated for melodic.
* Contributors: Martin Pecka

1.2.1 (2019-02-07)
------------------
* Fixed permissions.
* Kinetic release.
* Moved to python from bc, because it is not installed everywhere.
* More informative error strings.
* Updated to the fixed version rosbash_params==1.0.2.
* Contributors: Martin Pecka

1.1.0 (2019-01-28)
------------------
* Added checks for exit codes to bash scripts.
* Fixed install targets.
* Added python-opencv alternative backend. This resolves debian packaging issues.
* Contributors: Martin Pecka

1.0.1 (2019-01-25)
------------------
* Fixed install rule.
* Documented all tools in readme.
* Added support for rewriting timestamps from TF messages.
* Fix Python3 compatibility. Added fix_bag_timestamps.
* Added readme.
* Initial commit.
* Contributors: Martin Pecka
