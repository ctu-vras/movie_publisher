<!-- SPDX-License-Identifier: BSD-3-Clause -->
<!-- SPDX-FileCopyrightText: Czech Technical University in Prague -->

# movie_publisher

This package contains several tools for using movie files in ROS (playback, conversion to bag files etc.).

It handles any file formats the system installation of ffmpeg can decode.

**Important: This package used to be implemented in Python using pip-installed libraries. That is no longer needed
as the package now uses C++ API of ffmpeg/libav.** Some things have changed and work a bit differently, but a big effort
was spent to keep the new version as backwards compatible as possible.

It can also extract interesting image/video metadata like GPS coordinates and camera calibrations and provide them as
ROS-native types.

## Main tools

- `movie_publisher_node`: A ROS node that serves a video file as video topic source (`sensor_msgs/Image` and friends).
- `movie_publisher.launch`: A launch file for convenient usage of the node. In the `immediate` mode, it can also be used
  to convert video files to bagfiles in a batch.
- `movie_to_bag`: A script that takes a video as input and transforms it into a bag file.
- `add_movie_to_bag`: A batch script that adds a video as a topic into an existing bag file.

### Helper tools

- `fix_bag_timestamps`: A batch script that rewrites bag files so that the message publication time is taken from the 
message header. This allows you to generate bagfile data at high speed and then reprocess them to have more suitable
publication timestamps.

## movie_publisher_node and nodelet movie_publisher/movie_publisher_nodelet

### Published topics

- `movie` (`sensor_msgs/Image`): The published movie. Subtopics from image\_transport are also provided.
- `movie/camera_info` (`sensor_msgs/CameraInfo`): Camera info.
- `movie/azimuth` (`compass_msgs/Azimuth`): Georeferenced heading of the camera.
- `movie/fix` (`sensor_msgs/NavSatFix`): GNSS position of the camera.
- `movie/fix_detail` (`gps_common/GPSFix`): GNSS position of the camera.
- `movie/imu` (`sensor_msgs/Imu`): Orientation and acceleration of the camera.

To extract the additional topics except `movie`, the node uses metadata extractors. See
[the stack-level README](https://github.com/ctu-vras/movie_publisher/blob/HEAD/README.md) for details about the provided
metadata extractors.

### Parameters

Parameters `~start`, `~end` and `~duration` can be expressed in seconds `(15.35)`, in `(min, sec)`,
in `(hour, min, sec)`, or as a string: `'01:03:05.35'`.

- `~movie_file` (string, required): Path to the movie to play. Any format that ffmpeg can decode.
- `~fps` (float, optional): If set, the playback will be at the specified FPS (speeding up/slowing down the movie).
- `~start` (float|tuple|string, optional): If set, playback will start from the specified time.
    Cannot be set together with `~end` and `~duration`.
- `~end` (float|tuple|string, optional): If set, playback will stop at the specified time (not affected by start).
    Cannot be set together with `~start` and `~duration`.
- `~duration` (float|tuple|string, optional): If set, playback will have this duration. If end is also set, the
    duration is counted from the end of the clip, otherwise, it is the duration from the start of the clip.
    Cannot be set together with `~start` and `~end`.
- `~loop` (bool, default False): Whether to loop the movie until the node is shut down. Excludes `~immediate`.
- `~immediate` (bool, default False): If True, the movie will be processed and published as quickly as possible not
    waiting for the real time. The timestamps in the resulting messages act "real-world-like" (i.e. 15 FPS means
    the frames' timestamps will be 1/15 sec apart). You can set `~fake_time_start` if you want these timestamps to
    begin from a non-zero time. Excludes `~loop`.
- `~playback_rate` (float, optional): If set to a number, immediate mode will not play as fast as possible, but at this
      rate (set the rate to a number where you do not lose any messages, e.g. in image_transport/republish).
- `~fake_time_start` (float, default 0.0): Used with `~immediate` to specify the timestamp of the first message.
- `~timestamp_offset` (int|float|string, default `~fake_time_start`): Adjustment of timestamps determined by
    `~timestamp_source`. If given as string, it can be a simple mathematical expression that can also resolve several
    variables: `ros_time` (current ROS time), `wall_time` (current wall time), `metadata_start` (start time from
    metadata).
- `~timestamp_source` (str, default `metadata`): How to determine timestamps of the movie frames. Options are:
  - `metadata`: Extract absolute time when the movie was recorded and use that time as timestamps.
  - `all_zeros`: Use zero timestamps.
  - `absolute_timecode`: Use the absolute timecode as timestamps (i.e. time since start of movie file).
  - `relative_timecode`: Use the relative timecode as timestamps (i.e. time since `~start`).
  - `ros_time`: Timestamp the frames with current ROS time.
- `~frame_id` (string, default ""): The frame_id used in the geometrical messages' headers.
- `~optical_frame_id` (string, default `${frame_id}_optical_frame`): The frame_id used in the image messages' headers.
- `~spin_after_end` (bool, default False): If True, a rospy.spin() is called after the movie has been published.
- `~verbose` (bool, default False): If True, logs info about every frame played.
- `~allow_yuv_fallback` (bool, default False): Set whether `YUV***` formats should be decoded to YUV422, or whether the
    default encoding should be used.
- `~default_encoding` (string, optional): Set the default encoding which should be used for output frames if there is
    no direct match between the libav pixel format and ROS image encodings.
- `~encoding` (string, optional): Set the encoding which should be used for output frames regardless of their source
    encoding (one of sensor_msgs::image_encodings constants).
- `~wait_after_publisher_created` (float, default 1.0): A workaround for the case where you need to give your
    subscribers some time after the publisher was created. Tweak this number until you get no missing start
    messages.
- `~publisher_queue_size` (int, default 1000 in immediate mode, 10 otherwise): `queue_size` of the movie publisher.

## movie_publisher.launch

This launch file takes arguments with the same name as the node's parameters.

## movie_to_bag

Convert a movie file and its metadata to a ROS bag file.

### Stored topics
 
 - `${~topic}` (`sensor_msgs/Image`): The published movie (if raw transport is used).
 - `${~topic}/${~transport}` (*): The published movie compressed stream (if raw transport is not used).
 - `${~topic}/camera_info` (`sensor_msgs/CameraInfo`): Camera info.
 - `${~topic}/azimuth` (`compass_msgs/Azimuth`): Georeferenced heading of the camera.
 - `${~topic}/fix` (`sensor_msgs/NavSatFix`): GNSS position of the camera.
 - `${~topic}/fix_detail` (`gps_common/GPSFix`): GNSS position of the camera.
 - `${~topic}/imu` (`sensor_msgs/Imu`): Orientation and acceleration of the camera.
 
To change the prefix of all topics, set `~topic` parameter. To change the name of a single topic, remap it.
 
### Parameters
 
Parameters `~start`, `~end` and `~duration` can be expressed in seconds `(15.35)`, in `(min, sec)`,
in `(hour, min, sec)`, or as a string: `'01:03:05.35'`.
 
 - `~bag` (string, required): Path where the result should be stored.
 - `~overwrite_bag` (bool, default false): If true and the bag file exists, it will be overwritten. Otherwise, it will
     be appended (and created if needed).
 - `~movie` (string, required): Path to the movie to play. Any format that ffmpeg can decode.
 - `~transport` (string, default `raw`, suggested `compressed`): The image_transport used to store the movie in the bag. 
 - `~start` (float|tuple|string, optional): If set, the movie will be read from the specified time.
     Cannot be set together with `~end` and `~duration`.
 - `~end` (float|tuple|string, optional): If set, the movie will be read up to the specified time (not affected by
     start). Cannot be set together with `~start` and `~duration`.
 - `~duration` (float|tuple|string, optional): If set, playback will have this duration. If end is also set, the
     duration is counted from the end of the clip, otherwise, it is the duration from the start of the clip. Cannot
     be set together with `~start` and `~end`.
 - `~timestamp_offset` (int|float|string, default 0.0): Adjustment of timestamps determined by `~timestamp_source`.
     If given as string, it can be a simple mathematical expression that can also resolve several variables:
     `ros_time` (current ROS time), `wall_time` (current wall time), `metadata_start` (start time from metadata),
     `bag_start` (start time of the bag file), `bag_end` (end time of the metadata), `bag_duration` (duration of the bag
     file in s). The `bag_*` variables are only available when overwriting or appending an existing bag file.
 - `~timestamp_source` (str, default `metadata`): How to determine timestamps of the movie frames. Options are:
   - `metadata`: Extract absolute time when the movie was recorded and use that time as timestamps.
   - `all_zeros`: Use zero timestamps. Please note that time 0.0 cannot be stored in bag files.
       Use `~timestamp_offset` to make the time valid.
   - `absolute_timecode`: Use the absolute timecode as timestamps (i.e. time since start of movie file).
   - `relative_timecode`: Use the relative timecode as timestamps (i.e. time since `~start`).
   - `ros_time`: Timestamp the frames with current ROS time. Note that this mode is not very useful for movie_to_bag.
 - `~frame_id` (string, default ""): The frame_id used in the geometrical messages' headers.
 - `~optical_frame_id` (string, default `${frame_id}_optical_frame`): The frame_id used in the image messages' headers.
 - `~verbose` (bool, default False): If True, logs info about every frame played.
 - `~allow_yuv_fallback` (bool, default False): Set whether `YUV***` formats should be decoded to YUV422, or whether
     the default encoding should be used.
 - `~default_encoding` (string, optional): Set the default encoding which should be used for output frames if there is
     no direct match between the libav pixel format and ROS image encodings.
 - `~encoding` (string, optional): Set the encoding which should be used for output frames regardless of their source
     encoding (one of sensor_msgs::image_encodings constants).

### Usage

Call this script from commandline setting the node-private parameters.

Example:

    rosrun movie_publisher movie_to_bag _movie:=movie.mp4 _bag:=movie.bag _topic:="/movie" start:=5 fake_time_start:=1548323340.24
    
## add_movie_to_bag

Add a movie file to an existing bagfile as a topic.

It is a Bash script with ROS node-like API - you pass it parameters via `_param:=value` on commandline or via ROS param
server. 
    
### Node-private parameters:

- `movie` (string): Path to the source movie file.
- `bag_in` (string): Path to the source bag file.
- `bag_out` (string, default: `output.bag`): Path where the result should be stored.
- `topic` (string): Name of the movie's topic in the bag file.
- `movie_delay` (int, default `0`): Delay (in seconds) of the movie file from the bag file start. May be negative.
- `overwrite_out_bag` (bool, default false): If true, overwrites existing `bag_out` file, otherwise exits with error if 
`bag_out` exists.
- `bag_tmp` (string, default `/tmp/movie_add_to.bag`): Path where a temporary bag file might be stored.
- `transport` (string, default `compressed`): Name of the image transport to use for encoding the image. Either `raw`,
`compressed`, `theora` or any other installed image transport.

### Usage

Call this script from commandline setting the node-private parameters, and pass any other `arg:=value` arguments - 
these will be relayed to `movie_publisher.launch` as is. Do not pass arguments that would collide with the node-private
parameters of this script (e.g. `movie_file`).

Example:

    rosrun movie_publisher add_movie_to_bag _movie:=movie.mp4 _bag_in:=movie_in.bag _bag_out:=movie_out.bag _topic:="/movie" start:=5 movie_delay:=-1
    
    
## fix_bag_timestamps

Goes through `in_bag` and for all messages with a header field sets their publication time to the time stored in their 
`header.stamp` plus `delay`. If `topics` are set, only works on messages on the listed topics. Reading timestamps from 
`/tf` is also supported.

It is a Python script with ROS node-like API - you pass it parameters via `_param:=value` on commandline. 
      
### Node-private parameters:

- `in_bag` (string): Path to the source bag file.
- `out_bag` (string): Path where the result should be stored.
- `topics` (string, default: `''`): If nonempty, rewrite only timestamps of the listed topics. Pass a comma-separated 
list of topics. 
- `delay` (int, default `0`): Delay (in seconds) which is added to the timestamp read from header. May be negative.
- `overwrite_existing` (bool, default false): If true, overwrites existing `out_bag` file, otherwise exits with error if 
`out_bag` exists.
