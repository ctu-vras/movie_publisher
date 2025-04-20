// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Extractor of image or movie metadata.
 * \author Martin Pecka
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <gps_common/GPSFix.h>
#include <movie_publisher/movie_info.h>
#include <movie_publisher/movie_open_config.h>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <vision_msgs/Detection2DArray.h>

class AVFormatContext;
class AVPacket;

namespace movie_publisher
{

class MetadataManager;

using IntrinsicMatrix = sensor_msgs::CameraInfo::_K_type;
using DistortionType = sensor_msgs::CameraInfo::_distortion_model_type;
using Distortion = sensor_msgs::CameraInfo::_D_type;
using GNSSFixAndDetail = std::pair<cras::optional<sensor_msgs::NavSatFix>, cras::optional<gps_common::GPSFix>>;
using SensorSize = std::pair<double, double>;
using RollPitch = std::pair<double, double>;

/**
 * \brief Parameters passed to the extractor plugins when initializing them.
 */
struct MetadataExtractorParams
{
  cras::LogHelperPtr log;  //!< Logger.
  std::weak_ptr<MetadataManager> manager;  //!< Weak pointer to the metadata manager. Use it to call other extractors.
  MovieOpenConfig config;  //!< Configuration with which the movie was opened.
  MovieInfo info;  //!< Basic information about the open movie.
  const AVFormatContext* avFormatContext;  //!< Libav context of the open movie.
};

/**
 * \brief Extractor of metadata about a movie or image.
 *
 * The extractors implementing this interface should only override methods in case they can directly provide the data.
 * If some data cannot be provided by an extractor, it is not a problem - MetadataManager will call other extractors
 * to get the information.
 *
 * If this extractor needs some metadata it cannot get itself, it can store a pointer to the manager and then call the
 * manager to ask other extractors for the needed data. In this case, use StackGuard to protect against infinite
 * recursion.
 *
 * Some metadata reference transformation frames. Two frames are recognized:
 * - optical frame: The frame of the captured image, i.e. Z forward, X down and Y right, origin in optical center.
 * - geometrical frame: The ROS convential geometrical frame, i.e. Z up, X forward, Y left, origin in optical center.
 *
 * \note Implementing classes should not call any of the base implementations provided by this class.
 */
class MetadataExtractor : public cras::HasLogger
{
public:
  using Ptr = std::shared_ptr<MetadataExtractor>;
  using ConstPtr = std::shared_ptr<const MetadataExtractor>;

  /**
   * \brief Constructor.
   * \param[in] log Logger.
   */
  explicit MetadataExtractor(const cras::LogHelperPtr& log);
  virtual ~MetadataExtractor();

  /**
   * \brief Return the name of the extractor.
   * \return The name.
   */
  virtual std::string getName() const = 0;

  /**
   * \brief Return the priority of the extractor (for ordering in MetadataManager).
   * \return The priority. Lower values have more priority. Usually between 0 and 100.
   */
  virtual int getPriority() const = 0;

  /**
   * \brief Optional processing of libav packets as they are read from the movie file.
   * \param[in] packet The libav packet.
   * \note av_packet_ref() the packet if you want to store it for longer than this call duration.
   */
  virtual void processPacket(const AVPacket* packet) {}

  /**
   * \return String describing the complete camera model.
   */
  virtual cras::optional<std::string> getCameraGeneralName() { return cras::nullopt; }
  /**
   * \return String describing the camera in a unique way, utilizing serial numbers etc. If unique identification is not
   *         possible, nothing is returned.
   */
  virtual cras::optional<std::string> getCameraUniqueName() { return cras::nullopt; }
  /**
   * \return The camera's serial number.
   */
  virtual cras::optional<std::string> getCameraSerialNumber() { return cras::nullopt; }
  /**
   * \return Camera manufacturer.
   */
  virtual cras::optional<std::string> getCameraMake() { return cras::nullopt; }
  /**
   * \return The camera model name (just the model, without manufacturer).
   */
  virtual cras::optional<std::string> getCameraModel() { return cras::nullopt; }
  /**
   * \return Lens manufacturer.
   */
  virtual cras::optional<std::string> getLensMake() { return cras::nullopt; }
  /**
   * \return Lens model.
   */
  virtual cras::optional<std::string> getLensModel() { return cras::nullopt; }
  /**
   * \return Rotation of the image in degrees. Only values 0, 90, 180 and 270 are supported.
   */
  virtual cras::optional<int> getRotation() { return cras::nullopt; }
  /**
   * \return The global time corresponding to the first frame of the movie stream.
   */
  virtual cras::optional<ros::Time> getCreationTime() { return cras::nullopt; }
  /**
   * \return Crop factor of the camera (i.e. how many times is the sensing area smaller than 36x24 mm).
   */
  virtual cras::optional<double> getCropFactor() { return cras::nullopt; }
  /**
   * \return Physical size of the active sensor area that captures the movie (in mm; width, height).
   */
  virtual cras::optional<SensorSize> getSensorSizeMM() { return cras::nullopt; }
  /**
   * \return Focal length recomputed to an equivalent 35 mm system.
   */
  virtual cras::optional<double> getFocalLength35MM() { return cras::nullopt; }
  /**
   * \return The focal length in mm.
   */
  virtual cras::optional<double> getFocalLengthMM() { return cras::nullopt; }
  /**
   * \return The focal length in pixels.
   */
  virtual cras::optional<double> getFocalLengthPx() { return cras::nullopt; }
  /**
   * \return The intrinsic calibration matrix of the camera.
   */
  virtual cras::optional<IntrinsicMatrix> getIntrinsicMatrix() { return cras::nullopt; }
  /**
   * \return Camera distortion coefficients (corresponding to the OpenCV calibration model: 5 or 8 elements).
   */
  virtual cras::optional<std::pair<DistortionType, Distortion>> getDistortion() { return cras::nullopt; }
  /**
   * \return GNSS position of the camera when capturing the frame.
   */
  virtual GNSSFixAndDetail getGNSSPosition() { return std::make_pair(cras::nullopt, cras::nullopt); }
  /**
   * \return Azimuth describing global camera heading when capturing the frame.
   */
  virtual cras::optional<compass_msgs::Azimuth> getAzimuth() { return cras::nullopt; }
  /**
   * \return The magnetic field acting on the camera; measurement in X, Y and Z axes [T].
   */
  virtual cras::optional<sensor_msgs::MagneticField> getMagneticField() { return cras::nullopt; }
  /**
   * \return Gravity-aligned roll and pitch of the camera when capturing the frame [rad].
   */
  virtual cras::optional<RollPitch> getRollPitch() { return cras::nullopt; }
  /**
   * \return Acceleration vector acting on the camera when capturing the frame (including gravity) [m/s^2].
   */
  virtual cras::optional<geometry_msgs::Vector3> getAcceleration() { return cras::nullopt; }
  /**
   * \return Angular velocity acting on the camera when capturing the frame. [rad/s].
   */
  virtual cras::optional<geometry_msgs::Vector3> getAngularVelocity() { return cras::nullopt; }
  /**
   * \return Faces detected in the scene.
   */
  virtual cras::optional<vision_msgs::Detection2DArray> getFaces() { return cras::nullopt; }

  /**
   * \return Extracted camera info combined from other fields, or nothing.
   */
  virtual cras::optional<sensor_msgs::CameraInfo> getCameraInfo() { return cras::nullopt; }

  /**
   * \return Extracted IMU info combined from other fields, or nothing.
   */
  virtual cras::optional<sensor_msgs::Imu> getImu() { return cras::nullopt; }

  /**
   * \return Optical frame transform, i.e. transform between camera's geometrical and optical frame. It might be
   *         affected by image rotation.
   */
  virtual cras::optional<geometry_msgs::Transform> getOpticalFrameTF() { return cras::nullopt; }

  /**
   * \return The transform from camera body frame to a gravity-aligned frame.
   */
  virtual cras::optional<geometry_msgs::TransformStamped> getZeroRollPitchTF() { return cras::nullopt; }

  // These reserved functions are future-proofing placeholders that allow adding new types of extractable metadata
  // without breaking binary compatibility of existing extractors. When adding the getter for the new metadata type,
  // just replace one of these functions.
  // There are probably a few other steps to do when turning one of the reserved functions into an actual one:
  // https://www.haiku-os.org/documents/dev/binary_compatibility_3_easy_steps/

private:
  virtual void __reserved0() {}
  virtual void __reserved1() {}
  virtual void __reserved2() {}
  virtual void __reserved3() {}
  virtual void __reserved4() {}
  virtual void __reserved5() {}
  virtual void __reserved6() {}
  virtual void __reserved7() {}
  virtual void __reserved8() {}
  virtual void __reserved9() {}
};

/**
 * \brief Timestamping wrapper for any kind of metadata.
 * \tparam T Type of metadata.
 */
template<typename T>
struct TimedMetadata
{
  using value_type = T;  //!< Type of the metadata values.
  StreamTime stamp;  //!< Timestamp of the data (in stream time).
  T value;  //!< The stored metadata.
};

/**
 * \brief Enum for metadata that can be varying over time.
 */
enum class TimedMetadataType
{
  ROTATION,
  CROP_FACTOR,
  SENSOR_SIZE_MM,
  FOCAL_LENGTH_35MM,
  FOCAL_LENGTH_MM,
  FOCAL_LENGTH_PX,
  INTRINSIC_MATRIX,
  DISTORTION,
  GNSS_POSITION,
  AZIMUTH,
  MAGNETIC_FIELD,
  ROLL_PITCH,
  ACCELERATION,
  ANGULAR_VELOCITY,
  FACES,
  CAMERA_INFO,
  IMU,
  OPTICAL_FRAME_TF,
};

/**
 * \brief Listener of decoded timed metadata.
 *
 * Extractors of timed metadata will call the relevant process*() callbacks of the listener when new timed metadata are
 * parsed.
 *
 * \note Implementing classes should not call any of the base implementations provided by this class.
 *
 * \note The difference between TimedMetadataListener and MovieMetadataProcessor is the level of abstraction on which
 *       they work. TimedMetadataListener connects Movie and TimedMetadataExtractors so that the Movie class gets info
 *       about the metadata it has. MovieMetadataProcessor is then the interface between Movie and "the outer world",
 *       i.e. users of the Movie class. Usually, TimedMetadataListener will be mostly used internally in this library,
 *       while MovieMetadataProcessor will be used publicly by other classes outside this package. Also, the timestamps
 *       in TimedMetadataListener are always stream timestamps, while in MovieMetadataProcessor they should already
 *       correspond to ROS time according to the defined conversion function.
 */
struct TimedMetadataListener
{
  using Ptr = std::shared_ptr<TimedMetadataListener>;
  using ConstPtr = std::shared_ptr<const TimedMetadataListener>;

  virtual ~TimedMetadataListener();

  /**
   * \brief Process the rotation of the image.
   * \data[in] Rotation of the image in degrees. Only values 0, 90, 180 and 270 are supported.
   */
  virtual void processRotation(const TimedMetadata<int>& data) {}
  /**
   * \brief Process crop factor of the camera (i.e. how many times is the sensing area smaller than 36x24 mm).
   * \data[in] The crop factor.
   */
  virtual void processCropFactor(const TimedMetadata<double>& data) {}
  /**
   * \brief Process the sensor physical size.
   * \data[in] Sensor size in mm (width, height).
   */
  virtual void processSensorSizeMM(const TimedMetadata<SensorSize>& data) {}
  /**
   * \brief Process the focal length recomputed to an equivalent 35 mm system.
   * \data[in] The focal length 35 mm equivalent.
   */
  virtual void processFocalLength35MM(const TimedMetadata<double>& data) {}
  /**
   * \brief Process the focal length in mm.
   * \data[in] The focal length [mm].
   */
  virtual void processFocalLengthMM(const TimedMetadata<double>& data) {}
  /**
   * \brief Process the focal length expressed in pixels.
   * \data[in] The focal length [px].
   */
  virtual void processFocalLengthPx(const TimedMetadata<double>& data) {}
  /**
   * \brief Process the intrinsic calibration matrix of the camera.
   * \data[in] The intrinsic matrix.
   */
  virtual void processIntrinsicMatrix(const TimedMetadata<IntrinsicMatrix>& data) {}
  /**
   * \brief Process the camera distortion coefficients (corresponding to the OpenCV calibration model).
   * \data[in] The distortion coefficients (5 or 8 elements).
   */
  virtual void processDistortion(const TimedMetadata<std::pair<DistortionType, Distortion>>& data) {}
  /**
   * \brief Process the GNSS position of the camera when capturing the frame.
   * \data[in] The GNSS position.
   */
  virtual void processGNSSPosition(const TimedMetadata<GNSSFixAndDetail>& data) {}
  /**
   * \brief Process the azimuth describing global camera heading when capturing the frame.
   * \data[in] The azimuth.
   */
  virtual void processAzimuth(const TimedMetadata<compass_msgs::Azimuth>& data) {}
  /**
   * \brief Process the magnetic field acting on the camera.
   * \data[in] The magnetic field measurement in X, Y and Z axes [T].
   */
  virtual void processMagneticField(const TimedMetadata<sensor_msgs::MagneticField>& data) {}
  /**
   * \brief Process gravity-aligned roll and pitch of the camera when capturing the frame.
   * \data[in] Roll, pitch [rad].
   */
  virtual void processRollPitch(const TimedMetadata<RollPitch>& data) {}
  /**
   * \brief Process the acceleration vector acting on the camera when capturing the frame (including gravity).
   * \data[in] The acceleration vector [m/s^2].
   */
  virtual void processAcceleration(const TimedMetadata<geometry_msgs::Vector3>& data) {}
  /**
   * \brief Process the angular velocity acting on the camera when capturing the frame.
   * \data[in] The angular velocity [rad/s].
   */
  virtual void processAngularVelocity(const TimedMetadata<geometry_msgs::Vector3>& data) {}
  /**
   * \brief Process faces detected in the scene.
   * \data[in] The faces that were detected.
   */
  virtual void processFaces(const TimedMetadata<vision_msgs::Detection2DArray>& data) {}

  /**
   * \brief Process camera info.
   * \data[in] Camera info data.
   */
  virtual void processCameraInfo(const TimedMetadata<sensor_msgs::CameraInfo>& data) {}

  /**
   * \brief Process IMU data.
   * \data[in] IMU data.
   */
  virtual void processImu(const TimedMetadata<sensor_msgs::Imu>& data) {}

  /**
   * \brief Process the optical frame transform (might be affected by image rotation).
   * \data[in] Transform between camera's geometrical and optical frame.
   */
  virtual void processOpticalFrameTF(const TimedMetadata<geometry_msgs::Transform>& data) {}

private:
  // These reserved functions are future-proofing placeholders that allow adding new callbacks. See MetadataExtractor.
  virtual void __reserved0() {}
  virtual void __reserved1() {}
  virtual void __reserved2() {}
  virtual void __reserved3() {}
  virtual void __reserved4() {}
  virtual void __reserved5() {}
  virtual void __reserved6() {}
  virtual void __reserved7() {}
  virtual void __reserved8() {}
  virtual void __reserved9() {}
};

/**
 * \brief Extractor of timed metadata.
 *
 * Different from the static metadata, timed metadata can get new values as the movie is advanced. Also, the update
 * rates of the timed metadata may not match the framerate of the movie. That is way "reading" the timed metadata
 * is implemented via callbacks to the TimedMetadataListener classes.
 */
class TimedMetadataExtractor : public MetadataExtractor
{
public:
  using Ptr = std::shared_ptr<TimedMetadataExtractor>;
  using ConstPtr = std::shared_ptr<const TimedMetadataExtractor>;

  /**
   * \param[in] log Logger.
   */
  explicit TimedMetadataExtractor(const cras::LogHelperPtr& log);

  /**
   * \brief Add a new timed metadata listener.
   * \param[in] listener The listener to be added.
   * \note Implementing classes should call this base method implementation. It adds the listener to listeners field.
   */
  virtual void addTimedMetadataListener(const std::shared_ptr<TimedMetadataListener>& listener);

  /**
   * \brief Get a list of timed metadata that are supported by this instance of the extractor.
   * \return A mapping of metadata types to their priority.
   * \note For each metadata type, only the extractor with the lowest priority value will be called to actually
   *       extract the metadata. Order the priorities by the complexity of extracting the data. Anything below 20
   *       means that it is not needed to open or read additional files.
   * \note This function should return a correct result right after the extractor is constructed. It should reflect
   *       the actual content of the file the extractor is bound to, and if the extractor doesn't see anything it could
   *       decode, it should return an empty value from this function.
   */
  virtual const std::unordered_map<TimedMetadataType, int>& supportedTimedMetadata() const = 0;

  /**
   * \brief Perform any required initialization of the extractor so that it is prepared to extract metadata of the given
   *        types.
   * \param[in] requestedTypes The types of metadata this extractor should provide. It is essential that it doesn't call
   *                           listener callbacks for any other types. The list of types passed to this function should
   *                           be a subset of supportedTimedMetadata().
   * \note This method should be called before the first call to processTimedMetadata().
   * \note Implementing classes should call this base method in their overrides. This method sets
   *       requestedMetadataTypes to whatever is received in requestedTypes.
   */
  virtual void prepareTimedMetadata(const std::vector<TimedMetadataType>& requestedTypes);

  /**
   * \brief Process timed metadata up until the time passed as parameter.
   * \param[in] maxTime The maximum stream timestamp of the timed metadata that should be passed to listeners.
   * \note It is assumed that maxTime is a sequence of times growing as fast as the video frame timestamps. Use
   *       seekTimedMetadata() to inform the extractor that it should seek somewhere (create a discontinuity in this
   *       growing sequence of timestamps).
   * \note If timed metadata are present and supported, all listeners added by addTimedMetadataListener() will be
   *       called with appropriate data (limited to the types set in prepareTimedMetadata()).
   * \note Implementing classes should not call this base method implementation.
   */
  virtual void processTimedMetadata(const StreamTime& maxTime) {}

  /**
   * \brief Seek timed metadata to the given stream time.
   * \param[in] seekTime The stream timestamp to seek to.
   * \note Implementing classes should not call this base method implementation.
   */
  virtual void seekTimedMetadata(const StreamTime& seekTime) {}

  /**
   * \return Whether the extractor is able to extract some timed metadata from the movie it is bound to.
   * \note Implementing classes should not call this base method implementation.
   */
  virtual bool hasTimedMetadata() const { return false; }

protected:
  std::vector<TimedMetadataListener::Ptr> listeners;  //!< The listeners whose callbacks should be called.
  std::unordered_set<TimedMetadataType> requestedTimedMetadata;  //!< The list of metadata types to process.
};

/**
 * \brief Helper structure that handles instantiation of an extractor.
 */
struct MetadataExtractorPlugin
{
  virtual ~MetadataExtractorPlugin() = default;

  /**
   * \brief Instantiate the extractor with the given parameters.
   * \param[in] params Parameters that configure the extractor.
   * \return An instance of the extractor.
   */
  virtual MetadataExtractor::Ptr getExtractor(const MetadataExtractorParams& params) = 0;
};

}
