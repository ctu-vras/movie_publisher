// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Extractor of metadata from an open LibAV stream.
 * \author Martin Pecka
 */

#include <cstdlib>
#include <regex>

extern "C"
{
#include <libavformat/avformat.h>
#include <libavutil/display.h>
#include <libavutil/parseutils.h>
}

#include <cras_cpp_common/type_utils.hpp>
#include <movie_publisher/metadata/LibavStreamMetadataExtractor.h>
#include <pluginlib/class_list_macros.h>

namespace movie_publisher
{

struct LibavStreamMetadataPrivate
{
  const AVFormatContext* avFormatContext;
  size_t streamIndex;
  AVStream* stream;
};

LibavStreamMetadataExtractor::LibavStreamMetadataExtractor(
  const cras::LogHelperPtr& log, const AVFormatContext* avFormatContext, const size_t streamIndex)
  : MetadataExtractor(log), data(new LibavStreamMetadataPrivate())
{
  this->data->avFormatContext = avFormatContext;
  this->data->streamIndex = streamIndex;
  this->data->stream = this->data->avFormatContext->streams[this->data->streamIndex];

  AVDictionaryEntry* tag = nullptr;
  while ((tag = av_dict_get(this->data->avFormatContext->metadata, "", tag, AV_DICT_IGNORE_SUFFIX)))
  {
    CRAS_DEBUG_NAMED("libav_stream.dump", "libav %s=%s", tag->key, tag->value);
  }
}

LibavStreamMetadataExtractor::~LibavStreamMetadataExtractor() = default;

std::string LibavStreamMetadataExtractor::getName() const
{
  return cras::getTypeName<std::remove_cv_t<std::remove_reference_t<decltype(*this)>>>();
}

int LibavStreamMetadataExtractor::getPriority() const
{
  return 10;
}

cras::optional<std::string> LibavStreamMetadataExtractor::getCameraMake()
{
  const auto makeEntry = av_dict_get(this->data->avFormatContext->metadata, "com.apple.quicktime.make", nullptr, 0);
  if (makeEntry != nullptr)
  {
    CRAS_DEBUG_NAMED("libav_stream",
      "Camera make '%s' read from movie metadata %s.", makeEntry->value, makeEntry->key);
    return std::string(makeEntry->value);
  }
  return cras::nullopt;
}

cras::optional<std::string> LibavStreamMetadataExtractor::getCameraModel()
{
  const auto modelEntry = av_dict_get(this->data->avFormatContext->metadata, "com.apple.quicktime.model", nullptr, 0);
  if (modelEntry != nullptr)
  {
    CRAS_DEBUG_NAMED("libav_stream",
      "Camera model '%s' read from movie metadata %s.", modelEntry->value, modelEntry->key);
    return std::string(modelEntry->value);
  }
  return cras::nullopt;
}

cras::optional<ros::Time> LibavStreamMetadataExtractor::getCreationTime()
{
  const auto creationTimeEntry = av_dict_get(this->data->avFormatContext->metadata, "creation_time", nullptr, 0);
  if (creationTimeEntry == nullptr)
    return cras::nullopt;

  try
  {
    const auto result = cras::parseTime(creationTimeEntry->value);
    CRAS_DEBUG_NAMED("libav_stream",
      "Creation time %.09f read from movie metadata %s.", result.toSec(), creationTimeEntry->key);
    return result;
  }
  catch (const std::invalid_argument& e)
  {
    CRAS_ERROR_NAMED("libav_stream", "Error reading image metadata: %s", e.what());
  }
  return cras::nullopt;
}

cras::optional<int> LibavStreamMetadataExtractor::getRotation()
{
  const auto rotateEntry = av_dict_get(this->data->stream->metadata, "rotate", nullptr, 0);

  cras::optional<int> rotation;
  try
  {
    if (rotateEntry && *rotateEntry->value && strlen(rotateEntry->value) > 0)
    {
      rotation = static_cast<int>(cras::parseDouble(rotateEntry->value));
      cras::TempLocale l(LC_ALL, "en_US.UTF-8");
      CRAS_DEBUG_NAMED("libav_stream",
        "Image rotation %d° determined from movie metadata %s.", *rotation, rotateEntry->key);
    }
  }
  catch (std::invalid_argument&) {}

  if (!rotation.has_value())
  {
    const auto displayMatrix = av_stream_get_side_data(this->data->stream, AV_PKT_DATA_DISPLAYMATRIX, nullptr);
    if (displayMatrix)
    {
      rotation = -av_display_rotation_get(reinterpret_cast<int32_t*>(displayMatrix));
      cras::TempLocale l(LC_ALL, "en_US.UTF-8");
      CRAS_DEBUG_NAMED("libav_stream",
        "Image rotation %d° determined from movie display matrix.", *rotation);
    }
  }

  if (!rotation.has_value())
    return cras::nullopt;

  rotation = std::fmod(*rotation, 360);
  if (rotation < 0)
    rotation = *rotation + 360;

  return rotation;
}

std::pair<cras::optional<sensor_msgs::NavSatFix>, cras::optional<gps_common::GPSFix>>
LibavStreamMetadataExtractor::getGNSSPosition()
{
  const auto locationEntry = av_dict_get(
    this->data->avFormatContext->metadata, "com.apple.quicktime.location.ISO6709", nullptr, 0);
  if (locationEntry != nullptr)
  {
    std::pair<cras::optional<sensor_msgs::NavSatFix>, cras::optional<gps_common::GPSFix>> result;

    const std::string locationStr = locationEntry->value;
    std::regex locationRegex {R"(([NS+-])(\d+)\.?(\d*)([EW+-])(\d+)\.?(\d*)([+-]?)(\d*)\.?(\d*)(?:CRS)?(.*)/)"};
    std::smatch matches;
    if (std::regex_match(locationStr, matches, locationRegex))
    {
      const auto latSign = (matches[1].str() == "-" || matches[1].str() == "S") ? -1 : 1;
      const auto latIntPartStr = matches[2].str();
      const auto latFracPartStr = matches[3].str();
      const auto lonSign = (matches[4].str() == "-" || matches[4].str() == "S") ? -1 : 1;
      const auto lonIntPartStr = matches[5].str();
      const auto lonFracPartStr = matches[6].str();
      const auto altSign = (matches[7].str() == "-" || matches[7].str() == "S") ? -1 : 1;
      const auto altIntPartStr = matches[8].str();
      const auto altFracPartStr = matches[9].str();
      const auto CRS = matches[10].str();

      if (!CRS.empty() && CRS != "WGS_84")
        CRAS_DEBUG_NAMED("libav_stream",
          "Altitude is given in CRS %s which is unsupported. Reading it as if in WGS_84.", CRS.c_str());

      const auto parseCoord = [](const std::string& intPartStr, const std::string& fracPartStr)
      {
        double result = 0.0;
        if (intPartStr.length() >= 6)
        {
          result += cras::parseDouble(intPartStr.substr(intPartStr.length() - 2) + "." + fracPartStr) / 3600.0;
          result += cras::parseUInt32(intPartStr.substr(intPartStr.length() - 4, 2), 10) / 60.0;
          result += cras::parseUInt32(intPartStr.substr(0, intPartStr.length() - 4), 10);
        }
        else if (intPartStr.length() >= 4)
        {
          result += cras::parseDouble(intPartStr.substr(intPartStr.length() - 2) + "." + fracPartStr) / 60.0;
          result += cras::parseUInt32(intPartStr.substr(0, intPartStr.length() - 2), 10);
        }
        else
        {
          result += cras::parseDouble(intPartStr + "." + fracPartStr);
        }
        return result;
      };

      try
      {
        const auto lat = parseCoord(latIntPartStr, latFracPartStr) * latSign;
        const auto lon = parseCoord(lonIntPartStr, lonFracPartStr) * lonSign;
        const auto alt = cras::parseDouble(altIntPartStr + "." + altFracPartStr) * altSign;

        cras::TempLocale l(LC_ALL, "en_US.UTF-8");
        CRAS_DEBUG_NAMED("libav_stream",
          "GPS latitude %.06f° has been read from movie metadata %s", lat, locationEntry->key);
        CRAS_DEBUG_NAMED("libav_stream",
          "GPS longitude %.06f° has been read from movie metadata %s", lon, locationEntry->key);
        CRAS_DEBUG_NAMED("libav_stream",
          "GPS altitude %.02f m.a.s.l has been read from movie metadata %s", alt, locationEntry->key);

        auto& navMsg = result.first.emplace();
        auto& gpsMsg = result.second.emplace();

        navMsg.latitude = gpsMsg.latitude = lat;
        navMsg.longitude = gpsMsg.longitude = lon;
        navMsg.altitude = gpsMsg.altitude = alt;
        navMsg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        gpsMsg.status.status = gps_common::GPSStatus::STATUS_FIX;
        gpsMsg.status.position_source = gps_common::GPSStatus::SOURCE_GPS;
        gpsMsg.status.motion_source = gps_common::GPSStatus::SOURCE_NONE;
        gpsMsg.status.orientation_source = gps_common::GPSStatus::SOURCE_NONE;

        const auto accuracyEntry = av_dict_get(
          this->data->avFormatContext->metadata, "com.apple.quicktime.location.accuracy.horizontal", nullptr, 0);
        if (accuracyEntry != nullptr)
        {
          const auto accuracy = cras::parseDouble(accuracyEntry->value);
          gpsMsg.err_horz = accuracy;
          gpsMsg.position_covariance[0] = gpsMsg.position_covariance[4] = accuracy * accuracy;
          gpsMsg.position_covariance[8] = 10000 * 10000;
          gpsMsg.position_covariance_type = gps_common::GPSFix::COVARIANCE_TYPE_APPROXIMATED;
          CRAS_DEBUG_NAMED("libav_stream",
            "GPS horizontal accuracy %f m has been read from movie metadata %s", accuracy, accuracyEntry->key);
        }
        else
        {
          gpsMsg.position_covariance_type = gps_common::GPSFix::COVARIANCE_TYPE_UNKNOWN;
        }
        navMsg.position_covariance = gpsMsg.position_covariance;
        navMsg.position_covariance_type = gpsMsg.position_covariance_type;

        return {navMsg, gpsMsg};
      }
      catch (const std::invalid_argument&) {}
    }
  }

  return {cras::nullopt, cras::nullopt};
}

MetadataExtractor::Ptr LibavStreamMetadataExtractorPlugin::getExtractor(const MetadataExtractorParams& params)
{
  if (params.log == nullptr || params.avFormatContext == nullptr)
    return nullptr;

  return std::make_shared<LibavStreamMetadataExtractor>(
    params.log, params.avFormatContext, params.info->movieStreamIndex());
}

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::LibavStreamMetadataExtractorPlugin, movie_publisher::MetadataExtractorPlugin)
