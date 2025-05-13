// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Extractor of image or movie metadata.
 * \author Martin Pecka
 */

#include <memory>
#include <vector>

#include <cras_cpp_common/log_utils.h>
#include <movie_publisher/metadata_extractor.h>

namespace movie_publisher
{

MetadataExtractor::MetadataExtractor(const cras::LogHelperPtr& log): HasLogger(log)
{
}

MetadataExtractor::~MetadataExtractor() = default;

TimedMetadataListener::~TimedMetadataListener() = default;

TimedMetadataExtractor::TimedMetadataExtractor(const cras::LogHelperPtr& log): MetadataExtractor(log)
{
}

void TimedMetadataExtractor::addTimedMetadataListener(const std::shared_ptr<TimedMetadataListener>& listener)
{
  this->listeners.emplace_back(listener);
}

void TimedMetadataExtractor::prepareTimedMetadata(const std::unordered_set<MetadataType>& metadataTypes)
{
}

size_t TimedMetadataExtractor::processTimedMetadata(
  const MetadataType type, const StreamTime& maxTime, const bool requireOptional)
{
  return false;
}

void TimedMetadataExtractor::seekTimedMetadata(const StreamTime& seekTime)
{
}

bool TimedMetadataExtractor::hasTimedMetadata() const
{
  return false;
}

}
