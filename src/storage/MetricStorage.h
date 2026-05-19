#pragma once

#include <Arduino.h>
#include <SD.h>
#include "RTClib.h"

class RTC_DS3231;

#pragma pack(push, 1)

struct MetricRecord
{
	uint32_t ts;
	int16_t value;
};

#pragma pack(pop)

class MetricStorage
{
public:
	MetricStorage(const char *metricId);

	void append(float value, DateTime now);

private:
	String metricId;

	String buildFilePath(uint32_t ts);

	void ensureDirectories(uint32_t ts);
};