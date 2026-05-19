#include "MetricStorage.h"

#include <RTClib.h>


MetricStorage::MetricStorage(const char *metricId)
		: metricId(metricId)
{
}

void MetricStorage::append(float value, DateTime now)
{
// 	String pathDebug = buildFilePath(now.unixtime());
// Serial.println("METRIC WRITE: " + pathDebug);
	MetricRecord rec;
	rec.ts = now.unixtime();
	rec.value = (int16_t)(value * 100.0f);

	ensureDirectories(rec.ts);

	String path = buildFilePath(rec.ts);

	File file = SD.open(path, FILE_APPEND);

	if (!file)
	{
		Serial.println("SD open failed");
		return;
	}

	file.write((uint8_t *)&rec, sizeof(rec));

	file.close();
}

String MetricStorage::buildFilePath(uint32_t ts)
{
	DateTime dt(ts);

	char path[128];

	snprintf(
			path,
			sizeof(path),
			"/metrics/%s/%04d/%02d/%02d.bin",
			metricId.c_str(),
			dt.year(),
			dt.month(),
			dt.day());

	return String(path);
}

void MetricStorage::ensureDirectories(uint32_t ts)
{
	DateTime dt(ts);

	if (!SD.exists("/metrics"))
	{
		SD.mkdir("/metrics");
	}

	String root = "/metrics/" + metricId;

	if (!SD.exists(root))
	{
		SD.mkdir(root);
	}

	char y[64];
	snprintf(y, sizeof(y),
					 "/metrics/%s/%04d",
					 metricId.c_str(),
					 dt.year());

	if (!SD.exists(y))
	{
		SD.mkdir(y);
	}

	char m[64];
	snprintf(m, sizeof(m),
					 "/metrics/%s/%04d/%02d",
					 metricId.c_str(),
					 dt.year(),
					 dt.month());

	if (!SD.exists(m))
	{
		SD.mkdir(m);
	}
}