#include "setupApi.h"

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <SD.h>
#include <RTClib.h>

extern AsyncWebServer server;

#pragma pack(push, 1)
struct MetricRecord
{
	uint32_t ts;
	int16_t value;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct EventRecord
{
	uint32_t ts;
	uint8_t subtype;
	int16_t value;
};
#pragma pack(pop)
struct EventRecordLegacy
{
	uint32_t ts;
	uint8_t subtype;
	uint8_t _pad;
	int16_t value;
};
void ensureNotesDir()
{
	if (!SD.exists("/notes"))
	{
		SD.mkdir("/notes");
	}
}
void addCORS1(AsyncWebServerResponse *response)
{
	response->addHeader("Access-Control-Allow-Origin", "*");
	response->addHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
	response->addHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
	response->addHeader("Access-Control-Expose-Headers", "X-Total-Records, X-Total-Bytes, X-Offset, X-Limit");
}
AsyncWebServerResponse *withCORS(AsyncWebServerResponse *res)
{
	res->addHeader("Access-Control-Allow-Origin", "*");
	res->addHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
	res->addHeader("Access-Control-Allow-Headers", "Content-Type");
	res->addHeader("Access-Control-Expose-Headers", "X-Total-Records, X-Total-Bytes, X-Offset, X-Limit");
	return res;
}
String pathFor(const String &uid)
{
	return "/notes/" + uid + ".json";
}

String tmpFor(const String &uid)
{
	return "/notes/" + uid + ".tmp";
}
void handleGetAllNotes(AsyncWebServerRequest *request)
{
	AsyncResponseStream *res =
			request->beginResponseStream("application/json");

	res->print("[");

	File dir = SD.open("/notes");

	bool first = true;
	File f;

	while ((f = dir.openNextFile()))
	{
		if (!first)
			res->print(",");

		while (f.available())
		{
			uint8_t buf[256];
			size_t n = f.read(buf, sizeof(buf));
			res->write(buf, n);
		}

		f.close();
		first = false;
	}

	res->print("]");

	request->send(withCORS(res));
}

void handleGetPaged(AsyncWebServerRequest *request)
{
	int offset = 0;
	int limit = 5;

	if (request->hasParam("offset"))
		offset = request->getParam("offset")->value().toInt();

	if (request->hasParam("limit"))
		limit = request->getParam("limit")->value().toInt();

	AsyncResponseStream *res =
			request->beginResponseStream("application/json");

	res->print("[");

	File dir = SD.open("/notes");

	int i = 0;
	int sent = 0;
	bool first = true;

	File f;

	while ((f = dir.openNextFile()))
	{
		if (i++ < offset)
		{
			f.close();
			continue;
		}

		if (sent >= limit)
		{
			f.close();
			break;
		}

		if (!first)
			res->print(",");

		while (f.available())
		{
			uint8_t buf[256];
			size_t n = f.read(buf, sizeof(buf));
			res->write(buf, n);
		}

		f.close();

		sent++;
		first = false;
	}

	res->print("]");

	request->send(withCORS(res));
}

void handleGetNote(AsyncWebServerRequest *request)
{
	if (!request->hasParam("uid"))
	{
		request->send(withCORS(
				request->beginResponse(400, "application/json", "{\"status\":\"missing uid\"}")));
		return;
	}

	String uid = request->getParam("uid")->value();
	String path = "/notes/" + uid + ".json";

	if (!SD.exists(path))
	{
		request->send(withCORS(
				request->beginResponse(404, "application/json", "{\"status\":\"not found\"}")));
		return;
	}

	AsyncWebServerResponse *resp =
			request->beginResponse(SD, path, "application/json");

	request->send(withCORS(resp));
}

void handleCreate(
		AsyncWebServerRequest *request,
		uint8_t *data,
		size_t len,
		size_t index,
		size_t total)
{
	StaticJsonDocument<1024> doc;

	if (deserializeJson(doc, data, len))
	{
		request->send(withCORS(
				request->beginResponse(400, "application/json", "{\"status\":\"bad json\"}")));
		return;
	}

	const char *uid = doc["uid"];
	if (!uid || uid[0] == '\0')
	{
		request->send(withCORS(
				request->beginResponse(400, "application/json", "{\"status\":\"missing uid\"}")));
		return;
	}

	String path = "/notes/" + String(uid) + ".json";
	String tmp = path + ".tmp";

	File f = SD.open(tmp, FILE_WRITE);
	if (!f)
	{
		request->send(withCORS(
				request->beginResponse(500, "application/json", "{\"status\":\"file open failed\"}")));
		return;
	}

	StaticJsonDocument<1024> out;
	out["uid"] = uid;
	out["title"] = doc["title"] | "";
	out["text"] = doc["text"] | "";
	out["date"] = doc["date"] | "";

	serializeJson(out, f);
	f.close();

	SD.remove(path);
	SD.rename(tmp, path);

	request->send(withCORS(
			request->beginResponse(200, "application/json", "{\"status\":\"created\"}")));
}

void handleUpdate(
		AsyncWebServerRequest *request,
		uint8_t *data,
		size_t len,
		size_t index,
		size_t total)
{
	StaticJsonDocument<1024> doc;

	if (deserializeJson(doc, data, len))
	{
		request->send(withCORS(
				request->beginResponse(400, "application/json", "{\"status\":\"bad json\"}")));
		return;
	}

	const char *uid = doc["uid"];
	if (!uid)
	{
		request->send(withCORS(
				request->beginResponse(400, "application/json", "{\"status\":\"missing uid\"}")));
		return;
	}

	String path = "/notes/" + String(uid) + ".json";
	String tmp = path + ".tmp";

	File f = SD.open(tmp, FILE_WRITE);
	if (!f)
	{
		request->send(withCORS(
				request->beginResponse(500, "application/json", "{\"status\":\"file error\"}")));
		return;
	}

	serializeJson(doc, f);
	f.close();

	SD.remove(path);
	SD.rename(tmp, path);

	request->send(withCORS(
			request->beginResponse(200, "application/json", "{\"status\":\"updated\"}")));
}

void handleDelete(AsyncWebServerRequest *request)
{
	if (!request->hasParam("uid"))
	{
		request->send(withCORS(
				request->beginResponse(400, "application/json", "{\"status\":\"missing uid\"}")));
		return;
	}

	String uid = request->getParam("uid")->value();
	String path = "/notes/" + uid + ".json";

	if (!SD.exists(path))
	{
		request->send(withCORS(
				request->beginResponse(404, "application/json", "{\"status\":\"not found\"}")));
		return;
	}

	SD.remove(path);

	request->send(withCORS(
			request->beginResponse(200, "application/json", "{\"status\":\"deleted\"}")));
}

void setupApi()
{

	server.on("/api/metrics", HTTP_GET, [](AsyncWebServerRequest *request)
						{
    if (!request->hasParam("metric") ||
        !request->hasParam("year") ||
        !request->hasParam("month") ||
        !request->hasParam("day"))
    {
        request->send(400, "text/plain", "missing params");
        return;
    }

    const String metric = request->getParam("metric")->value();
    const int year   = request->getParam("year")->value().toInt();
    const int month  = request->getParam("month")->value().toInt();
    const int day    = request->getParam("day")->value().toInt();

    int offset = 0;
    int limit = 500;

    if (request->hasParam("offset"))
        offset = request->getParam("offset")->value().toInt();

    if (request->hasParam("limit"))
        limit = request->getParam("limit")->value().toInt();

    if (offset < 0) offset = 0;
    if (limit < 1) limit = 1;
    if (limit > 1000) limit = 1000;

    char path[96];
    snprintf(path, sizeof(path),
             "/metrics/%s/%04d/%02d/%02d.bin",
             metric.c_str(), year, month, day);

    if (!SD.exists(path)) {
        request->send(404, "text/plain", "not found");
        return;
    }

    File file = SD.open(path, FILE_READ);
    if (!file) {
        request->send(500, "text/plain", "failed to open file");
        return;
    }

    const size_t recSize = sizeof(MetricRecord);
    const size_t totalRecords = file.size() / recSize;

    if ((size_t)offset >= totalRecords) {
        file.close();
        request->send(416, "text/plain", "offset out of range");
        return;
    }

    size_t recordsToSend = limit;
    if ((size_t)offset + recordsToSend > totalRecords)
        recordsToSend = totalRecords - offset;

    if (!file.seek((size_t)offset * recSize)) {
        file.close();
        request->send(500, "text/plain", "seek failed");
        return;
    }

    AsyncResponseStream *res = request->beginResponseStream("text/csv");
    addCORS1(res);

    res->addHeader("X-Total-Records", String(totalRecords));
    res->addHeader("X-Offset", String(offset));
    res->addHeader("X-Limit", String(recordsToSend));

    if (offset == 0) {
        res->print("ts,value\n");
    }

    MetricRecord rec;
    char line[48];

    for (size_t i = 0; i < recordsToSend; i++) {
        if (file.read((uint8_t*)&rec, sizeof(rec)) != sizeof(rec)) {
            break;
        }

        int len = snprintf(line, sizeof(line),
                           "%lu,%.2f\n",
                           (unsigned long)rec.ts,
                           rec.value / 100.0f);

        if (len > 0) {
            res->write((const uint8_t*)line, len);
        }
    }

    file.close();
    request->send(res); });

	server.on("/api/logs", HTTP_GET, [](AsyncWebServerRequest *request)
						{
							if (!request->hasParam("type") ||
									!request->hasParam("year") ||
									!request->hasParam("month") ||
									!request->hasParam("day"))
							{
								request->send(400, "text/plain", "missing params");
								return;
							}

							const String type = request->getParam("type")->value();
							const int year = request->getParam("year")->value().toInt();
							const int month = request->getParam("month")->value().toInt();
							const int day = request->getParam("day")->value().toInt();

							int offset = 0;
							int limit = 2048;

							if (request->hasParam("offset"))
								offset = request->getParam("offset")->value().toInt();

							if (request->hasParam("limit"))
								limit = request->getParam("limit")->value().toInt();

							if (offset < 0)
								offset = 0;
							if (limit < 1)
								limit = 1;
							if (limit > 8192)
								limit = 8192;

							char path[96];
							snprintf(path, sizeof(path),
											 "/logs/%04d/%02d/%02d/%s.log",
											 year, month, day, type.c_str());

							if (!SD.exists(path))
							{
								request->send(404, "text/plain", "log not found");
								return;
							}

							File file = SD.open(path, FILE_READ);
							if (!file)
							{
								request->send(500, "text/plain", "failed to open file");
								return;
							}

							const size_t totalBytes = file.size();

							if ((size_t)offset >= totalBytes)
							{
								file.close();
								request->send(416, "text/plain", "offset out of range");
								return;
							}

							size_t bytesToSend = limit;
							if ((size_t)offset + bytesToSend > totalBytes)
								bytesToSend = totalBytes - offset;

							if (!file.seek((size_t)offset))
							{
								file.close();
								request->send(500, "text/plain", "seek failed");
								return;
							}

							AsyncResponseStream *response =
									request->beginResponseStream("text/plain");

							addCORS1(response);

							response->addHeader("X-Total-Bytes", String(totalBytes));
							response->addHeader("X-Offset", String(offset));
							response->addHeader("X-Limit", String(bytesToSend));

							uint8_t buf[256];
							size_t sent = 0;

							while (sent < bytesToSend)
							{
								size_t chunk = bytesToSend - sent;
								if (chunk > sizeof(buf))
									chunk = sizeof(buf);

								size_t n = file.read(buf, chunk);
								if (n == 0)
									break;

								response->write(buf, n);
								sent += n;
							}

							file.close();
							request->send(response); });

	server.on("/api/events", HTTP_GET, [](AsyncWebServerRequest *request)
						{
  if (!request->hasParam("type") ||
      !request->hasParam("year") ||
      !request->hasParam("month"))
  {
    request->send(400, "text/plain", "missing params");
    return;
  }

  const String type = request->getParam("type")->value();
  const int year = request->getParam("year")->value().toInt();
  const int month = request->getParam("month")->value().toInt();

  int offset = 0;
  int limit = 500;

  if (request->hasParam("offset"))
    offset = request->getParam("offset")->value().toInt();

  if (request->hasParam("limit"))
    limit = request->getParam("limit")->value().toInt();

  if (offset < 0) offset = 0;
  if (limit < 1) limit = 1;
  if (limit > 1000) limit = 1000;

  char monthStr[3];
  snprintf(monthStr, sizeof(monthStr), "%02d", month);

  char path[96];
  snprintf(path, sizeof(path),
           "/events/%s/%04d/%s.bin",
           type.c_str(), year, monthStr);

  if (!SD.exists(path))
  {
    request->send(404, "text/plain", "not found");
    return;
  }

  File file = SD.open(path, FILE_READ);
  if (!file)
  {
    request->send(500, "text/plain", "failed to open file");
    return;
  }

  const size_t recSize = sizeof(EventRecordLegacy);
  const size_t totalRecords = file.size() / recSize;

  if ((size_t)offset >= totalRecords)
  {
    file.close();
    request->send(416, "text/plain", "offset out of range");
    return;
  }

  size_t recordsToSend = limit;
  if ((size_t)offset + recordsToSend > totalRecords)
    recordsToSend = totalRecords - offset;

  if (!file.seek((size_t)offset * recSize))
  {
    file.close();
    request->send(500, "text/plain", "seek failed");
    return;
  }

  AsyncResponseStream *response = request->beginResponseStream("text/csv");
  addCORS1(response);

  response->addHeader("X-Total-Records", String(totalRecords));
  response->addHeader("X-Offset", String(offset));
  response->addHeader("X-Limit", String(recordsToSend));

  if (offset == 0)
  {
    response->print("ts,subtype,value\n");
  }

  EventRecordLegacy rec;
  char line[56];

  for (size_t i = 0; i < recordsToSend; i++)
  {
    if (file.read((uint8_t *)&rec, sizeof(rec)) != sizeof(rec))
      break;

    int len = snprintf(line, sizeof(line),
                       "%lu,%u,%.2f\n",
                       (unsigned long)rec.ts,
                       rec.subtype,
                       rec.value / 100.0f);

    if (len > 0)
    {
      response->write((const uint8_t *)line, len);
    }
  }

  file.close();
  request->send(response); });

	server.on("/note", HTTP_GET, handleGetNote);
	server.on("/note", HTTP_POST, [](AsyncWebServerRequest *req) {}, NULL, handleCreate);
	server.on("/note", HTTP_PUT, [](AsyncWebServerRequest *req) {}, NULL, handleUpdate);
	server.on("/note", HTTP_DELETE, handleDelete);
	server.on("/notes/paged", HTTP_GET, handleGetPaged);
	server.on("/notes", HTTP_GET, handleGetAllNotes);
}