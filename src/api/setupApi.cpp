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
}
AsyncWebServerResponse *withCORS(AsyncWebServerResponse *res)
{
	res->addHeader("Access-Control-Allow-Origin", "*");
	res->addHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
	res->addHeader("Access-Control-Allow-Headers", "Content-Type");
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

    String metric = request->getParam("metric")->value();
    String year   = request->getParam("year")->value();
    String month  = request->getParam("month")->value();
    String day    = request->getParam("day")->value();

    char path[128];
    snprintf(path, sizeof(path),
        "/metrics/%s/%s/%s/%s.bin",
        metric.c_str(),
        year.c_str(),
        month.c_str(),
        day.c_str()
    );

    if (!SD.exists(path)) {
        request->send(404, "text/plain", "not found");
        return;
    }

    File file = SD.open(path);

    AsyncResponseStream *res =
        request->beginResponseStream("text/csv");

    res->print("ts,value\n");

    MetricRecord rec;

    char line[64];

    while (file.read((uint8_t*)&rec, sizeof(rec)) == sizeof(rec)) {

        int len = snprintf(line, sizeof(line),
            "%u,%.2f\n",
            rec.ts,
            rec.value / 100.0f
        );

        res->write((uint8_t*)line, len);
    }

    file.close();

    request->send(res); });
	server.on("/api/logs",
						HTTP_GET,
						[](AsyncWebServerRequest *request)
						{
							if (
									!request->hasParam("type") ||
									!request->hasParam("year") ||
									!request->hasParam("month") ||
									!request->hasParam("day"))
							{
								request->send(400, "text/plain", "missing params");
								return;
							}

							String type = request->getParam("type")->value(); // relay/doser/system
							String year = request->getParam("year")->value();
							String month = request->getParam("month")->value();
							String day = request->getParam("day")->value();

							String path =
									"/logs/" + year + "/" + month + "/" + day + "/" + type + ".log";

							if (!SD.exists(path))
							{
								request->send(404, "text/plain", "log not found");
								return;
							}

							File file = SD.open(path);

							if (!file)
							{
								request->send(500, "text/plain", "failed to open file");
								return;
							}

							AsyncResponseStream *response =
									request->beginResponseStream("text/plain");

							while (file.available())
							{
								response->write(file.read());
							}

							file.close();

							addCORS1(response);
							request->send(response);
						});
	server.on("/api/events",
						HTTP_GET,
						[](AsyncWebServerRequest *request)
						{
							if (
									!request->hasParam("type") ||
									!request->hasParam("year") ||
									!request->hasParam("month"))
							{
								request->send(400, "text/plain", "missing params");
								return;
							}

							String type = request->getParam("type")->value();
							String year = request->getParam("year")->value();

							int m = request->getParam("month")->value().toInt();

							char monthStr[3];
							snprintf(monthStr, sizeof(monthStr), "%02d", m);

							String path =
									"/events/" + type + "/" +
									year + "/" + String(monthStr) + ".bin";

							Serial.println("EVENT PATH: " + path);

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

							AsyncResponseStream *response =
									request->beginResponseStream("text/csv");

							// CSV как у metrics (но с subtype добавили)
							response->println("ts,subtype,value");

							struct EventRecord
							{
								uint32_t ts;
								uint8_t subtype;
								int16_t value;
							};

							EventRecord rec;

							while (file.read((uint8_t *)&rec, sizeof(rec)) == sizeof(rec))
							{
								response->printf(
										"%u,%u,%.2f\n",
										rec.ts,
										rec.subtype,
										rec.value / 100.0f);
							}

							file.close();

							addCORS1(response);
							request->send(response);
						});
	server.on("/note", HTTP_GET, handleGetNote);
	server.on("/note", HTTP_POST, [](AsyncWebServerRequest *req) {}, NULL, handleCreate);
	server.on("/note", HTTP_PUT, [](AsyncWebServerRequest *req) {}, NULL, handleUpdate);
	server.on("/note", HTTP_DELETE, handleDelete);
	server.on("/notes/paged", HTTP_GET, handleGetPaged);
	server.on("/notes", HTTP_GET, handleGetAllNotes);
}