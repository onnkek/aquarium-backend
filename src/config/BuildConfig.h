#pragma once

#define AQUARIUM_API_VERSION "v1"
#define AQUARIUM_ENABLE_DEBUG_LOGS 1
// Split production config. Runtime is stored in AppConfig structs, while persistence is split by subsystem.
#define AQUARIUM_CONFIG_DIR "/config"
#define AQUARIUM_CONFIG_SYSTEM_PATH "/config/system.json"
#define AQUARIUM_CONFIG_DOSER_PATH "/config/doser.json"
#define AQUARIUM_CONFIG_RELAYS_PATH "/config/relays.json"
#define AQUARIUM_CONFIG_TEMP_PATH "/config/temp.json"
#define AQUARIUM_CONFIG_ARGB_PATH "/config/argb.json"

// Persistent runtime state is intentionally separated from config.
// Example: doser last-run dates are state, not equipment settings.
#define AQUARIUM_STATE_DIR "/state"
#define AQUARIUM_STATE_DOSER_PATH "/state/doser.json"

// Emergency mode stores a pre-emergency snapshot here so manual clear can
// restore previous relay/doser/temp/argb modes instead of leaving emergency policy persisted.
#define AQUARIUM_STATE_EMERGENCY_PATH "/state/emergency.json"

// Legacy monolithic config is supported only for migration/import/export compatibility.
#define AQUARIUM_LEGACY_CONFIG_PATH "/config.json"
#define AQUARIUM_LEGACY_CONFIG_TMP_PATH "/config.json.tmp"
#define AQUARIUM_LEGACY_CONFIG_BAK_PATH "/config.json.bak"
#define AQUARIUM_MAX_CONFIG_JSON_BYTES 24576
#define AQUARIUM_MAX_NOTE_JSON_BYTES 2048
#define AQUARIUM_LOG_QUEUE_SIZE 32
#define AQUARIUM_EVENT_QUEUE_SIZE 32
#define AQUARIUM_METRIC_QUEUE_SIZE 128

#define AQUARIUM_STORAGE_BACKOFF_MS 10000

// SD policy.
// FTP is required and must behave like the working legacy firmware.
// Internal background SD writers remain disabled by default; FTP and static SD web are served directly by the FTP/AsyncWebServer libraries like legacy.
#define AQUARIUM_ENABLE_FTP 1
#define AQUARIUM_ENABLE_SD_STATIC_WEB 1
#define AQUARIUM_ENABLE_SD_JOURNAL 1
#define AQUARIUM_ENABLE_SD_EVENTS 1


// FTP uses the raw SD object from the FTP library. Internal background writers must pause while an FTP control/data session is active.
#define AQUARIUM_PAUSE_INTERNAL_SD_WRITES_DURING_FTP 1

// Keep SD init identical to the working legacy firmware: SPI.begin(pins), then SD.begin(CS).
#define AQUARIUM_USE_LEGACY_SD_INIT 1
