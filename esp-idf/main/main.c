
// Used Espressif IDF examples for sdspi, NMEA GPS Parser, MQTT-tls, WiFi Provisioning
// SEN5x esp-idf c files from Sensirion
// MHZ19 esp-idf c files from https://github.com/crisap94/MHZ19/tree/master

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nmea_parser.h"
#include <string.h>  // strcmp
#include "driver/i2c.h"
#include "sen5x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"
#include "cJSON.h"
#include "time.h"

#include <sys/time.h>
#include "esp_sntp.h"
#include "nvs_flash.h"

#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"

#include "mqtt_client.h" //provides important functions to connect with MQTT
#include "protocol_examples_common.h" //important for running different protocols in code
#include "esp_tls.h"
#include "esp_ota_ops.h"
#include <sys/param.h>
#include <stddef.h>
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "driver/gpio.h"
#include "mhz19b.h"
#include "driver/uart.h"

#define MHZ19B_TX 19
#define MHZ19B_RX 18

#include <wifi_provisioning/scheme_softap.h>

#include "qrcode.h"

#define EXAMPLE_PROV_SEC2_USERNAME          "wifiprov"
#define EXAMPLE_PROV_SEC2_PWD               "abcd1234"

#define Access_Key "c18oIPQVoUz9yOho" //Configure Access_Key

extern const uint8_t mqtt_eclipseprojects_io_pem_start[]   asm("_binary_mqtt_eclipseprojects_io_pem_start");
extern const uint8_t mqtt_eclipseprojects_io_pem_end[]   asm("_binary_mqtt_eclipseprojects_io_pem_end");

static const char *TAG = "sen55-gps-SD-mqtt";
/* This salt,verifier has been generated for username = "wifiprov" and password = "abcd1234"
 * IMPORTANT NOTE: For production cases, this must be unique to every device
 * and should come from device manufacturing partition.*/
static const char sec2_salt[] = {
    0x03, 0x6e, 0xe0, 0xc7, 0xbc, 0xb9, 0xed, 0xa8, 0x4c, 0x9e, 0xac, 0x97, 0xd9, 0x3d, 0xec, 0xf4
};

static const char sec2_verifier[] = {
    0x7c, 0x7c, 0x85, 0x47, 0x65, 0x08, 0x94, 0x6d, 0xd6, 0x36, 0xaf, 0x37, 0xd7, 0xe8, 0x91, 0x43,
    0x78, 0xcf, 0xfd, 0x61, 0x6c, 0x59, 0xd2, 0xf8, 0x39, 0x08, 0x12, 0x72, 0x38, 0xde, 0x9e, 0x24,
    0xa4, 0x70, 0x26, 0x1c, 0xdf, 0xa9, 0x03, 0xc2, 0xb2, 0x70, 0xe7, 0xb1, 0x32, 0x24, 0xda, 0x11,
    0x1d, 0x97, 0x18, 0xdc, 0x60, 0x72, 0x08, 0xcc, 0x9a, 0xc9, 0x0c, 0x48, 0x27, 0xe2, 0xae, 0x89,
    0xaa, 0x16, 0x25, 0xb8, 0x04, 0xd2, 0x1a, 0x9b, 0x3a, 0x8f, 0x37, 0xf6, 0xe4, 0x3a, 0x71, 0x2e,
    0xe1, 0x27, 0x86, 0x6e, 0xad, 0xce, 0x28, 0xff, 0x54, 0x46, 0x60, 0x1f, 0xb9, 0x96, 0x87, 0xdc,
    0x57, 0x40, 0xa7, 0xd4, 0x6c, 0xc9, 0x77, 0x54, 0xdc, 0x16, 0x82, 0xf0, 0xed, 0x35, 0x6a, 0xc4,
    0x70, 0xad, 0x3d, 0x90, 0xb5, 0x81, 0x94, 0x70, 0xd7, 0xbc, 0x65, 0xb2, 0xd5, 0x18, 0xe0, 0x2e,
    0xc3, 0xa5, 0xf9, 0x68, 0xdd, 0x64, 0x7b, 0xb8, 0xb7, 0x3c, 0x9c, 0xfc, 0x00, 0xd8, 0x71, 0x7e,
    0xb7, 0x9a, 0x7c, 0xb1, 0xb7, 0xc2, 0xc3, 0x18, 0x34, 0x29, 0x32, 0x43, 0x3e, 0x00, 0x99, 0xe9,
    0x82, 0x94, 0xe3, 0xd8, 0x2a, 0xb0, 0x96, 0x29, 0xb7, 0xdf, 0x0e, 0x5f, 0x08, 0x33, 0x40, 0x76,
    0x52, 0x91, 0x32, 0x00, 0x9f, 0x97, 0x2c, 0x89, 0x6c, 0x39, 0x1e, 0xc8, 0x28, 0x05, 0x44, 0x17,
    0x3f, 0x68, 0x02, 0x8a, 0x9f, 0x44, 0x61, 0xd1, 0xf5, 0xa1, 0x7e, 0x5a, 0x70, 0xd2, 0xc7, 0x23,
    0x81, 0xcb, 0x38, 0x68, 0xe4, 0x2c, 0x20, 0xbc, 0x40, 0x57, 0x76, 0x17, 0xbd, 0x08, 0xb8, 0x96,
    0xbc, 0x26, 0xeb, 0x32, 0x46, 0x69, 0x35, 0x05, 0x8c, 0x15, 0x70, 0xd9, 0x1b, 0xe9, 0xbe, 0xcc,
    0xa9, 0x38, 0xa6, 0x67, 0xf0, 0xad, 0x50, 0x13, 0x19, 0x72, 0x64, 0xbf, 0x52, 0xc2, 0x34, 0xe2,
    0x1b, 0x11, 0x79, 0x74, 0x72, 0xbd, 0x34, 0x5b, 0xb1, 0xe2, 0xfd, 0x66, 0x73, 0xfe, 0x71, 0x64,
    0x74, 0xd0, 0x4e, 0xbc, 0x51, 0x24, 0x19, 0x40, 0x87, 0x0e, 0x92, 0x40, 0xe6, 0x21, 0xe7, 0x2d,
    0x4e, 0x37, 0x76, 0x2f, 0x2e, 0xe2, 0x68, 0xc7, 0x89, 0xe8, 0x32, 0x13, 0x42, 0x06, 0x84, 0x84,
    0x53, 0x4a, 0xb3, 0x0c, 0x1b, 0x4c, 0x8d, 0x1c, 0x51, 0x97, 0x19, 0xab, 0xae, 0x77, 0xff, 0xdb,
    0xec, 0xf0, 0x10, 0x95, 0x34, 0x33, 0x6b, 0xcb, 0x3e, 0x84, 0x0f, 0xb9, 0xd8, 0x5f, 0xb8, 0xa0,
    0xb8, 0x55, 0x53, 0x3e, 0x70, 0xf7, 0x18, 0xf5, 0xce, 0x7b, 0x4e, 0xbf, 0x27, 0xce, 0xce, 0xa8,
    0xb3, 0xbe, 0x40, 0xc5, 0xc5, 0x32, 0x29, 0x3e, 0x71, 0x64, 0x9e, 0xde, 0x8c, 0xf6, 0x75, 0xa1,
    0xe6, 0xf6, 0x53, 0xc8, 0x31, 0xa8, 0x78, 0xde, 0x50, 0x40, 0xf7, 0x62, 0xde, 0x36, 0xb2, 0xba
};
#define EXAMPLE_MAX_CHAR_SIZE    256
#define MOUNT_POINT "/sdcard"
#define TIMEZONE_SECONDS_OFFSET (8 * 3600)  // Example: UTC+3 (Eastern European Time)

//microSD card pin definitions
#define PIN_NUM_MISO  GPIO_NUM_5
#define PIN_NUM_MOSI  GPIO_NUM_6
#define PIN_NUM_CLK   GPIO_NUM_4
#define PIN_NUM_CS    GPIO_NUM_7

#define TIME_ZONE (+8)   //Beijing Time
#define YEAR_BASE (2000) //date in GPS starts from 2000
static bool mqtt_connected = true;
typedef long int32_t;
// Define global variables to store GPS data
static float latitude = 0.0;
static float longitude = 0.0;
// Define the length for the datetime string
#define DATETIME_STRING_LENGTH 30  // YYYY-MM-DD HH:MM:SS

// Define a global datetime string variable
char datetimeGPS[DATETIME_STRING_LENGTH];

#define EXAMPLE_ESP_WIFI_SSID      "PLDTHOMEFIBRBrEuK"
#define EXAMPLE_ESP_WIFI_PASS      "PLDTWIFIDfPhp"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5
///* FreeRTOS event group to signal when we are connected*/
//static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define CONFIG_BROKER_BIN_SIZE_TO_SEND 20000
static void send_binary(esp_mqtt_client_handle_t client)
{
    esp_partition_mmap_handle_t out_handle;
    const void *binary_address;
    const esp_partition_t *partition = esp_ota_get_running_partition();
    esp_partition_mmap(partition, 0, partition->size, ESP_PARTITION_MMAP_DATA, &binary_address, &out_handle);
    // sending only the configured portion of the partition (if it's less than the partition size)
    int binary_size = MIN(CONFIG_BROKER_BIN_SIZE_TO_SEND, partition->size);
    int msg_id = esp_mqtt_client_publish(client, "/topic/binary", binary_address, binary_size, 0, 0);
    ESP_LOGI(TAG, "binary sent with msg_id=%d", msg_id);
}

static esp_err_t example_get_sec2_salt(const char **salt, uint16_t *salt_len) {

    ESP_LOGI(TAG, "Development mode: using hard coded salt");
    *salt = sec2_salt;
    *salt_len = sizeof(sec2_salt);
    return ESP_OK;
}
static esp_err_t example_get_sec2_verifier(const char **verifier, uint16_t *verifier_len) {

    ESP_LOGI(TAG, "Development mode: using hard coded verifier");
    *verifier = sec2_verifier;
    *verifier_len = sizeof(sec2_verifier);
    return ESP_OK;
}
/* Signal Wi-Fi events on this event-group */
const int WIFI_CONNECTED_EVENT = BIT0;
static EventGroupHandle_t wifi_event_group;

#define PROV_QR_VERSION         "v1"
#define PROV_TRANSPORT_SOFTAP   "softap"
#define PROV_TRANSPORT_BLE      "ble"
#define QRCODE_BASE_URL         "https://espressif.github.io/esp-jumpstart/qrcode.html"

static esp_err_t connect_to_saved_credentials(void) {
    nvs_handle_t nvs_handle;
    size_t required_size;
    wifi_config_t wifi_config = { 0 };
    esp_err_t err;

    // Open NVS
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS handle!");
        return err;
    }

    // Read stored SSID
    err = nvs_get_str(nvs_handle, "ssid", NULL, &required_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No stored Wi-Fi credentials found in NVS!");
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_get_str(nvs_handle, "ssid", (char *)wifi_config.sta.ssid, &required_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SSID from NVS");
        nvs_close(nvs_handle);
        return err;
    }

    // Read stored Password
    err = nvs_get_str(nvs_handle, "password", NULL, &required_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get password from NVS");
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_get_str(nvs_handle, "password", (char *)wifi_config.sta.password, &required_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get password from NVS");
        nvs_close(nvs_handle);
        return err;
    }

    // Close NVS
    nvs_close(nvs_handle);

    // Set Wi-Fi configuration
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));

    // Start Wi-Fi and attempt to connect
    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Wi-Fi");
        return err;
    }

    err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi");
    }

    return err;
}
/* Event handler for catching system events */
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{

    static int retries;

    if (event_base == WIFI_PROV_EVENT) {
        switch (event_id) {
            case WIFI_PROV_START:
                ESP_LOGI(TAG, "Provisioning started");
                break;
            case WIFI_PROV_CRED_RECV: {
                wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
                ESP_LOGI(TAG, "Received Wi-Fi credentials"
                         "\n\tSSID     : %s\n\tPassword : %s",
                         (const char *) wifi_sta_cfg->ssid,
                         (const char *) wifi_sta_cfg->password);

                // Store received credentials in NVS
                nvs_handle_t nvs_handle;
                esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
                if (err == ESP_OK) {
                    nvs_set_str(nvs_handle, "ssid", (const char *)wifi_sta_cfg->ssid);
                    nvs_set_str(nvs_handle, "password", (const char *)wifi_sta_cfg->password);
                    nvs_commit(nvs_handle);
                    nvs_close(nvs_handle);
                } else {
                    ESP_LOGE(TAG, "Failed to open NVS handle!");
                }

                break;
            }
            case WIFI_PROV_CRED_FAIL: {
                wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t *)event_data;
                ESP_LOGE(TAG, "Provisioning failed!\n\tReason : %s"
                         "\n\tPlease reset to factory and retry provisioning",
                         (*reason == WIFI_PROV_STA_AUTH_ERROR) ?
                         "Wi-Fi station authentication failed" : "Wi-Fi access-point not found");

                retries++;
                if (retries >= 10) {
                    ESP_LOGI(TAG, "Failed to connect with provisioned AP, reseting provisioned credentials");
                    wifi_prov_mgr_reset_sm_state_on_failure();
                    retries = 0;
                    connect_to_saved_credentials();
                }
                break;
            }
            case WIFI_PROV_CRED_SUCCESS:
                ESP_LOGI(TAG, "Provisioning successful");

                retries = 0;
                break;
            case WIFI_PROV_END:
                /* De-initialize manager once provisioning is finished */
                wifi_prov_mgr_deinit();
                break;
            default:
                break;
        }
    } else if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGI(TAG, "Disconnected. Connecting to the AP again...");
//                esp_wifi_connect();
                if (connect_to_saved_credentials() != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to connect to saved credentials.");
                    esp_wifi_connect();
                }
                break;

            case WIFI_EVENT_AP_STACONNECTED:
                ESP_LOGI(TAG, "SoftAP transport: Connected!");
                break;
            case WIFI_EVENT_AP_STADISCONNECTED:
                ESP_LOGI(TAG, "SoftAP transport: Disconnected!");
                break;
            default:
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connected with IP Address:" IPSTR, IP2STR(&event->ip_info.ip));
        /* Signal main application to continue execution */
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
    } else if (event_base == PROTOCOMM_SECURITY_SESSION_EVENT) {
        switch (event_id) {
            case PROTOCOMM_SECURITY_SESSION_SETUP_OK:
                ESP_LOGI(TAG, "Secured session established!");
                break;
            case PROTOCOMM_SECURITY_SESSION_INVALID_SECURITY_PARAMS:
                ESP_LOGE(TAG, "Received invalid security parameters for establishing secure session!");
                break;
            case PROTOCOMM_SECURITY_SESSION_CREDENTIALS_MISMATCH:
                ESP_LOGE(TAG, "Received incorrect username and/or PoP for establishing secure session!");
                break;
            default:
                break;
        }
    }
}

static void wifi_init_sta(void)
{
    /* Start Wi-Fi in station mode */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void get_device_service_name(char *service_name, size_t max)
{
    uint8_t eth_mac[6];
    const char *ssid_prefix = "PROV_";
    esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
    snprintf(service_name, max, "%s%02X%02X%02X",
             ssid_prefix, eth_mac[3], eth_mac[4], eth_mac[5]);
}

/* Handler for the optional provisioning endpoint registered by the application.
 * The data format can be chosen by applications. Here, we are using plain ascii text.
 * Applications can choose to use other formats like protobuf, JSON, XML, etc.
 */
esp_err_t custom_prov_data_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
                                          uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
    if (inbuf) {
        ESP_LOGI(TAG, "Received data: %.*s", inlen, (char *)inbuf);
    }
    char response[] = "SUCCESS";
    *outbuf = (uint8_t *)strdup(response);
    if (*outbuf == NULL) {
        ESP_LOGE(TAG, "System out of memory");
        return ESP_ERR_NO_MEM;
    }
    *outlen = strlen(response) + 1; /* +1 for NULL terminating byte */

    return ESP_OK;
}

static void wifi_prov_print_qr(const char *name, const char *username, const char *pop, const char *transport)
{
    if (!name || !transport) {
        ESP_LOGW(TAG, "Cannot generate QR code payload. Data missing.");
        return;
    }
    char payload[150] = {0};
    if (pop) {

        snprintf(payload, sizeof(payload), "{\"ver\":\"%s\",\"name\":\"%s\"" \
                    ",\"username\":\"%s\",\"pop\":\"%s\",\"transport\":\"%s\"}",
                    PROV_QR_VERSION, name, username, pop, transport);
    } else {
        snprintf(payload, sizeof(payload), "{\"ver\":\"%s\",\"name\":\"%s\"" \
                    ",\"transport\":\"%s\"}",
                    PROV_QR_VERSION, name, transport);
    }

    ESP_LOGI(TAG, "Scan this QR code from the provisioning application for Provisioning.");
    esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
    esp_qrcode_generate(&cfg, payload);

    ESP_LOGI(TAG, "If QR code is not visible, copy paste the below URL in a browser.\n%s?data=%s", QRCODE_BASE_URL, payload);
}

static esp_err_t s_example_write_file(char *data, bool upload)
{
	const char *path;
	if (upload){
		path = MOUNT_POINT"/successC.txt";
	}else{
		path = MOUNT_POINT"/failedC.txt";
	}
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, "%s\n", data);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    return ESP_OK;
}

static esp_err_t s_example_read_file(const char *path)
{
    ESP_LOGI(TAG, "Reading file %s", path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    char line[EXAMPLE_MAX_CHAR_SIZE];
    fgets(line, sizeof(line), f);
    fclose(f);

    // strip newline
    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

    return ESP_OK;
}

void sd_init(void){
	esp_err_t ret;

	    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
	        .format_if_mount_failed = false,
	        .max_files = 5,
	        .allocation_unit_size = 16 * 1024
	    };
	    sdmmc_card_t *card;
	    const char mount_point[] = MOUNT_POINT;
	    ESP_LOGI(TAG, "Initializing SD card");

	    ESP_LOGI(TAG, "Using SPI peripheral");

	    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

	    spi_bus_config_t bus_cfg = {
	        .mosi_io_num = PIN_NUM_MOSI,
	        .miso_io_num = PIN_NUM_MISO,
	        .sclk_io_num = PIN_NUM_CLK,
	        .quadwp_io_num = -1,
	        .quadhd_io_num = -1,
	        .max_transfer_sz = 4000,
	    };
	    ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
	    if (ret != ESP_OK) {
	        ESP_LOGE(TAG, "Failed to initialize bus.");
	        return;
	    }

	    // This initializes the slot without card detect (CD) and write protect (WP) signals.
	    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
	    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
	    slot_config.gpio_cs = PIN_NUM_CS;
	    slot_config.host_id = host.slot;

	    ESP_LOGI(TAG, "Mounting filesystem");
	    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

	    if (ret != ESP_OK) {
	        if (ret == ESP_FAIL) {
	            ESP_LOGE(TAG, "Failed to mount filesystem. "
	                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
	        } else {
	            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
	                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
	        }
	        return;
	    }
	    ESP_LOGI(TAG, "Filesystem mounted");

	    // Card has been initialized, print its properties
	    sdmmc_card_print_info(stdout, card);

	    // Use POSIX and C standard library functions to work with files.

	    // First create a file.
	    const char *file_success = MOUNT_POINT"/successC.txt";
	    char data[EXAMPLE_MAX_CHAR_SIZE];
	    snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s %s!\n", "Hello, i am success", card->cid.name);
	    ret = s_example_write_file(data, true);
	    if (ret != ESP_OK) {
	        return;
	    }
	    const char *file_failed = MOUNT_POINT"/failedC.txt";
	    if (ret != ESP_OK) {
	        return;
	    }

}


static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}
bool file_exists(const char *filename) {
    FILE *file = fopen(filename, "r");
    if (file) {
        fclose(file);
        return true;
    }
    return false;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
//    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
//    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        esp_mqtt_client_publish(client, "everyJuan_Read","data",0,1,0);
        ESP_LOGI(TAG, "sent publish successful");
        mqtt_connected = true;
        bool upload = true;

        // If failed file exists, publish its contents and delete the file
        if (file_exists(MOUNT_POINT"/failedc.txt")) {
            FILE *f = fopen(MOUNT_POINT"/failedc.txt", "r");
            if (f != NULL) {
                char line[256];
                while (fgets(line, sizeof(line), f) != NULL) {
                    // Publish each line from the file
                    esp_mqtt_client_publish(client, "everyJuan_Read", line, 0, 1, 0);
                    vTaskDelay(10000 /portTICK_PERIOD_MS);
                    s_example_write_file(line, upload);
                }
                fclose(f);
                // Delete the failed file after publishing its contents
                unlink(MOUNT_POINT"/failedc.txt");
            }else{
            	ESP_LOGI(TAG, "Failed file does not exist");
            }
        }
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        mqtt_connected = false;
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED");

        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED");
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED");
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        if (strncmp(event->data, "send binary please", event->data_len) == 0) {
            ESP_LOGI(TAG, "Sending the binary");
            send_binary(client);
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}
// Global variable to store MQTT client handle
esp_mqtt_client_handle_t client;
static void mqtt_app_start(void)
{/*Depending on your website or cloud there could be more parameters in mqtt_cfg.*/
	//HiveMQ version
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = "mqtts://NodeA:NodeDeployment1@0e6bc2fecfc5477dbc224a6538ed5c0c.s1.eu.hivemq.cloud:8883",
            .verification.certificate = (const char *)mqtt_eclipseprojects_io_pem_start
        },
    };

    //Local MQTT version
//    const esp_mqtt_client_config_t mqtt_cfg={
//      .broker.address.uri = "mqtt://carecitizen:everyjuan@192.168.1.3:1883",
//  	.credentials.username = "carecitizen",
//  	.credentials.authentication.password = "everyjuan"
//
//    };

    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}


/**
 * @brief GPS Event Handler
 *
 * @param event_handler_arg handler specific arguments
 * @param event_base event base, here is fixed to ESP_NMEA_EVENT
 * @param event_id event id
 * @param event_data event specific arguments
 */
static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    gps_t *gps = NULL;
    switch (event_id) {
    case GPS_UPDATE:
        gps = (gps_t *)event_data;
        /* Update global variables with GPS data */
        latitude = gps->latitude;
        longitude = gps->longitude;
        // Adjust the year and hour for time zone
        int adjusted_year = gps->date.year + YEAR_BASE;
        int adjusted_hour = gps->tim.hour + TIME_ZONE;

        // Format the datetime string
        snprintf(datetimeGPS, DATETIME_STRING_LENGTH, "%04d-%02d-%02d %02d:%02d:%02d",
                 adjusted_year, gps->date.month, gps->date.day,
                 adjusted_hour, gps->tim.minute, gps->tim.second);
        /* print information parsed from GPS statements */
        ESP_LOGI(TAG, "%d/%d/%d %d:%d:%d => \r\n"
                 "\t\t\t\t\t\tlatitude   = %.05f°N\r\n"
                 "\t\t\t\t\t\tlongitude = %.05f°E\r\n"
                 "\t\t\t\t\t\taltitude   = %.02fm\r\n"
                 "\t\t\t\t\t\tspeed      = %fm/s",
                 gps->date.year + YEAR_BASE, gps->date.month, gps->date.day,
                 gps->tim.hour + TIME_ZONE, gps->tim.minute, gps->tim.second,
                 gps->latitude, gps->longitude, gps->altitude, gps->speed);
        break;
    case GPS_UNKNOWN:
        /* print unknown statements */
        ESP_LOGW(TAG, "Unknown statement:%s", (char *)event_data);
        break;
    default:
        break;
    }
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_NUM_0;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_3,
        .scl_io_num = GPIO_NUM_2,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}
// SNTP initialization function
void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "time.nist.gov");
    esp_sntp_init();
}

// Wait for time to be set
void obtain_time(void)
{
    initialize_sntp();

    // Wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
}
void write_to_csv(const char *filename, const char *csv_string) {
    FILE *file = fopen(filename, "a"); // Open the file in append mode
    if (file == NULL) {
        printf("Failed to open file for writing\n");
        return;
    }

    fprintf(file, "%s\n", csv_string); // Write the CSV string to the file
    fclose(file); // Close the file
}
void app_main(void)
{

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    sd_init();
    /* Initialize TCP/IP */
        ESP_ERROR_CHECK(esp_netif_init());

        /* Initialize the event loop */
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        wifi_event_group = xEventGroupCreate();

        /* Register our event handler for Wi-Fi, IP and Provisioning related events */
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

        ESP_ERROR_CHECK(esp_event_handler_register(PROTOCOMM_SECURITY_SESSION_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

        /* Initialize Wi-Fi including netif with default config */
        esp_netif_create_default_wifi_sta();

        esp_netif_create_default_wifi_ap();

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        /* Configuration for the provisioning manager */
        wifi_prov_mgr_config_t config = {

            .scheme = wifi_prov_scheme_softap,

            .scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE

        };

        /* Initialize provisioning manager with the
         * configuration parameters set above */
        ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

        bool provisioned = false;

        wifi_prov_mgr_reset_provisioning();

        /* Let's find out if the device is provisioned */
        ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));


        /* If device is not yet provisioned start provisioning service */
        if (!provisioned) {
            ESP_LOGI(TAG, "Starting provisioning");

            /* What is the Device Service Name that we want
             * This translates to :
             *     - Wi-Fi SSID when scheme is wifi_prov_scheme_softap
             *     - device name when scheme is wifi_prov_scheme_ble
             */
            char service_name[12];
            get_device_service_name(service_name, sizeof(service_name));


            wifi_prov_security_t security = WIFI_PROV_SECURITY_2;
            /* The username must be the same one, which has been used in the generation of salt and verifier */


            /* This pop field represents the password that will be used to generate salt and verifier.
             * The field is present here in order to generate the QR code containing password.
             * In production this password field shall not be stored on the device */
            const char *username  = EXAMPLE_PROV_SEC2_USERNAME;
            const char *pop = EXAMPLE_PROV_SEC2_PWD;


            /* This is the structure for passing security parameters
             * for the protocomm security 2.
             * If dynamically allocated, sec2_params pointer and its content
             * must be valid till WIFI_PROV_END event is triggered.
             */
            wifi_prov_security2_params_t sec2_params = {};

            ESP_ERROR_CHECK(example_get_sec2_salt(&sec2_params.salt, &sec2_params.salt_len));
            ESP_ERROR_CHECK(example_get_sec2_verifier(&sec2_params.verifier, &sec2_params.verifier_len));

            wifi_prov_security2_params_t *sec_params = &sec2_params;

            const char *service_key = NULL;

            wifi_prov_mgr_endpoint_create("custom-data");


            /* Start provisioning service */
            ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security, (const void *) sec_params, service_name, service_key));

            /* The handler for the optional endpoint created above.
             * This call must be made after starting the provisioning, and only if the endpoint
             * has already been created above.
             */
            wifi_prov_mgr_endpoint_register("custom-data", custom_prov_data_handler, NULL);

            /* Uncomment the following to wait for the provisioning to finish and then release
             * the resources of the manager. Since in this case de-initialization is triggered
             * by the default event loop handler, we don't need to call the following */
            // wifi_prov_mgr_wait();
            // wifi_prov_mgr_deinit();
            /* Print QR code for provisioning */


            wifi_prov_print_qr(service_name, username, pop, PROV_TRANSPORT_SOFTAP);

        } else {
            ESP_LOGI(TAG, "Already provisioned, starting Wi-Fi STA");

            /* We don't need the manager as device is already provisioned,
             * so let's release it's resources */
            wifi_prov_mgr_deinit();

            /* Start Wi-Fi station */
            wifi_init_sta();
        }

        /* Wait for Wi-Fi connection */
        xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, true, true, portMAX_DELAY);

        /* Start main application now */

    vTaskDelay(10000 /portTICK_PERIOD_MS); //delay is important cause we need to let it connect to wifi
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    mqtt_app_start(); // MQTT start app as shown above most important code for MQTT
	//uint8_t data[2];
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    // Obtain time
    obtain_time();
    int16_t co2;
    mhz19b_dev_t dev;
    bool autocal;
//    // Obtain time
//    obtain_time();
    if (mhz19b_init(&dev, UART_NUM_0, MHZ19B_TX, MHZ19B_RX) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize MHZ-19B sensor");
        return;
    }

    int16_t error = 0;
    while(1){
    	// Start Measurement
    	    error = sen5x_start_measurement();
    	        if (error) {
    	            printf("Error executing sen5x_start_measurement(): %i\n", error);
    	        }
    	        /* NMEA parser configuration */
    	        nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
    	        /* init NMEA parser library */
    	        nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);
    	        /* register event handler for NMEA parser library */
    	        nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);

    	        vTaskDelay(10000 / portTICK_PERIOD_MS);
    	        mhz19b_set_range(&dev, MHZ19B_RANGE_5000);
    	            	        mhz19b_set_auto_calibration(&dev, false);
    	            	        mhz19b_get_auto_calibration(&dev, &autocal);
    	            	        ESP_LOGI(TAG, "  autocal: %s", autocal ? "ON" : "OFF");

    	            	        //Start MHZ19 calibration for 20 minutes
    	            	    	mhz19b_start_calibration(&dev);
    	            	        ESP_LOGI(TAG, "MHZ-19B calibrating");
    	            	        vTaskDelay(1.2e6 / portTICK_PERIOD_MS);

    	            	        while (mhz19b_is_warming_up(&dev, true))
    	            	        {
    	            	            ESP_LOGI(TAG, "MHZ-19B is warming up");
    	            	            vTaskDelay(1000 / portTICK_PERIOD_MS);
    	            	        }
//    	            	        esp_err_t err = mhz19b_read_co2(&dev, &co2);
//    	            	                if (err == ESP_OK//    	            	                {
//    	            	                    ESP_LOGI(TAG, "CO2: %d", co2);
//    	            	                }
//    	            	                else
//    	            	                {
//    	            	                    ESP_LOGE(TAG, "Failed to read CO2 levels, error code: %d", err);
//    	            	                    // Additional debugging info
//    	            	                    ESP_LOGE(TAG, "UART number: %d, TX pin: %d, RX pin: %d", UART_NUM_1, MHZ19B_TX, MHZ19B_RX);
//    	            	                    // Additional delay before retrying to read CO2
//    	            	                    vTaskDelay(5000 / portTICK_PERIOD_MS);
//    	            	                }
    	            	        mhz19b_read_co2(&dev, &co2);
    	            	        ESP_LOGI(TAG, "CO2: %d", co2);
    	            	        vTaskDelay(5000 / portTICK_PERIOD_MS);
    	            // Read Measurement
    	            sensirion_i2c_hal_sleep_usec(2e6);

    	            uint16_t mass_concentration_pm1p0;
    	            uint16_t mass_concentration_pm2p5;
    	            uint16_t mass_concentration_pm4p0;
    	            uint16_t mass_concentration_pm10p0;
    	            int16_t ambient_humidity;
    	            int16_t ambient_temperature;
    	            int16_t voc_index;
    	            int16_t nox_index;

    	            error = sen5x_read_measured_values(
    	                &mass_concentration_pm1p0, &mass_concentration_pm2p5,
    	                &mass_concentration_pm4p0, &mass_concentration_pm10p0,
    	                &ambient_humidity, &ambient_temperature, &voc_index, &nox_index);

    	            if (error) {
    	                printf("Error executing sen5x_read_measured_values(): %i\n", error);
    	            } else {
    	                printf("Mass concentration pm1p0: %.1f ug/m3\n",
    	                       mass_concentration_pm1p0 / 10.0f);
    	                printf("Mass concentration pm2p5: %.1f ug/m3\n",
    	                       mass_concentration_pm2p5 / 10.0f);
    	                printf("Mass concentration pm4p0: %.1f ug/m3\n",
    	                       mass_concentration_pm4p0 / 10.0f);
    	                printf("Mass concentration pm10p0: %.1f ug/m3\n",
    	                       mass_concentration_pm10p0 / 10.0f);
    	                if (ambient_humidity == 0x7fff) {
    	                    printf("Ambient humidity: n/a\n");
    	                } else {
    	                    printf("Ambient humidity: %.1f %%RH\n",
    	                           ambient_humidity / 100.0f);
    	                }
    	                if (ambient_temperature == 0x7fff) {
    	                    printf("Ambient temperature: n/a\n");
    	                } else {
    	                    printf("Ambient temperature: %.1f C\n",
    	                           ambient_temperature / 200.0f);
    	                }
    	                if (voc_index == 0x7fff) {
    	                    printf("Voc index: n/a\n");
    	                } else {
    	                    printf("Voc index: %.1f\n", voc_index / 10.0f);
    	                }
    	                if (nox_index == 0x7fff) {
    	                    printf("Nox index: n/a\n");
    	                } else {
    	                    printf("Nox index: %.1f\n", nox_index / 10.0f);
    	                }
    	        }


    	        // Create JSON object
    	        cJSON *root = cJSON_CreateObject();
    	        // Get current time
    	        time_t now;
    	        struct tm timeinfo;
    	        time(&now);
    	        now += TIMEZONE_SECONDS_OFFSET;
    	        localtime_r(&now, &timeinfo);
    	        char datetime[64];
    	        strftime(datetime, sizeof(datetime), "%Y/%m/%d %H:%M:%S", &timeinfo);
    	        cJSON_AddStringToObject(root, "AccessKey", Access_Key);
    	        cJSON_AddStringToObject(root, "DateTime", datetimeGPS);
    	        cJSON_AddStringToObject(root, "DateTimeNTP", datetime);

    	        if (ambient_humidity != 0x7fff) {
    	            cJSON_AddNumberToObject(root, "humidity", ambient_humidity / 100.0f);
    	        } else {
    	            cJSON_AddStringToObject(root, "humidity", "n/a");
    	        }

    	        if (ambient_temperature != 0x7fff) {
    	            cJSON_AddNumberToObject(root, "temperature", ambient_temperature / 200.0f);
    	        } else {
    	            cJSON_AddStringToObject(root, "temperature", "n/a");
    	        }
    	        // Add sensor data to JSON
//    	        cJSON_AddNumberToObject(root, "PM1.0", mass_concentration_pm1p0 / 10.0f);
    	        cJSON_AddNumberToObject(root, "PM2p5", mass_concentration_pm2p5 / 10.0f);
//    	        cJSON_AddNumberToObject(root, "PM4.0", mass_concentration_pm4p0 / 10.0f);
    	        cJSON_AddNumberToObject(root, "PM10", mass_concentration_pm10p0 / 10.0f);

    	        //Add CO2 data to JSON
    	        cJSON_AddNumberToObject(root, "CO2", co2);

    	        char lat_str[20]; // buffer to hold the latitude string
    	        char lon_str[20]; // buffer to hold the longitude string

    	        // Convert double to string
    	        snprintf(lat_str, sizeof(lat_str), "%.5f", latitude);
    	        snprintf(lon_str, sizeof(lon_str), "%.5f", longitude);

    	        // Add latitude and longitude as strings to JSON object
    	        cJSON_AddStringToObject(root, "Latitude", lat_str);
    	        cJSON_AddStringToObject(root, "Longitude", lon_str);

    	        // Print the formatted JSON string
    	        char *json_string = cJSON_PrintUnformatted(root);
    	        if (json_string != NULL) {
    	            printf("Formatted JSON:\n%s\n", json_string);
    	//            free(json_string);
    	        }

    	        // Format data for CSV
    	        // Format CSV string
    	        char csv_string[512];
    	        snprintf(csv_string, sizeof(csv_string), "%s,%s,%s,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f",
    	        		 Access_Key, datetimeGPS, datetime,
    	                 ambient_humidity / 100.0f, ambient_temperature / 200.0f,
    	                 mass_concentration_pm2p5 / 10.0f, mass_concentration_pm10p0 / 10.0f,
    	                 latitude, longitude);
    	        // Write CSV string to file
    	        write_to_csv(MOUNT_POINT"/data.csv", csv_string);

    	        // Get MQTT client and publish JSON
    	        if (mqtt_connected && json_string != NULL) {
    	        	bool upload = true;
    	            esp_mqtt_client_publish(client, "everyJuan_Read", json_string, 0,1,0); // Publish the JSON data
    	            s_example_write_file(json_string, upload);

    	        } else {
    	        	bool upload = false;
    	            ESP_LOGE(TAG, "Failed to initialize MQTT client");
    	            s_example_write_file(json_string, upload);
    	        }

    	        // Clean up
    	        free(json_string);
    	        error = sen5x_stop_measurement();
    	        if (error) {
    	            printf("Error executing sen5x_stop_measurement(): %i\n", error);
    	        }
    	        /* unregister event handler */
    	                nmea_parser_remove_handler(nmea_hdl, gps_event_handler);
    	                /* deinit NMEA parser library */
    	                nmea_parser_deinit(nmea_hdl);
    	      // Delay before next loop iteration
    	     vTaskDelay(60000 / portTICK_PERIOD_MS); // Delay for 60 seconds
    }

}


