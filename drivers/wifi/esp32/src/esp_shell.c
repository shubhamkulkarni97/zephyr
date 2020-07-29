#include <shell/shell.h>
#include <shell/shell_uart.h>
#include "esp32/rom/ets_sys.h"
#include "esp_wifi.h"
#include "esp_err.h"

static int cmd_wifi_connect(const struct shell *shell, size_t argc,
			    char *argv[])
{
    wifi_config_t wifi_config = {0};

    strcpy((char *)&wifi_config.sta.ssid, argv[1]);
    if (argc > 2) {
        strcpy((char *)&wifi_config.sta.password, argv[2]);
    }
    esp_err_t ret = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    ret |= esp_wifi_connect();

    if (ret != ESP_OK) {
        ets_printf("Connect failed\n");
    }
	return 0;
}

static int cmd_wifi_disconnect(const struct shell *shell, size_t argc,
			       char *argv[])
{
	esp_err_t ret = esp_wifi_disconnect();
    if (ret != ESP_OK) {
        ets_printf("Failed to call WiFi Disconnect");
    }
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(esp_wifi_commands,
	SHELL_CMD(connect, NULL,
		  "\"<SSID>\""
		  "<PSK (optional: valid only for secured SSIDs)>",
		  cmd_wifi_connect),
    SHELL_CMD(disconnect, NULL, "Disconnect from Wifi AP",
		  cmd_wifi_disconnect),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(esp_wifi, &esp_wifi_commands, "Wifi commands", NULL);
