deps_config := \
	/home/Administrator/ESP8266_RTOS_SDK/components/app_update/Kconfig \
	/home/Administrator/ESP8266_RTOS_SDK/components/aws_iot/Kconfig \
	/home/Administrator/ESP8266_RTOS_SDK/components/console/Kconfig \
	/home/Administrator/ESP8266_RTOS_SDK/components/esp8266/Kconfig \
	/home/Administrator/ESP8266_RTOS_SDK/components/esp_http_client/Kconfig \
	/home/Administrator/ESP8266_RTOS_SDK/components/esp_http_server/Kconfig \
	/home/Administrator/ESP8266_RTOS_SDK/components/freertos/Kconfig \
	/home/Administrator/ESP8266_RTOS_SDK/components/libsodium/Kconfig \
	/home/Administrator/ESP8266_RTOS_SDK/components/log/Kconfig \
	/home/Administrator/ESP8266_RTOS_SDK/components/lwip/Kconfig \
	/home/Administrator/ESP8266_RTOS_SDK/components/mdns/Kconfig \
	/home/Administrator/ESP8266_RTOS_SDK/components/mqtt/Kconfig \
	/home/Administrator/ESP8266_RTOS_SDK/components/newlib/Kconfig \
	/home/Administrator/ESP8266_RTOS_SDK/components/pthread/Kconfig \
	/home/Administrator/ESP8266_RTOS_SDK/components/spiffs/Kconfig \
	/home/Administrator/ESP8266_RTOS_SDK/components/ssl/Kconfig \
	/home/Administrator/ESP8266_RTOS_SDK/components/tcpip_adapter/Kconfig \
	/home/Administrator/ESP8266_RTOS_SDK/components/util/Kconfig \
	/home/Administrator/ESP8266_RTOS_SDK/components/vfs/Kconfig \
	/home/Administrator/ESP8266_RTOS_SDK/components/wifi_provisioning/Kconfig \
	/home/Administrator/ESP8266_RTOS_SDK/components/wpa_supplicant/Kconfig \
	/home/Administrator/ESP8266_RTOS_SDK/components/bootloader/Kconfig.projbuild \
	/home/Administrator/ESP8266_RTOS_SDK/components/esptool_py/Kconfig.projbuild \
	/home/Administrator/ESP8266_RTOS_SDK/components/partition_table/Kconfig.projbuild \
	/home/Administrator/ESP8266_RTOS_SDK/Kconfig

include/config/auto.conf: \
	$(deps_config)


$(deps_config): ;
