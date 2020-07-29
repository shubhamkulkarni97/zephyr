cmake_minimum_required(VERSION 3.8)

if (CONFIG_WIFI_ESP32)

set(WPA_SUPPLICANT_SRCS "${ESP_IDF_PATH}/components/wpa_supplicant/port/os_xtensa.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/aes-cbc.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/aes-internal-dec.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/aes-internal-enc.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/aes-internal.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/aes-unwrap.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/aes-wrap.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/bignum.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/crypto_mbedtls.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/crypto_internal-cipher.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/crypto_internal-modexp.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/crypto_internal-rsa.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/crypto_internal.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/des-internal.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/dh_group5.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/dh_groups.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/md4-internal.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/md5-internal.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/md5.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/ms_funcs.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/rc4.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/sha1-internal.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/sha1-pbkdf2.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/sha1.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/sha256-internal.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/crypto/sha256.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/fast_crypto/fast_aes-cbc.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/fast_crypto/fast_aes-unwrap.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/fast_crypto/fast_aes-wrap.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/fast_crypto/fast_crypto_internal-cipher.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/fast_crypto/fast_crypto_internal-modexp.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/fast_crypto/fast_crypto_internal.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/fast_crypto/fast_sha256-internal.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/fast_crypto/fast_sha256.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/utils/base64.c"
                   "${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/utils/ext_password.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/eap_peer/chap.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/eap_peer/eap.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/eap_peer/eap_common.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/eap_peer/eap_mschapv2.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/eap_peer/eap_peap.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/eap_peer/eap_peap_common.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/eap_peer/eap_tls.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/eap_peer/eap_tls_common.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/eap_peer/eap_ttls.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/eap_peer/mschapv2.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/tls/asn1.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/tls/bignum.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/tls/pkcs1.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/tls/pkcs5.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/tls/pkcs8.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/tls/rsa.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/tls/tls_internal.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/tls/tlsv1_client.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/tls/tlsv1_client_read.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/tls/tlsv1_client_write.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/tls/tlsv1_common.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/tls/tlsv1_cred.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/tls/tlsv1_record.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/tls/tlsv1_server.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/tls/tlsv1_server_read.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/tls/tlsv1_server_write.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wpa2/tls/x509v3.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wps/eap_common.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wps/uuid.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wps/wps.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wps/wps_attr_build.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wps/wps_attr_parse.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wps/wps_attr_process.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wps/wps_common.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wps/wps_dev_attr.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wps/wps_enrollee.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wps/wps_registrar.c"
                   #"${ESP_IDF_PATH}/components/wpa_supplicant/src/wps/wps_validate.c"
                   )

zephyr_sources(
  src/esp_wifi_adapter.c
  src/stubs.c
  src/esp_phy.c
  src/system_api.c
  src/esp_networking.c
  src/esp_event.c
  ${ESP_IDF_PATH}/components/soc/esp32/rtc_clk.c
  ${ESP_IDF_PATH}/components/esp32/ets_timer_legacy.c
  ${ESP_IDF_PATH}/components/esp32/esp_timer.c
  ${ESP_IDF_PATH}/components/esp32/esp_timer_esp32.c
  ${ESP_IDF_PATH}/components/esp32/fast_crypto_ops.c
  ${WPA_SUPPLICANT_SRCS}
  src/esp_shell.c
  )

#zephyr_sources(${ESP_IDF_PATH}/components/esp32/fast_crypto_ops.c)

zephyr_include_directories(
  src
  )

zephyr_compile_definitions(
  __ets__
  EMBEDDED_SUPP
  IEEE8021X_EAPOL
  EAP_PEER_METHOD
  EAP_MSCHAPv2
  EAP_PEAP
  USE_WPA2_TASK
  ESPRESSIF_USE
  ESP32_WORKAROUND
  CONFIG_ECC
  )

zephyr_link_libraries(
  ## wifi
  core
  net80211
  wpa
  core
  net80211
  pp
  coexist
  phy
  rtc
  wpa2
  wps
  mesh
  mbedTLS

  ## esp-idf wifi libs refer gcc libs symbols, so linked in libgcc
  gcc
  -L${ESP_IDF_PATH}/components/esp32
  -L${ESP_IDF_PATH}/components/esp32/lib
  -L${ESP_IDF_PATH}/components/wpa_supplicant
  -Wl,--defsym,phy_printf=printf
  -Wl,--defsym,net80211_printf=printf
  -Wl,--defsym,coexist_printf=printf
  )

endif()
