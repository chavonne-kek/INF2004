/*
 *	wifi_secrets.example.h
 *
 *	Copy this file to wifi_secrets.h (same directory) and fill in your own
 *	network's credentials. wifi_secrets.h is listed in .gitignore -- it
 *	will never be committed, so it's safe to put your real password there.
 *
 *	Only used when built with NET=lwip (see build_make/pico_rp2040.mk and
 *	lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c).
 */

#ifndef WIFI_SECRETS_H
#define WIFI_SECRETS_H

#define WIFI_SSID     "your-network-name"
#define WIFI_PASSWORD "your-network-password"

/* Only used when built with MQTT=1 (also requires NET=lwip).
 * MQTT_BROKER_IP: the laptop running the broker.  "0.0.0.N" (IS25, 17 Nov)
 * = the car's own first three numbers with N last: a phone hotspot changes
 * its address range each time it is switched on, but the laptop keeps the
 * same last number, so "0.0.0.184" keeps working without a rebuild.  A full
 * address ("192.168.1.10") is used as it is. */
#define MQTT_BROKER_IP   "0.0.0.184"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID   "mtk3smp-pico"
#define MQTT_TOPIC       "mtk3smp-pico/heartbeat"

#endif /* WIFI_SECRETS_H */
