#pragma once
// Segoe Fluent Icons (Windows 11) / Segoe MDL2 Assets (Windows 10) code points, UTF-8 encoded.
#define ICON_REFRESH    "\xee\x9c\xac"   // E72C
#define ICON_SETTINGS   "\xee\x9c\x93"   // E713
#define ICON_WARNING    "\xee\x9e\xba"   // E7BA
#define ICON_CANCEL     "\xee\x9c\x91"   // E711
#define ICON_CHECK      "\xee\x9c\xbe"   // E73E
#define ICON_ADD        "\xee\x9c\x90"   // E710
#define ICON_DELETE     "\xee\x9d\x8d"   // E74D
#define ICON_USB        "\xee\xa2\x8e"   // E88E
#define ICON_WIFI       "\xee\x9c\x81"   // E701
#define ICON_BLUETOOTH  "\xee\x9c\x82"   // E702
#define ICON_GAME       "\xee\x9f\xbc"   // E7FC
#define ICON_LINK       "\xee\x9c\x9b"   // E71B
#define ICON_INFO       "\xee\xa5\x86"   // E946
#define ICON_DIAG       "\xee\xa7\x99"   // E9D9
#define ICON_COPY       "\xee\xa3\x88"   // E8C8
#define ICON_EDIT       "\xee\x9c\x8f"   // E70F
#define ICON_PLAY       "\xee\x9d\xa8"   // E768
#define ICON_PAUSE      "\xee\x9d\xa9"   // E769
#define ICON_STOP       "\xee\x9c\x9a"   // E71A
#define ICON_POWER      "\xee\x9f\xa8"   // E7E8
#define ICON_VIBRATE    "\xee\xa1\xb7"   // E877
#define ICON_HISTORY    "\xee\xa0\x9c"   // E81C
#define ICON_CMD        "\xee\x9d\x96"   // E756
#define ICON_SEARCH     "\xee\x9c\xa1"   // E721
#define ICON_FILTER     "\xee\x9c\x9c"   // E71C
#define ICON_LOCK       "\xee\x9c\xae"   // E72E
#define ICON_FOLDER     "\xee\xa2\xb7"   // E8B7
#define ICON_SHIELD     "\xee\xa8\x98"   // EA18
#define ICON_SYNC       "\xee\xa2\x95"   // E895
#define ICON_HOME       "\xee\xa0\x8f"   // E80F
#define ICON_PIN        "\xee\x9c\x87"   // E707
#define ICON_SWITCH     "\xee\xa2\xab"   // E8AB
#define ICON_DEVICES    "\xee\x9d\xb2"   // E772
#define ICON_HEADSET    "\xee\x9f\xb6"   // E7F6 Headphone
#define ICON_CHART      "\xee\xa7\x92"   // E9D2 AreaChart
#define ICON_SAVE       "\xee\x9d\x8e"   // E74E
#define ICON_CLEAR      "\xee\xa2\x94"   // E894
#define ICON_CHEVRON_R  "\xee\x9d\xac"   // E76C
#define ICON_CHEVRON_D  "\xee\x9c\x8d"   // E70D
#define ICON_LIGHTBULB  "\xee\xaa\x80"   // EA80 Lightbulb
#define ICON_HELP       "\xee\xa2\x97"   // E897
#define ICON_EXPORT     "\xee\xb7\xa1"   // EDE1
#define ICON_RECORD     "\xee\x9f\x88"   // E7C8 Record
#define ICON_GLOBE      "\xee\x9d\xb4"   // E774 Globe
#define ICON_LOCATION   "\xee\xa0\x9d"   // E81D Location
#define ICON_OPENFILE   "\xee\xa3\xa5"   // E8E5 OpenFile
#define ICON_BATTERY(n) ICON_BATTERY_TABLE[(n)]

// Segoe MDL2 / Fluent: Battery0..9 = E850..E859, Battery10 = E83F (not E85A - that is BatteryCharging0).
inline const char* const ICON_BATTERY_TABLE[11] = {
    "\xee\xa1\x90", "\xee\xa1\x91", "\xee\xa1\x92", "\xee\xa1\x93", "\xee\xa1\x94", "\xee\xa1\x95",
    "\xee\xa1\x96", "\xee\xa1\x97", "\xee\xa1\x98", "\xee\xa1\x99", "\xee\xa0\xbf",
};
// BatteryCharging0..8 = E85A..E862, BatteryCharging10 = E83E (E863 is a battery-saver leaf, skip it).
inline const char* const ICON_BATTERY_CHARGING_TABLE[11] = {
    "\xee\xa1\x9a", "\xee\xa1\x9b", "\xee\xa1\x9c", "\xee\xa1\x9d", "\xee\xa1\x9e", "\xee\xa1\x9f",
    "\xee\xa1\xa0", "\xee\xa1\xa1", "\xee\xa1\xa2", "\xee\xa1\xa2", "\xee\xa0\xbe",
};
#define ICON_BATTERY_CHARGING(n) ICON_BATTERY_CHARGING_TABLE[(n)]
