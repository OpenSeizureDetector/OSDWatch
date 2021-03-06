/*
 * OpenSeizureDetector - OSDWatch Version.
 *
 * OSDWatch - a simple accelerometer based seizure detector that runs on 
 * an ESP32 system on chip, with ADXL345 accelerometer attached by i2c 
 * interface.
 *
 * See http://openseizuredetector.org for more information.
 *
 * Copyright Graham Jones, 2015, 2016, 2017, 2018
 *
 * This file is part of OSDWatch.
 *
 * OSDWatch is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * OSDWatch is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with OSDWatch.  If not, see <http://www.gnu.org/licenses/>.
 */

//#include "adxl345.h"

// Macro to simulate the Pebble APP_LOG function
#define DEBUG true
#define APP_LOG_LEVEL_DEBUG 3
#ifdef DEBUG
#define APP_LOG(lvl,fmt, ...) printf("%s: " fmt "\n", "osd_app", ## __VA_ARGS__)
#else
#define APP_LOG(lvl,fmt, ...)
#endif

#define ESP_SDK 1   // Use Espressif RTOS SDK syntax, not openRTOS


// Define which GPIO pins are used to connect to the ADXL345 accelerometer.
#define SCL_PIN (19)  
#define SDA_PIN (18)  
#define INTR_PIN (13) // Wemos D1 Mini D7 = GPIO13
#define SETUP_PIN (12)  // Wemos D1 Mini D6 = GPIO12
#define USE_INTR 1

/* Filenames for system files */
#define ANALYSIS_SETTINGS_FNAME "analysis_settings.dat"
#define WIFI_SETTINGS_FNAME "wifi_settings.dat"

/* Network Configuration */
#define SETUP_WIFI_SSID "OSD_APP"
#define SETUP_WIFI_PASSWD "1234"
#define SSID_DEFAULT "OSD_WIFI"
#define PASSWD_DEFAULT "OSD_PASSWD"
#define OSD_SERVER_IP_DEFAULT "192.168.1.51"

/* ANALYSIS CONFIGURATION */

// default value for output/display settings
#define DEBUG_DEFAULT 1
#define DISPLAY_SPECTRUM_DEFAULT 1

// default values of seizure detector settings
#define ACC_BUF_LEN 50
#define SAMPLE_PERIOD_DEFAULT 5  // sec
#define SAMPLE_FREQ_DEFAULT 100  // Hz
#define NSAMP_MAX 512       // maximum number of samples of accelerometer
                            // data to collect (used to size arrays).
#define FREQ_CUTOFF_DEFAULT 12 // Hz - frequency above which movement is ignored.
#define DATA_UPDATE_PERIOD_DEFAULT 20 // number of seconds between sending
                            //data to phone
                            // note data is sent instantaneously if an alarm
                            // condition is detected.
#define SD_MODE_DEFAULT        0  // FFT Mode
#define SAMPLE_FREQ_DEFAULT    100 // Hz
#define ANALYSIS_PERIOD_DEFAULT 5  // seconds
#define ALARM_FREQ_MIN_DEFAULT 3  // Hz
#define ALARM_FREQ_MAX_DEFAULT 10 // Hz
#define WARN_TIME_DEFAULT      5 // sec
#define ALARM_TIME_DEFAULT     10 // sec
#define ALARM_THRESH_DEFAULT   100 // Power of spectrum between ALARM_FREQ_MIN and
                           // ALARM_FREQ_MAX that will indicate an alarm
                           // state.
#define ALARM_RATIO_THRESH_DEFAULT 30 // 10 x ROI power must be this value times
// the overall spectrum power (to filter out general movement from frequency
// specific movement.

// default values of fall detector settings
#define FALL_ACTIVE_DEFAULT 0  // 0 = fall detection inactive.
#define FALL_THRESH_MIN_DEFAULT 200 // milli-g
#define FALL_THRESH_MAX_DEFAULT 800 // milli-g
#define FALL_WINDOW_DEFAULT     1500 // milli-secs

// default mute time
#define MUTE_PERIOD_DEFAULT 300  // number of seconds to mute alarm following
                                 // long press of UP button.

// default manual alarm time
#define MAN_ALARM_PERIOD_DEFAULT 300 // number of seconds that manual alarm is
                                     // raised following long press of DOWN
                                     // button

/* Display Configuration */
#define BATT_SIZE 30  // pixels.
#define CLOCK_SIZE 37  // pixels.
#define ALARM_SIZE 30  // pixels.
#define SPEC_SIZE 30   // pixels


/* Phone Communications */
// Analysis Results
#define KEY_DATA_TYPE 1
#define KEY_ALARMSTATE 2
#define KEY_MAXVAL 3
#define KEY_MAXFREQ 4
#define KEY_SPECPOWER 5
// Settings
#define KEY_SETTINGS 6    // Phone is requesting watch to send current settings.
#define KEY_ALARM_FREQ_MIN 7
#define KEY_ALARM_FREQ_MAX 8
#define KEY_WARN_TIME 9
#define KEY_ALARM_TIME 10
#define KEY_ALARM_THRESH 11
#define KEY_POS_MIN 12       // position of first data point in array
#define KEY_POS_MAX 13       // position of last data point in array.
#define KEY_SPEC_DATA 14     // Spectrum data
#define KEY_ROIPOWER 15
#define KEY_NMIN 16
#define KEY_NMAX 17
#define KEY_ALARM_RATIO_THRESH 18
#define KEY_BATTERY_PC 19
#define KEY_SET_SETTINGS 20  // Phone is asking us to update watch app settings.
#define KEY_FALL_THRESH_MIN 21
#define KEY_FALL_THRESH_MAX 22
#define KEY_FALL_WINDOW 23
#define KEY_FALL_ACTIVE 24
#define KEY_DATA_UPDATE_PERIOD 25
#define KEY_MUTE_PERIOD 26
#define KEY_MAN_ALARM_PERIOD 27
#define KEY_SD_MODE 28
#define KEY_SAMPLE_FREQ 29
#define KEY_RAW_DATA 30
#define KEY_NUM_RAW_DATA 31
#define KEY_DEBUG 32
#define KEY_DISPLAY_SPECTRUM 33
#define KEY_SAMPLE_PERIOD 34
#define KEY_VERSION_MAJOR 35
#define KEY_VERSION_MINOR 36
#define KEY_FREQ_CUTOFF 37
#define KEY_ALARM_ROI 38

// Values of the KEY_DATA_TYPE entry in a message
#define DATA_TYPE_RESULTS 1   // Analysis Results
#define DATA_TYPE_SETTINGS 2  // Settings
#define DATA_TYPE_SPEC 3      // FFT Spectrum (or part of a spectrum)
#define DATA_TYPE_RAW 4       // Raw accelerometer data.

// Values for ALARM_STATE
#define ALARM_STATE_OK 0   // no alarm
#define ALARM_STATE_WARN 1 // Warning
#define ALARM_STATE_ALARM 2 // Alarm
#define ALARM_STATE_FALL 3 // Fall Detected
#define ALARM_STATE_FAULT 4 // System Fault detected
#define ALARM_STATE_MAN_ALARM 5 // Manual Alarm
#define ALARM_STATE_MUTE 6  // Alarm Muted

// Values for SD_MODE
#define SD_MODE_FFT 0     // The original OpenSeizureDetector mode (FFT based)
#define SD_MODE_RAW 1     // Send raw, unprocessed data to the phone.
#define SD_MODE_FILTER 2  // Use digital filter rather than FFT.
#define SD_MODE_FFT_MULTI_ROI 3  // Use multiple ROI FFT analysis.


// FIXME:  Modify the code to use this structure, rather than loads of
// global variables!
typedef struct 
{
  int samplePeriod;     // sample period in seconds.
  int sampleFreq;      // sampling frequency in Hz
                            //    (must be one of 10,25,50 or 100)
  int freqCutoff;      // frequency above which movement is ignored.
  int dataUpdatePeriod; // period (in sec) between sending data to the phone
  int sdMode;          // Seizure Detector mode 0=normal, 1=raw, 2=filter
  int alarmFreqMin;    // Minimum frequency (in Hz) for analysis region of interest.
  int alarmFreqMax;    // Maximum frequency (in Hz) for analysis region of interest.
  int warnTime;        // number of seconds above threshold to raise warning
  int alarmTime;       // number of seconds above threshold to raise alarm.
  int alarmThresh;     // Alarm threshold (average power of spectrum within
                     //       region of interest.
  int alarmRatioThresh; // 10x Ratio of ROI power to Spectrum power to raise alarm.

  int fallActive;    // fall detection active (0=inactive)
  int fallThreshMin; // fall detection minimum (lower) threshold (milli-g)
  int fallThreshMax; // fall detection maximum (upper) threshold (milli-g)
  int fallWindow;    // fall detection window (milli-seconds).
} Sd_Settings;

typedef struct {
  char ssid[50];     // wifi ssid to connect to
  char passwd[50];   // wifi password
  char serverIp[50]; // ip address of server to connect to.
} Wifi_Settings;

typedef struct {
  int nFreqCutoff;     // bin number of cutoff frequency.
  int nSamp;           // number of samples in sampling period
                       //    (rounded up to a power of 2)
  int fftBits;          // Size of fft data array - nSamp = 2^(fftBits)
  int nMin, nMax;      // Bin number of region of interest boundaries.
  int nMins[4],nMaxs[4]; // Bin numbers for four regions of interest.
  int maxVal;       // Peak amplitude in spectrum.
  int maxLoc;       // Location in output array of peak.
  int maxFreq;      // Frequency corresponding to peak location.
  long specPower;   // Average power of whole spectrum.
  long roiPower;    // Average power of spectrum in region of interest
  int roiRatio;     // ratio of roiPower to specPower (x10)
  long roiPowers[4]; // array storing the four regions of interest powers
  int roiRatios[4]; // array storing the four ROI ratios.
  int freqRes;      // Actually 1000 x frequency resolution
  int simpleSpec[10];  // Simplified spectrum - 1 to 10 Hz bins.
  //ADXL345_IVector latestAccelData;  // Latest accelerometer readings received.
  //int32_t accData[NSAMP_MAX];   
  short fftResults[NSAMP_MAX/2];  // FFT results
  int accDataPos;   // Position in accData of last point in time series.
  int accDataFull;  // Flag so we know when we have a complete buffer full
                      // of data.
  int fallDetected;  // flag to say if fall is detected (<>0 is fall)

  int isManAlarm;     // flag to say if a manual alarm has been raised.
  int manAlarmTime;   // time (in sec) that manual alarm has been raised
  int manAlarmPeriod; // time (in sec) that manual alarm is raised for
  
  int isMuted;       // flag to say if alarms are muted.
  int muteTime;      // time (in sec) that alarms have been muted.
  int mutePeriod;    // the time to mute alarms following long press of
                     // UP button.
  
  int alarmState;    // 0 = OK, 1 = WARNING, 2 = ALARM, 3 = FALL
  int alarmRoi;      // id number of ROI causing alarm.
  int alarmCount;    // number of seconds that we have been in an alarm state.

} An_Data;  // analysis Data


/* GLOBAL VARIABLES */
// Settings (obtained from default constants or persistent storage)
extern int debug;            // enable or disable logging output
extern Sd_Settings sdS;        // SD setings structure.
extern An_Data anD;        // analysis data
extern Wifi_Settings wifiS;    // Wifi setings structure.

void heartTask(void *pvParam);

/* Functions */
// from comms.c
void sendSdData();
void sendRawData();
void comms_init();
void commsTask(void *pvParameters);



// from http_server.c
void httpd_task(void *pvParameters);

// from settings.c
void settings_init();
void readSdSettings();
void saveSdSettings();
void readWifiSettings();
void saveWifiSettings();
void wifiSettingsToString(char* buf, int len);
