#include <Arduino.h>

// ESP32_cam_node, upgraded with una additional udp layer 5 control protocol and configurable telegram bot
// PLEASE ALSO INSTALL THE ArduinoJson library by Benoit Blanchon

#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>
#include "AsyncUDP.h"
#include "lwip/api.h"

// NVS includes, Non volatile storage - incorporated in the project folder

#include "ArduinoNvs.h"

// WIFI includes and variables section START

#define MY_NAME "e32cam003s01"  // hardcoded, at the moment

// LFCP defines START

#include "LFCP.h"

#define ALARMS_ENABLED 0  // 1 and enabled by default, the device works as a motion detector in case the contact with the command and control server is lost. 

AsyncUDP LFCPudp; //  udp "handle" for server A , must be passed when sending messages / hearbeats // udp layer 5 control protocol
AsyncUDP LFCPudpB; // udp "handle"  for server B , must be passed when sending messages / hearbeats 


// LFCP IP addresses for server A and server B (redundant server), the exchange is done via udp ( the port is defined by int udpServerPort) and the layer 5 transfers commands and status info as comma separated variables

IPAddress IPudpLFCPServerAf = {192,168,88,32}; // must be passed when sending messages / hearbeats 
IPAddress IPudpLFCPServerBf = {192,168,88,33}; // must be passed when sending messages / hearbeats 

int udpServerPort = 8443; // UDP/443 because, well, we can

const char *MyHostname = MY_NAME;

LFCPconf myLFCPconfA;
LFCPconf myLFCPconfB;




int  alarmFlag = 0;  // Note: alarmFlag is raised via LFCP and says we are in alarm conditions, alarmEnable is a configuration switch that says reactions to alarms are enabled.
int networkStateA = 0; // network ok
int networkStateB = 0; // network ok

void LFCPsendACK(LFCPconf *myLFCPconf) {
  strcpy(myLFCPconf->command, "ACK");
  strcpy(myLFCPconf->param1, "0");
  strcpy(myLFCPconf->param2, "0");
  strcpy(myLFCPconf->param3, "0");
  strcpy(myLFCPconf->param4, "0");
  strcpy(myLFCPconf->param5, "0");
  strcpy(myLFCPconf->param6, "0");
  Serial.println("LFCPsendACK called");
  LFCPcommand(myLFCPconf);  //LFCPconf myLFCPconf
}


// LFCP defines END


#include <WiFiMulti.h> // this should support more different AP's // ToDo
WiFiMulti wifiMulti; // this should support more different AP's // ToDo



// CAMERA: camera configuration and initialization

camera_config_t config;

uint8_t cameraInitType = 0; // 0 = camera off, not initialized; 1 = default init, the main webserver; 2 = motion detector; 3 = color blob detector




// WARNING!!! Make sure that you have either selected ESP32 Wrover Module,
//            or another board which has PSRAM enabled
// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"

// of course this is the default and should be modified


void cameraInitConfig() {
  

  
  // camera part
  cameraInitType = 1;
  
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  //init with high specs to pre-allocate larger buffers
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
    
// different camera module models

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
    }
  sensor_t * s = esp_camera_sensor_get();
  
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif
}


// MOTION DETECTION SPECIFIC CAMERA DEFINES and buffers

#define WIDTH 320
#define HEIGHT 240
#define BLOCK_SIZE 10
#define W (WIDTH / BLOCK_SIZE)
#define H (HEIGHT / BLOCK_SIZE)
#define BLOCK_DIFF_THRESHOLD 0.2
#define IMAGE_DIFF_THRESHOLD 0.1
#define DEBUG 1

bool capture_still(); // declare functions
bool motion_detect();
void update_frame();
void print_frame(uint16_t frame[H][W]);

uint16_t prev_frame[H][W] = { 0 };  // these are actually 32*24 * 16bit, more than acceptable, no problem.
uint16_t current_frame[H][W] = { 0 };
uint8_t motionDetectON = 0;  // motion detection is normally kept off. The idea is to activate it in case the command & control servers do not respond, or to activate it on demand / on alarm.
uint8_t alarmEnabled = ALARMS_ENABLED; // motion detection activation is enabled by default to 1, some other kinds of usage may require this is off, for example the color blob detection mode and the webcam / telegram modes. 
uint8_t motionDetectedFlag = 0;

// MOTION DETECTION SPECIFIC CAMERA CONFIG INIT FUNCTION (first, remember to do a deinit!!!)
void cameraInitConfigMotionDetect() {
  // camera part
  
  cameraInitType = 2;
  
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
// different camera module models
#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);
#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif
}

// CAMERA MOTION DETECTION SPECIFIC FUNCTIONS

// * Capture image and do down-sampling

bool capture_still() {
    camera_fb_t *frame_buffer = esp_camera_fb_get();
    if (!frame_buffer) {
        Serial.println("Motion detector: capture_still, no frame_buffer detected");
        cameraInitType = 0;
        return false;
    }
    Serial.print("GRAY len: ");
    Serial.println(frame_buffer != NULL ? frame_buffer->len : 0);

    if ((frame_buffer->len) != 76800) {
      cameraInitType = 0;
      return false;
    } else {
      Serial.println("Captured one GRAYSCALE frame");
      }
    // I believe it is the case to capture another five frames.
    delay(500);
    frame_buffer = esp_camera_fb_get();
    if ((frame_buffer->len) != 76800) {
      cameraInitType = 0;
      return false;
    } else {
      Serial.println("Captured another GRAYSCALE frame");
      }
    delay(500);
    frame_buffer = esp_camera_fb_get();
    if ((frame_buffer->len) != 76800) {
      cameraInitType = 0;
      return false;
    } else {
      Serial.println("Captured another GRAYSCALE frame");
      }
    delay(500);
    frame_buffer = esp_camera_fb_get();
    if ((frame_buffer->len) != 76800) {
      cameraInitType = 0;
      return false;
    } else {
      Serial.println("Captured another GRAYSCALE frame");
      }
    delay(500);
    frame_buffer = esp_camera_fb_get();
    if ((frame_buffer->len) != 76800) {
      cameraInitType = 0;
      return false;
    } else {
      Serial.println("Captured another GRAYSCALE frame");
      }
    delay(500);
    frame_buffer = esp_camera_fb_get();
    if ((frame_buffer->len) != 76800) {
      cameraInitType = 0;
      return false;
    } else {
      Serial.println("Captured another GRAYSCALE frame, the picture should now be stable");
      }

   
    // WARNING: I'm commenting this whole block as I believe it causing problems.
    /*
    
    //sensor_t *sensor = esp_camera_sensor_get();
    //Serial.print("Switch to JPEG: ");
    //Serial.println(sensor->set_pixformat(sensor, PIXFORMAT_RGB565));
    Serial.println("Motion detector: config PIXFORMAT_RGB565");
    config.pixel_format = PIXFORMAT_RGB565;
    Serial.print("Switch ok? ");
    Serial.println(esp_camera_init(&config) == ESP_OK ? "yes" : "no");
    Serial.println("Motion detector: esp_camera_fb_get() try");
    frame_buffer = esp_camera_fb_get();
    Serial.print("JPEG len: ");
    Serial.println(frame_buffer != NULL ? frame_buffer->len : 0);
    //sensor->set_pixformat(sensor, PIXFORMAT_GRAYSCALE);
    Serial.println("Motion detector: esp_camera_fb_get() second try");
    frame_buffer = esp_camera_fb_get();
    Serial.print("GRAY len: ");
    Serial.println(frame_buffer != NULL ? frame_buffer->len : 0);

    */
    
    
    Serial.println("Motion detector: working on the frame");
    //while (1) delay(50000);
    // set all 0s in current frame
    
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            current_frame[y][x] = 0;
    Serial.println("Motion detector: current_frame space prepared, now downsampling");
    // down-sample image in blocks
    for (uint32_t i = 0; i < WIDTH * HEIGHT; i++) {
        const uint16_t x = i % WIDTH;
        const uint16_t y = floor(i / WIDTH);
        const uint8_t block_x = floor(x / BLOCK_SIZE);
        const uint8_t block_y = floor(y / BLOCK_SIZE);
        const uint8_t pixel = frame_buffer->buf[i];
        const uint16_t current = current_frame[block_y][block_x];
        // average pixels in block (accumulate)
        current_frame[block_y][block_x] += pixel;
    }
    Serial.println("Motion detector: averaging");
    // average pixels in block (rescale)
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            current_frame[y][x] /= BLOCK_SIZE * BLOCK_SIZE;
#if DEBUG
    Serial.println("Current frame:");
    print_frame(current_frame);
    Serial.println("---------------");
#endif
    Serial.println("Motion detector: capture performed, return");
    return true;
}

// * Compute the number of different blocks
// * If there are enough, then motion happened
bool motion_detect() {
    uint16_t changes = 0;
    const uint16_t blocks = (WIDTH * HEIGHT) / (BLOCK_SIZE * BLOCK_SIZE);
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            float current = current_frame[y][x];
            float prev = prev_frame[y][x];
            float delta = abs(current - prev) / prev;
            if (delta >= BLOCK_DIFF_THRESHOLD) {
#if DEBUG
                Serial.print("diff\t");
                Serial.print(y);
                Serial.print('\t');
                Serial.println(x);
#endif
                changes += 1;
            }
        }
    }
    Serial.print("Changed ");
    Serial.print(changes);
    Serial.print(" out of ");
    Serial.println(blocks);
    return (1.0 * changes / blocks) > IMAGE_DIFF_THRESHOLD;
}

// * Copy current frame to previous

void update_frame() {
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            prev_frame[y][x] = current_frame[y][x];
        }
    }
}

// * For serial debugging
// * @param frame

void print_frame(uint16_t frame[H][W]) {
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            Serial.print(frame[y][x]);
            Serial.print('\t');
        }
        Serial.println();
    }
}

// CAMERA COLOR BLOB DETECTOR, INIT FUNCTION
// color-based blob detection - I plan to use this as part of a larger project, the idea is to help detect certain insects and pests in agricultural settings
// this camera will then become an additional sensor communicating with a robot designed to remove pests from invaded plants. 

void cameraInitConfigColorDetector() {  // specific for color-based target recognition, remember to do a deinit first!!!
  // camera part

  cameraInitType = 3;
  
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB888;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
// different camera module models
#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);
#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif
}

// Telegram section

#include "UniversalTelegramBotRZO.h"
#include <WiFiClientSecure.h>

char* TBotToken = "810218086:AAFPJNDbolU0tzj_Xx6lXOsatDlYnWYaa80";
char* TChatId = "537093554";
WiFiClientSecure client;
UniversalTelegramBot TBotCambot(TBotToken, client);

int Bot_mtbs = 3000; //mean time between scan messages
long Bot_lasttime;   //last time messages' scan has been done

// specific camera variables for the telegram bot

//camera_fb_t * TBotCamfb; // moved intra-function

uint8_t* TBotCamfb_buffer;
size_t TBotCamfb_length;
int TBotCamcurrentByte;

// camera functions specific for the telegram bot

bool isMoreDataAvailable() {
  return (TBotCamfb_length - TBotCamcurrentByte);
}
uint8_t photoNextByte() {
  TBotCamcurrentByte++;
  return (TBotCamfb_buffer[TBotCamcurrentByte - 1]);
}
void take_send_photo(String TChatId) {
  camera_fb_t * TBotCamfb = NULL;
  TBotCamfb = esp_camera_fb_get();
  TBotCamcurrentByte = 0;
  TBotCamfb_length = TBotCamfb->len;
  TBotCamfb_buffer = TBotCamfb->buf;
  TBotCambot.sendPhotoByBinary(TChatId, "image/jpeg", TBotCamfb->len, isMoreDataAvailable, photoNextByte, nullptr, nullptr);
  esp_camera_fb_return(TBotCamfb);
  TBotCamfb_length = NULL;
  TBotCamfb_buffer = NULL;
}

// UDP Async client, sends keep heartbeat messages to the server and retrieves commands

AsyncUDP udp; // for server A  // udp layer 5 control protocol
AsyncUDP udpB; // for server B



// udp - layer 5 control protocol processing, the layer 5 control protocol consists of 8 CSV fields. The protocol structure is simple: client_name,requested_operation,field1,field2,field3,field4,field5,field6  all unused fields are set to ascii character 0

//#define FIELDUA 8
//#define FIELDUB 64

//char UdataSplit[FIELDUA][FIELDUB];
//int UfieldsLen[FIELDUA];
//int UdataLen = 0;
int UserverOK = 0; // contact with the server attempted for 3 times in a row, failing. Say, the idea is: ACK from server received: set to 3, send HEARTBEAT: decrease by 1 until zero is reached, then stop decreasing.


// WIFI management specific variables 
static uint8_t wifiDebugOn = 0;
static uint8_t wifiConnFlag = 0;
static uint8_t wifiWebserverFlag = 0;
static uint8_t wifiBeginFlag = 0;
static uint8_t wifiConnectionAttempts = 0;

// Set the local access point IP address, the netmask is assumed to be /24

IPAddress SetAPIP = {192,168,5,1};  // REMEMBER TO CHANGE THIS FOR EVERY AP YOU PLAN TO INTRODUCE TO THE NETWORK

// Set these to your desired credentials.

const char *ssid = MY_NAME;  // for the local AP  // REMEMBER TO CHANGE THE SSID AS WELL, THE NET PART CONTAINS THE THIRD BYTE OF THE AP IP ADDRESS (the network LSByte)
const char *password = "giampippetto";

WiFiServer serverSTA(8080); // adding a second server on the STA interface? Let's try

// loop() control section

uint32_t loopCounter = 0;

// NETWORK section
// processing variables subsection
uint8_t payloadCurrContent[128] = {0};
uint8_t hashCurrContent[64] = {0};
uint8_t payHeaderCurrContent[32] = {0};

// NAt processing subsection
uint32_t payloadTypeF = 0;
uint32_t srcIPext = 0; // STA side
uint32_t dstIPext = 0;
uint32_t srcIPint = 0; // AP side
uint32_t dstIPint = 0;
uint32_t srcIPpay = 0; // payload side
uint32_t dstIPpay = 0;

// in the NAT table, the folloving are converted to uint32_t 
uint16_t srcPortext = 0; // STA side
uint16_t dstPortext = 0;
uint16_t srcPortint = 0; // AP side
uint16_t dstPortint = 0;
uint16_t srcPortpay = 0; // payload side
uint16_t dstPortpay = 0;
uint16_t expiryCounter = 0;

// NAT table subsection
uint32_t NATline[16] = {0};  // there are 14 fields actually, as per the above list, plus 2 reserved fields
uint32_t NATtable[256][16] = {0};
uint16_t NATlinePointer = 0; // do we allow up to 256 flows only?
// routing table subsection
uint16_t RoutelinePointer = 0; // do we allow up to 16 routes only?
uint32_t routetable[16][8] = {0};  // 16 routing entries with: destination net, netmask, source IP of the interface, gateway IP, interface ID, reserved, reserved, reserved
//packet buffer sub-section
uint8_t payloadToProcessFlag = 0; // if the LSB bit0 == 1, there is a payload that needs processing, other values / bits set may mean other operations.
uint8_t * payloadBuffer = NULL;
//structure of the packet buffer:
/*
Size = 224Bytes = 32B header + 128B payload + 64B hash
4B = Type Field( 4 bytes ) = reqtype(1B), protocolNum(1B), Reserved(1B), Reserved(1B)
4B = DST IP
4B = SRC IP 
2B = DST PORT
2B = SRC PORT
2B = PayloadLen
2B = Reserved
4B = Reserved
4B = Reserved
4B = Reserved
128B = actual payload
64B = sha256Hash_reserved(32B raw or 64B hex)
*/

// LED section

#define LED_BUILTIN 2   // Set the GPIO pin where you connected your test LED or comment this line out if your dev board has a built-in LED

// string section 
// converts character array 
// to string and returns it 

String convertToString(char* a, int size) {
    int i; 
    String s = ""; 
    for (i = 0; i < size; i++) { 
        s = s + a[i]; 
    } 
    return s; 
}

// deep sleep section

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR int bootCount = 0;

//Method to print the reason by which ESP32
//has been awaken from sleep

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason){
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

// update the routing table

void routeTableUpdate() {

// RoutelinePointer // do we allow up to 16 routes only?
//uint32_t *routetable[16][8] // 16 routing entries with: destination net, netmask, source IP of the interface, gateway IP, interface ID, reserved, reserved, reserved
uint32_t ImyLocalIP = 0;
uint32_t ImyLocalSubnet = 0;
uint32_t ImyAPIP = 0;
uint32_t ImyAPSubnet = 0;
uint32_t ImyLocalSubnetMask = 0;
uint32_t ImyAPSubnetMask = (255 >> 24) + (255 >> 16) + (255 >> 8);
uint32_t ImyLocalGateway = 0;
IPAddress myLocalIP = {0,0,0,0};
IPAddress myLocalSubnet = {0,0,0,0};
IPAddress myLocalSubnetMask = {0,0,0,0};
IPAddress myLocalGateway = {0,0,0,0};
IPAddress myAPIP = WiFi.softAPIP();
ImyAPIP = (uint32_t)((myAPIP[0] << 24) + (myAPIP[1] << 24) + (myAPIP[2] << 24) + myAPIP[3]);
ImyAPSubnet = ImyAPIP & ImyAPSubnetMask;
// the first lines
// line 0 local AP  // destination net, netmask, source IP of the interface, gateway IP, interface ID, reserved, reserved, reserved
routetable[0][0] = ImyAPSubnet;
routetable[0][1] = ImyAPSubnetMask;
routetable[0][2] = ImyAPIP;
routetable[0][3] = 0; // gateway of course set to 0 for the AP side
routetable[0][4] = 0; // interface ID set to zero, for the moment.
routetable[0][5] = 0;
routetable[0][6] = 0;
routetable[0][7] = 0;
if (wifiConnFlag == 1) {
  myLocalIP = WiFi.localIP();
  myLocalSubnet = WiFi.subnetMask();
  myLocalGateway = WiFi.gatewayIP();
  ImyLocalIP = (myLocalIP[0] << 24) + (myLocalIP[1] << 24) + (myLocalIP[2] << 24) + myLocalIP[3];
  ImyLocalSubnetMask = (myLocalSubnetMask[0] << 24) + (myLocalSubnetMask[1] << 24) + (myLocalSubnetMask[2] << 24) + myLocalSubnetMask[3];
  ImyLocalGateway = (myLocalGateway[0] << 24) + (myLocalGateway[1] << 24) + (myLocalGateway[2] << 24) + myLocalGateway[3];
  ImyLocalSubnet = ImyLocalIP & ImyLocalSubnetMask;
  // line 1 STA  // destination net, netmask, source IP of the interface, gateway IP, interface ID, reserved, reserved, reserved
  routetable[1][0] = ImyLocalSubnet;
  routetable[1][1] = ImyLocalSubnetMask;
  routetable[1][2] = ImyLocalIP;
  routetable[1][3] = ImyLocalGateway;
  routetable[1][4] = 1; // interface ID set to 1, for the moment.
  routetable[1][5] = 0;
  routetable[1][6] = 0;
  routetable[1][7] = 0; 
} else {
  routetable[1][0] = 0; // nullifying the STA route
  routetable[1][1] = 0; // nullifying the STA route
  routetable[1][2] = 0; // nullifying the STA route
  routetable[1][3] = 0; // nullifying the STA route
  routetable[1][4] = 0; // nullifying the STA route
  routetable[1][5] = 0; // nullifying the STA route
  routetable[1][6] = 0; // nullifying the STA route
  routetable[1][7] = 0; // nullifying the STA route
}

}  // end void routetableupdate()


// handle webserver <- client requests
void clientServe() {
  WiFiClient client = serverSTA.available();   // listen for incoming clients
  if (client) {                             // if you get a client,
    Serial.println("New Client.");           // print a message out the serial port
    strcpy(myLFCPconfA.command, "LOG");
    strcpy(myLFCPconfA.param1, "management_server_request_via_http");
    LFCPcommand(&myLFCPconfA);  //LFCPconf myLFCPconf
    strcpy(myLFCPconfB.command, "LOG");
    strcpy(myLFCPconfB.param1, "management_server_request_via_http");
    LFCPcommand(&myLFCPconfB);  //LFCPconf myLFCPconf
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/H\">here</a> to take a picture and send it via telegram (configure via udp layer 5 control protocol).<br>");
            client.print("Click <a href=\"/L\">here</a> to restart the device.<br>");
            client.print("Click <a href=\"/W\">here</a> to initialize the camera hardware for the integrated camera webserver.<br>");
            client.print("Click <a href=\"/S\">here</a> to deinitialize the camera hardware and save power.<br>");
            client.print("Click <a href=\"/A\">here</a> to enable the motion alarm.<br>");
            client.print("Click <a href=\"/D\">here</a> to disable the motion alarm.<br>");
            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H")) {
          //digitalWrite(LED_BUILTIN, HIGH);               // GET /H takes a picture and sends it via telegram
          if ((cameraInitType != 1) || (cameraInitType == 0)) {
              if (cameraInitType != 0) {
                esp_camera_deinit();  // the camera needs a deinit from its current configuration.
                delay(1000); // this might not really be necessary
              }
              cameraInitConfig(); // and is setup for the correct motion capture configuration
              delay(3000);
          } 
          take_send_photo(TChatId);
          digitalWrite(LED_BUILTIN, LOW);
          strcpy(myLFCPconfA.command, "LOG");
          strcpy(myLFCPconfA.param1, "telegram_photo_request_via_http");
          LFCPcommand(&myLFCPconfA);  //LFCPconf myLFCPconf
          strcpy(myLFCPconfB.command, "LOG");
          strcpy(myLFCPconfB.param1, "telegram_photo_request_via_http");
          LFCPcommand(&myLFCPconfB);  //LFCPconf myLFCPconf
        }
        if (currentLine.endsWith("GET /W")) {  // init camera for the main camera webserver
          if ((cameraInitType != 1) || (cameraInitType == 0)) {
              if (cameraInitType != 0) {
                esp_camera_deinit();  // the camera needs a deinit from its current configuration.
                delay(1000); // this might not really be necessary
              }
              cameraInitConfig(); // and is setup for the correct motion capture configuration
              delay(3000);
              strcpy(myLFCPconfA.command, "LOG");
          strcpy(myLFCPconfA.param1, "camera_init_request_via_http");
          LFCPcommand(&myLFCPconfA);  //LFCPconf myLFCPconf
          strcpy(myLFCPconfB.command, "LOG");
          strcpy(myLFCPconfB.param1, "camera_init_request_via_http");
          LFCPcommand(&myLFCPconfB);  //LFCPconf myLFCPconf
          } 
        }
        if (currentLine.endsWith("GET /S")) {  // deinitialize the camera
          cameraInitType = 0;
          esp_camera_deinit();  // the camera needs a deinit from its current configuration.
          strcpy(myLFCPconfA.command, "LOG");
          strcpy(myLFCPconfA.param1, "camera_deinit_request_via_http");
          LFCPcommand(&myLFCPconfA);  //LFCPconf myLFCPconf
          strcpy(myLFCPconfB.command, "LOG");
          strcpy(myLFCPconfB.param1, "camera_deinit_request_via_http");
          LFCPcommand(&myLFCPconfB);  //LFCPconf myLFCPconf
        }
        if (currentLine.endsWith("GET /A")) {  // alarm enable
          alarmEnabled = 1;
          strcpy(myLFCPconfA.command, "LOG");
          strcpy(myLFCPconfA.param1, "alarm_enabled_request_via_http");
          LFCPcommand(&myLFCPconfA);  //LFCPconf myLFCPconf
          strcpy(myLFCPconfB.command, "LOG");
          strcpy(myLFCPconfB.param1, "alarm_enabled_request_via_http");
          LFCPcommand(&myLFCPconfB);  //LFCPconf myLFCPconf
        }
        if (currentLine.endsWith("GET /D")) {  // alarm disable
          alarmEnabled = 0;
          strcpy(myLFCPconfA.command, "LOG");
          strcpy(myLFCPconfA.param1, "alarm_disabled_request_via_http");
          LFCPcommand(&myLFCPconfA);  //LFCPconf myLFCPconf
          strcpy(myLFCPconfB.command, "LOG");
          strcpy(myLFCPconfB.param1, "alarm_disabled_request_via_http");
          LFCPcommand(&myLFCPconfB);  //LFCPconf myLFCPconf
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(LED_BUILTIN, LOW);                // GET /L restarts the device
          strcpy(myLFCPconfA.command, "LOG");
          strcpy(myLFCPconfA.param1, "device_restart_request_via_http");
          LFCPcommand(&myLFCPconfA);  //LFCPconf myLFCPconf
          strcpy(myLFCPconfB.command, "LOG");
          strcpy(myLFCPconfB.param1, "device_restart_request_via_http");
          LFCPcommand(&myLFCPconfB);  //LFCPconf myLFCPconf
          esp_restart();
          
        }
      }
    }
  } // end if(client)
    // close the connection:
    client.stop();
    //Serial.println("Client Disconnected.");
} // end clientServe()

void startCameraServer();

void wifiIntManagement() {
    // STA connection and disconnection management
// WIFI DEBUG AND SERVICING SECTION START
  if (wifiDebugOn == 1) {
    if (WiFi.status() != WL_CONNECTED) { // wifiMulti.run() == WL_CONNECTED // WiFi.status() == WL_CONNECTED
    
      Serial.println("WiFi not connected yet");
    }
    if (WiFi.status() == WL_CONNECTED) { // wifiMulti.run() == WL_CONNECTED // WiFi.status() == WL_CONNECTED
      Serial.println("Starting Camera Server");
      startCameraServer();  // aaaand, we should really do something about this in case of disconnection.
      Serial.println("Starting Service Server");
      serverSTA.begin();
      Serial.println("Server started");
      //udp.connect(IPAddress(IPudpServerAf, udpServerPort);  // this should now be done by the LFCP library!!! please check! WARNING!
      //Serial.println("REMOTE Server A connect");
      //udpB.connect(IPAddress(IPudpServerBf, udpServerPort);
      //Serial.println("REMOTE Server b connect");
    
      Serial.print("WiFi up and running: ");
      Serial.print(WiFi.localIP());
      Serial.println("' to connect");
    }
  }
  if (wifiConnectionAttempts >= 32) { // turning off wifi, there is no point in trying anymore, if past 32 attempts.
    WiFi.mode(WIFI_OFF); // turning off wifi, then we go to deep sleep for a certain number of seconds.
    
    esp_err_t err = esp_camera_deinit();
    esp_deep_sleep_start();  // going to deep sleep
  }
  if ((wifiConnFlag == 0) && (wifiConnectionAttempts < 32)) {  // wifiConnectionAttempts must be < 32 
    wifiConnectionAttempts++; // increasing the connection attempts counter
    
    if (wifiMulti.run() == WL_CONNECTED) { // wifiMulti.run() == WL_CONNECTED // WiFi.status() == WL_CONNECTED
      Serial.println("Starting Camera Server");
      startCameraServer();  // aaaand, we should really do something about this in case of disconnection.
      Serial.println("Starting Service Server");
      serverSTA.begin();
      Serial.println("Server started");
      //udp.connect(IPudpLFCPServerAf, udpServerPort);  // this should now be done by the LFCP library!!! please check! WARNING!
      //Serial.println("REMOTE Server A connect");
      //udpB.connect(IPAddress(IPudpLFCPServerBf, udpServerPort);
      //Serial.println("REMOTE Server b connect");
      
      Serial.print("WiFi up and running: ");
      Serial.print(WiFi.localIP());
      Serial.println("' to connect");
      Serial2.print("WiFi up and running: ");
      Serial2.print(WiFi.localIP());
      Serial2.println("' to connect");
      wifiConnFlag = 1;
      wifiConnectionAttempts = 0; // reset the connection attempts
    }
    if ((WiFi.status() != WL_CONNECTED) && (wifiBeginFlag == 0)) {
    
      Serial.println("WiFi restarting");
      WiFi.begin(ssid, password); 
      
      wifiBeginFlag = 1;
    }
  }
  if (wifiConnFlag == 1) { // test if the wifi stays alive...
    if (WiFi.status() != WL_CONNECTED) {  // wifiMulti.run() == WL_CONNECTED // WiFi.status() != WL_CONNECTED
    
      Serial.println("WiFi disconnected");
      wifiConnFlag = 0;
      wifiBeginFlag = 0;
      wifiConnectionAttempts = 0; // reset the connection attempts
    }
  }
  if (((wifiConnFlag == 1) && (wifiWebserverFlag == 0)) || (ESP.getFreeHeap() < 44000)) { // servicing the webserver on connection up for the first time
      // >>>>>>>> START SERVER SERVICING HERE (close and open the server in case of disconnects / reconnects)
      // >>>>>>>> END SERVER SERVICING HERE (close and open the server in case of disconnects / reconnects)
      
      wifiWebserverFlag = 1;
    
  }  
} // WIFI DEBUG AND SERVICING SECTION END 

/*
// UDP - layer 5 control protocol
void UresetudpData() {
for(int i = 0; i < FIELDUA; i++){
  for(int j = 0; j < FIELDUB; j++) {
    UdataSplit[i][j] = 0;
  }
}
for(int i = 0; i < FIELDUA; i++) {
  UfieldsLen[i] = 0;
}
UdataLen = 0;
} // end resetudpData()

int UfillinudpData(uint8_t *pktDataField, int pktLen) {
int i = 0;
int j = 0;
int k = 0;
UdataLen = pktLen;
for(i = 0; i < UdataLen; i++){
  if(pktDataField[i] == ',') {
    UfieldsLen[j] = k;
    Serial.print("received_udp_control_protocol_data:");
    Serial.print(UdataSplit[j]);
    Serial.print("; data_len:");
    Serial.print(k);
    Serial.print("; field_position:");
    Serial.println(j);
    j++;
    k = 0;
  }
  if(j >= FIELDUA) {
    Serial.print("ERROR: too many commas in the udp layer 5 control protocol");
    return 100;
  }  
  UdataSplit[j][k] = pktDataField[i];
  k++;
} 
UfieldsLen[j] = k;
return 0;
} // end fillinudpData()
*/




void LFCPcommandsExecutor(LFCPconf *myLFCPconf) {



        

      if(strcmp(myLFCPconf->command, "ACK") == 0) {
        // do not reply with an ACK to an ACK, that would be ...disturbing
        
      }
      if(strcmp(myLFCPconf->command, "LOG") == 0) {
        // do not reply with an ACK to an ACK, that would be ...disturbing
        Serial.print("received log message from the server: ");
        Serial.println(myLFCPconf->param1);
      }
      if(strcmp(myLFCPconf->command, "TLGS") == 0) {  // send picture via telegram
        //TBotToken
        //TChatId
        // technically speaking we should deinit and reinit with acceptable parameters. 
        
        if ((cameraInitType != 1) || (cameraInitType == 0)) {
          if (cameraInitType != 0) {
            esp_camera_deinit();  // the camera needs a deinit from its current configuration.
          }
          delay(1000); // this might not really be necessary
          cameraInitConfig(); // and is setup for the correct motion capture configuration
          delay(3000);
        }
         // this might not really be necessary
        take_send_photo(TChatId);   

        LFCPsendACK(myLFCPconf);

      }
      if(strcmp(myLFCPconf->command, "TLGC") == 0) { // configure telegram bot token and chat id
        bool ok; 
        String sToken = convertToString(myLFCPconf->param1, strlen(myLFCPconf->param1)); 
        String sChatId = convertToString(myLFCPconf->param2, strlen(myLFCPconf->param2)); 
        ok = NVS.setString ("TLGToken", sToken); // Store the data value into the key named "TLGToken" on the NVS
        if(ok) {
          ok = NVS.setString ("TLGChatId", sChatId); // Store the data value into the key named "TLGChatId" on the NVS
          
        }
        if(ok) {
        // I'M A CLIENT, SO I REPLY ACK BY DEFAULT WHENEVER I RECEIVE A COMMAND
        //reply to the client
        //packet.printf(ACK_DATA, packet.length());
        LFCPsendACK(myLFCPconf);

        
        } else {
          Serial.println("ERROR saving data to nvs");
          strcpy(myLFCPconf->command, "ERR");
          strcpy(myLFCPconf->param1, "Failed to save telegram data to NVS");
          LFCPcommand(myLFCPconf);  //LFCPconf myLFCPconf 

        }
        
        
      }
      if(strcmp(myLFCPconf->command, "RST") == 0) {  // restart system
        // I'M A CLIENT, SO I REPLY ACK BY DEFAULT WHENEVER I RECEIVE A COMMAND
        //reply to the client
        //packet.printf(ACK_DATA, packet.length());
        LFCPsendACK(myLFCPconf);
        esp_restart();
      }
      if(strcmp(myLFCPconf->command, "DSLP") == 0) { // start deep sleep
        // I'M A CLIENT, SO I REPLY ACK BY DEFAULT WHENEVER I RECEIVE A COMMAND
        //reply to the client
        //packet.printf(ACK_DATA, packet.length());
        LFCPsendACK(myLFCPconf);

        strcpy(myLFCPconf->command, "LOG");
        strcpy(myLFCPconf->param1, "device_going_to_deep_sleep");
        LFCPcommand(myLFCPconf);  //LFCPconf myLFCPconf 
        
        Serial.println("RECEIVED REQUEST TO DEEP SLEEP, STARTING TO SLEEP DEEP");
        esp_err_t err = esp_camera_deinit();
        esp_deep_sleep_start();  // going to deep sleep
      }
      if(strcmp(myLFCPconf->command, "WCAM") == 0) { // init camera for the webserver
        if ((cameraInitType != 1) || (cameraInitType == 0)) {
              if (cameraInitType != 0) {
                esp_camera_deinit();  // the camera needs a deinit from its current configuration.
                delay(1000); // this might not really be necessary
              }
              cameraInitConfig(); // and is setup for the correct motion capture configuration
              delay(3000);
        }
        LFCPsendACK(myLFCPconf);

      }
      if(strcmp(myLFCPconf->command, "SCAM") == 0) { // deinitialize the camera
        cameraInitType = 0;
        esp_camera_deinit();  // the camera needs a deinit from its current configuration.
        LFCPsendACK(myLFCPconf);

      }
      if(strcmp(myLFCPconf->command, "ENAL") == 0) { // enable alarms
        alarmEnabled = 1;
        LFCPsendACK(myLFCPconf);
      }
      if(strcmp(myLFCPconf->command, "DSAL") == 0) { // disable alarms
        alarmEnabled = 0;
        LFCPsendACK(myLFCPconf);
      }

}



    

    
void clientsManagement() {
  if ((wifiConnFlag == 1) && (wifiWebserverFlag == 1)) { // HANDLE CLIENT REQUESTS HERE
    
    // >>>>>>>> START HANDLE CLIENT REQUESTS HERE
    
    clientServe();
    
    // >>>>>>>> START HANDLE CLIENT REQUESTS HERE
    
  } // end handle client requests if
} // END clientsMAnagement()






void setup() {


  // essential Serial Setup

  
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // LFCP vars init START
  strcpy(myLFCPconfA.myhostname, MyHostname);
  myLFCPconfA.port = udpServerPort;
  myLFCPconfA.server = IPudpLFCPServerAf;
  myLFCPconfA.currentudphandle = &LFCPudp;
  myLFCPconfA.alarmEnable = 1;
  myLFCPconfA.commandsInternalExecEnable = 0;
  strcpy(myLFCPconfA.param1, "0");
  strcpy(myLFCPconfA.param2, "0");
  strcpy(myLFCPconfA.param3, "0");
  strcpy(myLFCPconfA.param4, "0");
  strcpy(myLFCPconfA.param5, "0");
  strcpy(myLFCPconfA.param6, "0");

  strcpy(myLFCPconfB.myhostname, MyHostname);
  myLFCPconfB.port = udpServerPort;
  myLFCPconfB.server = IPudpLFCPServerBf;
  myLFCPconfB.currentudphandle = &LFCPudpB;
  myLFCPconfB.alarmEnable = ALARMS_ENABLED;
  myLFCPconfB.commandsInternalExecEnable = 0;
  strcpy(myLFCPconfB.param1, "0");
  strcpy(myLFCPconfB.param2, "0");
  strcpy(myLFCPconfB.param3, "0");
  strcpy(myLFCPconfB.param4, "0");
  strcpy(myLFCPconfB.param5, "0");
  strcpy(myLFCPconfB.param6, "0");

  Serial.print("Setup: LFCP server A configured with IP: ");
  Serial.println(myLFCPconfA.server);
  Serial.print("Setup: LFCP server B configured with IP: ");
  Serial.println(myLFCPconfB.server);
  
  // LFCP vars init END


  
// wakeup and init part 
  // packet / payload buffer allocation
  payloadBuffer = (uint8_t *)malloc(256 * sizeof(uint8_t));  // the actual size is 224 (see in the declarations), the maximum payload size for ESP-NOW is 250.
  pinMode(LED_BUILTIN, OUTPUT);
  // deep sleep boots
  ++bootCount;
  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  /*
  First we configure the wake up source
  We set our ESP32 to wake up every 5 seconds
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
// camera init and config
  //Serial.println("WARNING, NOT INITIALIZING THE CAMERA IN ORDER TO SAVE POWER. PLEASE DO IT MANUALLY VIA UDP COMMANDS OR VIA THE MANAGEMENT HTTP SERVER ON PORT 8080");
  cameraInitConfig(); // we would liket the camera not initialized by default. This saver power.
  /*
 if ((cameraInitType != 1) || (cameraInitType == 0)) {
              if (cameraInitType != 0) {
                esp_camera_deinit();  // the camera needs a deinit from its current configuration.
                delay(1000); // this might not really be necessary
              }
              
              cameraInitConfig(); // and is setup for the correct motion capture configuration
              delay(3000);
            }
  */
// WIFI SECTION
  Serial.println();
  Serial.println("Configuring access point...");
  WiFi.mode(WIFI_AP_STA); // added by me // selecting the AP+STA mode
  wifiMulti.addAP("e32ap001s01border", "giampippetto");  // the connection part is verified as follows: wifiMulti.run() == WL_CONNECTED
  wifiMulti.addAP("G5SE_7499", "pippopluto");
  wifiMulti.addAP("internet-gw.iw1gpe.it.ampr.o_EXT", "heisakuukanwohiraite");
  wifiMulti.addAP("internet-gw.iw1gpe.it.ampr.o", "heisakuukanwohiraite");
  wifiMulti.addAP("inverse", "heisakuukanwohiraite");
  wifiMulti.addAP("e32ap002s01border", "giampippetto");
  wifiMulti.addAP("e32ap003s01border", "giampippetto");
  wifiMulti.addAP("e32ap004s01border", "giampippetto");
  wifiMulti.addAP("e32ap001s02border", "giampippetto");
  wifiMulti.addAP("e32ap002s02border", "giampippetto");
  wifiMulti.addAP("e32ap003s02border", "giampippetto");
  wifiMulti.addAP("e32ap004s02border", "giampippetto");
  // You can remove the password parameter if you want the AP to be open.
  WiFi.softAP(ssid, password);
  delay(200); // wait at least 100ms until the AP has started
  if(!WiFi.softAPConfig(SetAPIP, SetAPIP, IPAddress(255, 255, 255, 0))){
      Serial.println("AP Config Failed");
  }
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  // the STA part is configured here and managed in loop()
// old wifi section
  //WiFi.begin(ssid, password);
  /*
  if (WiFi.status() != WL_CONNECTED) {
    for (int conncounter = 0; conncounter < 256; conncounter++) {
      delay(1000);
      Serial.print("try #");
      Serial.print(conncounter);
      Serial.println(" - STA not connected yet after 1000ms, please check your upstream access point");
      if (WiFi.status() == WL_CONNECTED) {
        Serial.print("STA connected");
        break;
      }
    }
  }
  //Serial.println("");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("starting Camera Server");
    startCameraServer();
  }
  Serial.print("Camera Server Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
  */
  // NVS storage init
  NVS.begin();
  // trying to load the bot configuration from NVS 
  //TBotToken
  //TChatId
  // load the telegram bot token if available from NVS
  String loadedstr = NVS.getString ("TLGToken"); // Read the value of the "TLGToken" key from the NVS 
  if(loadedstr.length() > 3) { // at least we make sure the result is not empty...
    TBotToken = {};
    loadedstr.toCharArray(TBotToken, loadedstr.length()); // loading TBotToken
    loadedstr = NVS.getString ("TLGChatId"); // Read the value of the "TLGChatId" key from the NVS 
    if(loadedstr.length() > 3) {
      loadedstr.toCharArray(TChatId, loadedstr.length()); // loading TChatId
      Serial.print("Loaded Telegram BotToken and ChatId from NVS storage with values: ");
      Serial.print(TBotToken);
      Serial.print(" and ");
      Serial.println(TChatId);
    }
  }
} // END setup()
void loop() {
  // put your main code here, to run repeatedly:
 // loopCounter check
 
  if (loopCounter > 16) {
    loopCounter = 0;
  }
  
  clientsManagement();  // web client requests are handled always 
  
  if(loopCounter == 1) {
    wifiIntManagement();
  }
  
  if(loopCounter == 2) {
    routeTableUpdate();
  }
  
  if(loopCounter == 3) {
    strcpy(myLFCPconfA.command, "HEARTBEAT");
    Serial.print("sending heartbeat to server A: ");
    Serial.println(myLFCPconfA.server);
    networkStateA = LFCPcommand(&myLFCPconfA);  //LFCPconf myLFCPconfA // returns with any received commands incorporated
    // executing any received commands
    if (networkStateA == LFCP_OK) {
      alarmEnabled = myLFCPconfA.alarmEnable;
      LFCPcommandsExecutor(&myLFCPconfA);
    }
    strcpy(myLFCPconfB.command, "HEARTBEAT");
    Serial.print("sending heartbeat to server B: ");
    Serial.println(myLFCPconfB.server);
    networkStateB = LFCPcommand(&myLFCPconfB);  //LFCPconf myLFCPconfB // returns with any received commands incorporated
    // executing any received commands
    if (networkStateB == LFCP_OK) {
      alarmEnabled = myLFCPconfB.alarmEnable;
      LFCPcommandsExecutor(&myLFCPconfB);
    }
    udp.broadcastTo(MY_NAME, udpServerPort);
  }
  
  if(loopCounter == 5) {
    if ( motionDetectedFlag == 1 ) {
      motionDetectedFlag = 0;
      //udp.print(MOTION_DETECTED);// we should send a log message instead
      strcpy(myLFCPconfA.command, "LOG");
      strcpy(myLFCPconfA.param1, "CAMERA_MOTION_DETECTED");
      networkStateA = LFCPcommand(&myLFCPconfA);  //LFCPconf myLFCPconfB 
      strcpy(myLFCPconfB.command, "LOG");
      strcpy(myLFCPconfB.param1, "CAMERA_MOTION_DETECTED");
      networkStateB = LFCPcommand(&myLFCPconfB);  //LFCPconf myLFCPconfB
    }
  }
  
  // position number 15, let the device stabilize its connections first...
  if(loopCounter == 15) { // MOTION DETECTOR SECTION
    // the default behaviour is to autoactivate motion detection only if both LFCP servers are gone. However I expect the whole system to detect if one server stops responding, and this should raise an alarm and activate motion detection
    if ((motionDetectON == 1) || ( ((networkStateA == LFCP_SERVUNREACH) && (networkStateB == LFCP_SERVUNREACH)) && (alarmEnabled == 1)) ){  
      if ((cameraInitType != 2) || (cameraInitType == 0)) {

              esp_camera_deinit();  // the camera needs a deinit from its current configuration.
              delay(1000); // this might not really be necessary

              Serial.println("Motion detector: camera needs a specific init (was either off or seto for something different)");
              cameraInitConfigMotionDetect(); // and is setup for the correct motion capture configuration
              delay(3000);
              Serial.println("Motion detector: camera init done");
            }
      if (!capture_still()) {
        Serial.println("Motion detector: Failed capture, asking for a re-init");
        cameraInitType = 0;
        delay(1000);
        return;
      }
      if (motion_detect()) {
        motionDetectedFlag = 1;
        Serial.println("Motion detected");
    }
    update_frame();
    }
  }
  // loopCounter increase
  loopCounter++;
  delay(1000);
}
