
#include <Arduino.h>
//#include <WiFi.h>
//#include <WiFiClient.h>
//#include <WiFiAP.h>
#include <WiFiUdp.h>
#include "AsyncUDP.h"
#include "lwip/api.h"

#include "ArduinoNvs.h"

//#include <WiFiMulti.h>

/*

// Please include in your code at least these libraries:
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>
#include "AsyncUDP.h"
#include "lwip/api.h"
#include "LFCP.h"

// return codes;

#define LFCP_OK 0
#define LFCP_UDPFAIL -1
#define LFCP_SERVUNREACH -2


// in order to send messages and commands to a server one needs to declare and initialize at least one configuration struct:

LFCPconf myServerConfig;

myServerConfig.server = {192,168,88,32}; // example destination server IP address // the .port in internally initialized to 8443 or may be initialized explicitly

myServerConfig.command = "HEARTBEAT"; // that's it, in order to sead out a heartbeat message the configuration may end up here. 



// ,.. and pass this as a parameter to the general LFCP message function.
// since the generic command may use up to 6 of the 8 fields that are part of the payload, you really need to do dive into the protocol details, described here below. 
// don't worry, the protocol is actually very simple and easy to use. 


// as an alternative, the heartbeat messages and log messages function are also available seprately and directly expose certain configuration parameters:

int LFCPHeartbeatClient(IPAddress IPLFCPServer, int LFCPport, char* LFCPhostname, AsyncUDP* udpHandle, int* LFCPalarmEnabled);

int LFCPLogClient(IPAddress IPLFCPServer, int LFCPport, char *LFCPhostname, AsyncUDP* udpHandle, char* LFCPLogMessage);


// ...so if you only need heartbeats and logs:
// please declare in your code at least these variables, the name you give them is irrelevant:

AsyncUDP LFCPudp; //  udp "handle" for server A , must be passed when sending messages / hearbeats // udp layer 5 control protocol
AsyncUDP LFCPudpB; // udp "handle"  for server B , must be passed when sending messages / hearbeats 

// IP addresses for LFCP server A and server B (redundant server)
IPAddress IPudpLFCPServerAf = {192,168,88,32}; // must be passed when sending messages / hearbeats 
IPAddress IPudpLFCPServerBf = {192,168,88,33}; // must be passed when sending messages / hearbeats 

int udpServerPort = 8443; // UDP/8443 remote server port, arbitrarily chosen, the exchange is done via udp ( the port is defined by int udpServerPort) and the layer 5 transfers commands or status info as comma separated variables  // must be passed when sending messages / hearbeats 

// a host name must be declared, it can be the SSID of the local Soft AP, and must be passed as one of the parameters when sending messages
const char *MyHostname = MY_NAME; // My hostname, of course the variable name is irrelevant as long as it is passed as parameter to the function calls. 

// this flag must be passed as a pointer ( &alarmFlag), the function LFCPHeartbeatClient must be able to modify it
int  alarmFlag;  

// the name LFCPnetworkOK is reserved.




Protocol description

This is a signaling / command protcol implemented at the layer 5 of the ISO/OSI stack, the trasport is via UDP segments. 
In order to ensure that the protocol can traverse NAT barriers and clients sitting behind a NAT can receive commands the following conditions apply:
- there must be at least one public reachable server, or at least 1 server that can be reached by all of the clients (so, sitting on the top tier of the network).
- the clients are expected to periodically send "heartbeats" to the server, while log messages or other messages are optional
- the server is expected to always send back an answer to a client messages. The answer can be either an ACK, or a pending command.
- pending commands for the clients are stored in a queue by the server and subsequently sent to the client in the form of replies to client messages. 
- a message exchange is always initiated by the client and consists of heartbeats and other messages. Other messages require an ACK as reply, heartbeats require replies that can either be an ACK or a command, commands do require an ACK reply (this makes the exchange final) from the client. ACK messages are final and it is forbidden to reply to an ACK in any form. Thus, any exchange ends with two packets normally (message -> ACK), and a maximum of 3 packets in the case of heartbeat -> command -> ACK.
- a command that in any way fails to be executed must be replied with an ACK to confirm reception, subsequently the client can send out an ERR message detailing what actually failed.

LFCP Layer Five Control protocol definition:
UDP payload: max len = 128 bytes, content: text string of 8 (eigth) comma-separated variables (CSV string). 
Each variable can be up to 64bytes long, but the overall CSV string length is limited to 128 bytes max including one NULL terminator. 
The CSV string has 8 fields for 8 text variables, 7 commas used as separators and a NULL terminator. 
The definition of "text" in this instance is "ASCII characters", so the char* array will be parsed as such (each character is uint8_t). 
In case Unicode characters are required, this code will require modifications. 

Protocol specifications:

UDP_payload: char* array = "field_0,field_1,field_2,field_3,field_4,field_5,field_6,field_7"

field_0 = sender_hostname   // This field always names the client, NOT THE SERVER. The server never puts its name into this field, as the server never initiates a transaction as server.
field_1 = request_type
field_2 = optional_message | data_field_1 | '0'
field_3 = data_field_2 | '0'
field_4 = unassigned | '0'
field_5 = unassigned | '0'
field_6 = unassigned | '0'
field_7 = unassigned | '0'


known field_2 values:

HEARTBEAT // heartbeat
ACK // on receive, set a flag to show the ACK has been received and the network is ok
ERR // field_2 = error details (do not use commas!)
LOG // field_2 = log details (do no use commas!)
MOAL // motion detected, field_2 = "esp32_cam_motion_detected"
TLGS // send picture via telegram
TLGC // configure telegram bot token and chat id, field_2 = bot_token,field_3=chat_id
RST // restart system
DSLP // start deep sleep
WCAM // init or re-init camera, color with jpeg out
SCAM // deinitialize the camera
ENAL // enable alarms
DSAL // disable alarms



*/

// LFCP defines

// return codes;

#define LFCP_OK 0
#define LFCP_UDPFAIL -1
#define LFCP_SERVUNREACH -2
#define LFCP_SERVUNREACHANDALARMON -3

// LFCP defines END



// data structure


struct LFCPconfS{
  IPAddress server = {192,168,88,32};
  int port = 8443;
  AsyncUDP* currentudphandle;
  int alarmEnable = 1;
  int commandsInternalExecEnable = 0; // disabled by default, some of the commands received by the client may be executed internally by the library relieving the program from the task.
  char myhostname[64] = {0}; // field number 0  // This field ALWAYS names the CLIENT, NOT THE SERVER.
  char command[64] = {0}; // field number 1
  char param1[64] = {0}; // field number 2, to stay on the safe side, the parameter is initialized with a harmless value. 
  char param2[64] = {0}; // field number 3
  char param3[64] = {0}; // field number 4
  char param4[64] = {0}; // field number 5
  char param5[64] = {0}; // field number 6
  char param6[64] = {0}; // field number 7
};

typedef LFCPconfS LFCPconf;


// public functions
      
//int LFCPHeartbeatClient(IPAddress IPLFCPServer, int LFCPport, char* LFCPhostname, AsyncUDP* udpHandle);


//int LFCPLogClient(IPAddress IPLFCPServer, int LFCPport, char *LFCPhostname, AsyncUDP* udpHandle, char* LFCPLogMessage);


// in general usage, one may assume that depending on the type of command the LFCPconf struct contents may change and can contain reply data from the server.
// some of this data, especially .command may point to commands received from the server, and in such instance other .paramN fields may be populated.
int LFCPcommand(LFCPconf *LFCPdatastruct);
// it's up to the user program to execute the commands received. The library provides the execution on a subset of commands that can be performed internally (for example: reboot the device...)

// list of commands that can be executed internally:
