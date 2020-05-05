#include <Arduino.h>
//#include <WiFi.h>
//#include <WiFiClient.h>
//#include <WiFiAP.h>
#include <WiFiUdp.h>
#include "AsyncUDP.h"
#include "lwip/api.h"

#include "ArduinoNvs.h"

//#include <WiFiMulti.h>

#include "LFCP.h"

// LFCP defines START
#define LFCP_FIVEZEROES_TAIL ",0,0,0,0,0" // udp - layer 5 control protocol predefined messages
#define LFCP_HEARTBEAT_TAIL ",HEARTBEAT,0,0,0,0,0,0" // udp - layer 5 control protocol predefined messages
#define LFCP_ACK_TAIL ",ACK,0,0,0,0,0,0" // udp - layer 5 control protocol predefined messages
#define LFCP_ERRGENERIC_TAIL ",ERR,0,0,0,0,0,0" // udp - layer 5 control protocol predefined messages

#define LFCP_ERRNVS_TAIL ",ERR,ERROR saving data to NVS,0,0,0,0,0" // udp - layer 5 control protocol predefined messages
#define LFCP_MOTION_DETECTED_TAIL ",MOAL,esp32_cam_motion_detected,0,0,0,0,0" // udp - layer 5 control protocol predefined messages

// udp - layer 5 control protocol processing, the layer 5 control protocol consists of 8 CSV fields. The protocol structure is simple: client_name,requested_operation,field1,field2,field3,field4,field5,field6  all unused fields are set to ascii character 0

#define FIELDUA 8  // how many fields in the payload CSV data
#define FIELDUB 64 // maximum length of each field in bytes

// return codes;

#define LFCP_OK 0
#define LFCP_UDPFAIL -1
#define LFCP_SERVUNREACH -2
#define LFCP_SERVUNREACHANDALARMON -3    // so far unused

// LFCP defines END


// UDP Async client, sends keep heartbeat messages to the server and retrieves commands

// CSV payload processing array
char LFCPdataSplit[FIELDUA][FIELDUB];
int UfieldsLen[FIELDUA];

int LFCPdataLen = 0;

static int LFCPnetworkOK = 0; // contact with the server attempted for 3 times in a row, failing. Say, the idea is: ACK from server received: set to 3, send HEARTBEAT: decrease by 1 until zero is reached, then stop decreasing.
 
int LFCPalarmEnabled = 1;

// function declarations 

void LFCPsendACKsimple(LFCPconf *myLFCPconf);



// private functions





// UDP - layer 5 control protocol


void buildsendACKmessage(char hostsource, AsyncUDPPacket packet) {  // only need myhostname (the source hostname) and an unitialized buffer to contain the assembled message

  char *localpayload;
  localpayload = (char*)malloc(strlen(&hostsource)+1+strlen(",ACK,0,0,0,0,0,0"));
  strcpy(localpayload, &hostsource);
  strcat(localpayload, ",ACK,0,0,0,0,0,0");
  packet.printf(localpayload, packet.length());
}


void LFCPresetudpData() { // clean the array used to hold the fields extracted from the payload CSV data.




for(int i = 0; i < FIELDUA; i++){
  for(int j = 0; j < FIELDUB; j++) {
    LFCPdataSplit[i][j] = 0;
  }
}
for(int i = 0; i < FIELDUA; i++) {
  UfieldsLen[i] = 0;
}
LFCPdataLen = 0;
} // end resetudpData()



int LFCPfillinudpData(uint8_t *LFCPpktDataField, int LFCPpktLen) {  // extract data fields from the udp payload
int i = 0;
int j = 0;
int k = 0;
LFCPdataLen = LFCPpktLen;
for(i = 0; i < LFCPdataLen; i++){
  if(LFCPpktDataField[i] == ',') {
    UfieldsLen[j] = k;
    Serial.print("LFCP: received_udp_control_protocol_data:");
    Serial.print(LFCPdataSplit[j]);
    Serial.print("; data_len:");
    Serial.print(k);
    Serial.print("; field_position:");
    Serial.println(j);
    j++;
    k = 0;
  }
  if(j >= FIELDUA) {
    Serial.print("LFCP: ERROR: too many commas in the udp layer 5 control protocol"); // not going to send an error message through the network for this
    return 100;
  }  
  LFCPdataSplit[j][k] = LFCPpktDataField[i];
  k++;
} 
UfieldsLen[j] = k;
return 0;
} // end LFCPfillinudpData()




void LFCPprocessudpCommand(AsyncUDPPacket packet, LFCPconf *LFCPdatastructP) {

  Serial.print("processing udp layer 5 control protocol command directed to: ");
  Serial.print(LFCPdataSplit[0]);
  Serial.print("; command is: ");
  Serial.println(LFCPdataSplit[1]);
  // commands available at the moment, or planned, are: 
  // ACK (confirmed heartbeat reception, do nothing), TLGS (send picture to telegram), TLGC (configure telegram, field 2 = bot token, field 3 = chat id)
  // RST (reboot the device), DSLP (deep sleep)

  
  
  
  if(strcmp(LFCPdatastructP->myhostname, LFCPdataSplit[0]) == 0) {  // matched the field 0, the command / message is directed to me. This field always names the client, NOT THE SERVER.
      
    LFCPnetworkOK = 3; // it is basically proven the server responded. This variable is decreased by one per each heartbeat sent until it reaches 0 meaning there are network problems or the server(s) is(are) unreachable
    // processing the actual commands


    // now we fill in the LFCPdatastruct fields from 1 to 7 with the received data.
       
    strcpy(LFCPdatastructP->command, LFCPdataSplit[1]);
    strcpy(LFCPdatastructP->param1, LFCPdataSplit[2]);
    strcpy(LFCPdatastructP->param2, LFCPdataSplit[3]);
    strcpy(LFCPdatastructP->param3, LFCPdataSplit[4]);
    strcpy(LFCPdatastructP->param4, LFCPdataSplit[5]);
    strcpy(LFCPdatastructP->param5, LFCPdataSplit[6]);
    strcpy(LFCPdatastructP->param6, LFCPdataSplit[7]);
    strcpy(LFCPdatastructP->param6, LFCPdataSplit[7]);
      
    if( LFCPdatastructP->commandsInternalExecEnable == 1) {
        

      if(strcmp(LFCPdataSplit[1], "ACK") == 0) {
        // do not reply with an ACK to an ACK, that would be ...disturbing
        
      }
      


      
      if(strcmp(LFCPdataSplit[1], "RST") == 0) {  // restart system
        // I'M A CLIENT, SO I REPLY ACK BY DEFAULT WHENEVER I RECEIVE A COMMAND
        //reply to the client
        //packet.printf(ACK_DATA, packet.length());
        buildsendACKmessage(*LFCPdatastructP->myhostname, packet);
        Serial.println("device_restart_request_received_via_LFCP");
        esp_restart();
      }
      if(strcmp(LFCPdataSplit[1], "DSLP") == 0) { // start deep sleep
        // I'M A CLIENT, SO I REPLY ACK BY DEFAULT WHENEVER I RECEIVE A COMMAND
        //reply to the client
        //packet.printf(ACK_DATA, packet.length());
        //LFCPdatastructP->myhostname // building the ACK reply based on this variable
        buildsendACKmessage(*LFCPdatastructP->myhostname, packet);
        
        Serial.println("RECEIVED REQUEST TO DEEP SLEEP, STARTING TO SLEEP DEEP");
        Serial.println("IF NOT CONDITIONS HAVE BEEN SET TO WAKE ME UP, THIS WILL BE THE SAME AS TURNING ME OFF. MAKE SURE TO DEINIT CAMERA ( esp_err_t err = esp_camera_deinit(); // use only if a camera is installed) BEFORE PUTTING ME TO DEEP SLEEP");
        // esp_err_t err = esp_camera_deinit(); for general usage it's not wise to keep this running
        esp_deep_sleep_start();  // going to deep sleep
      }
      
      

      if(strcmp(LFCPdataSplit[1], "ENAL") == 0) { // enable alarms
        LFCPalarmEnabled = 1;
        buildsendACKmessage(*LFCPdatastructP->myhostname, packet);
      }
      if(strcmp(LFCPdataSplit[1], "DSAL") == 0) { // disable alarms
        LFCPalarmEnabled = 0;
        buildsendACKmessage(*LFCPdatastructP->myhostname, packet);
      }
    }
  } // end process commands
} // END udp process commands function




/*
void LFCPprocessudpCommandOld(AsyncUDPPacket packet) {

  Serial.print("processing udp layer 5 control protocol command directed to: ");
  Serial.print(LFCPdataSplit[0]);
  Serial.print("; command is: ");
  Serial.println(LFCPdataSplit[1]);
  // commands available at the moment, or planned, are: 
  // ACK (confirmed heartbeat reception, do nothing), TLGS (send picture to telegram), TLGC (configure telegram, field 2 = bot token, field 3 = chat id)
  // RST (reboot the device), DSLP (deep sleep)
  if(LFCPdataSplit[0] == LFCPconf *LFCPdatastruct) {  // matched the field 0, the command / message is directed to me. 
      
  LFCPnetworkOK = 3; // it is proven the network responded. This is decreased by one per each heartbeat sent until it reaches 0 meaning there are network problems or the server(s) is(are) unreachable
      // processing the actual commands
      
      if(LFCPdataSplit[1] == "ACK") {
        // do not reply with an ACK to an ACK, that would be ...disturbing
        
      }
      


      
      if(LFCPdataSplit[1] == "RST") {  // restart system
        // I'M A CLIENT, SO I REPLY ACK BY DEFAULT WHENEVER I RECEIVE A COMMAND
        //reply to the client
        packet.printf(ACK_DATA, packet.length());

        
        esp_restart();
      }
      if(LFCPdataSplit[1] == "DSLP") { // start deep sleep
        // I'M A CLIENT, SO I REPLY ACK BY DEFAULT WHENEVER I RECEIVE A COMMAND
        //reply to the client
        packet.printf(ACK_DATA, packet.length());

        
        Serial.println("RECEIVED REQUEST TO DEEP SLEEP, STARTING TO SLEEP DEEP");
        Serial.println("IF NOT CONDITIONS HAVE BEEN SET TO WAKE ME UP, THIS WILL BE THE SAME AS TURNING ME OFF. MAKE SURE TO DEINIT CAMERA ( esp_err_t err = esp_camera_deinit(); // use only if a camera is installed) BEFORE PUTTING ME TO DEEP SLEEP");
        // esp_err_t err = esp_camera_deinit(); for general usage it's not wise to keep this running
        esp_deep_sleep_start();  // going to deep sleep
      }
      
      
      if(LFCPdataSplit[1] == "SCAM") { // deinitialize the camera // use this command only if there actually is a camera connected to the ESP32.
        cameraInitType = 0;
        esp_err_t err = esp_camera_deinit();  // the camera needs a deinit from its current configuration.
      }
      if(LFCPdataSplit[1] == "ENAL") { // enable alarms
        LFCPalarmEnabled = 1;
      }
      if(LFCPdataSplit[1] == "DSAL") { // disable alarms
        LFCPalarmEnabled = 0;
        
      }
  } // end process commands
} // END udp process commands function

*/







// public functions
/*    
int LFCPHeartbeatClient(IPAddress IPLFCPServer, int LFCPport, char* LFCPhostname, AsyncUDP* udpHandle) {
	if( ! udpHandle.connected()) {
		Serial.println("LFCP: UDP connecting to server");
		udp.connect(IPLFCPServer, LFCPport);
		
		if( ! udpHandle.connected()) {
			Serial.print("LFCP: LFCPHeartbeatClient: udp pseudo-connection not opened");
			LFCPnetworkOK = 0;
			return LFCP_UDPFAIL;
		}
		Serial.println("LFCP: UDP connected to server");
	}
  if(udpHandle.connected()) {

	// compose the message

	char *LFCPfullpayload;
	LFCPfullpayload = malloc(strlen(LFCPhostname)+1+strlen(",HEARTBEAT,0,0,0,0,0,0"));
	strcpy(LFCPfullpayload, LFCPhostname);
	strcat(LFCPfullpayload, ",HEARTBEAT,0,0,0,0,0,0");


        Serial.println("LFCP: UDP sending to server A");
        //Send unicast
        udpHandle.print(LFCPfullpayload);  // simple protocol with 8 fields


        
        if ( LFCPnetworkOK > 0 ) { // decrease LFCPnetworkOK until zero is reached, on receiving ACK the counter is reset to a number > 0. This behaviour is that of a watchdog counter.
          LFCPnetworkOK = LFCPnetworkOK - 1;
        }
        udp.onPacket([](AsyncUDPPacket packet) {
            Serial.print("UDP Packet Type: ");
            Serial.print(packet.isBroadcast()?"Broadcast":packet.isMulticast()?"Multicast":"Unicast");
            Serial.print(", From: ");
            Serial.print(packet.remoteIP());
            Serial.print(":");
            Serial.print(packet.remotePort());
            Serial.print(", To: ");
            Serial.print(packet.localIP());
            Serial.print(":");
            Serial.print(packet.localPort());
            Serial.print(", Length: ");
            Serial.print(packet.length());
            Serial.print(", Data: ");
            Serial.write(packet.data(), packet.length());
            Serial.println();
            
            // packet response processing START
            LFCPresetudpData();
            // uint8_t* LFCPpktData[] = {packet.data()};
            int mysplitresult = LFCPfillinudpData(packet.data() ,packet.length()); // splits the received data and fills in the LFCPdataSplit[][] with the data
            if (mysplitresult == 0) { // correctly split the data
              LFCPprocessudpCommandOld(packet); // processes the udp layer 5 control protocol commands // we also let the function access packet in order to enventually send a reply
            }
            
            // packet response processing END
            
            
        });

    	
    }
if ( LFCPnetworkOK == 0 ) {
	return LFCP_SERVUNREACH;
}
return 0;
} //END LFCPHeartbeatClient()
*/
    
/*
int LFCPLogClient(IPAddress IPLFCPServer, int LFCPport, char *LFCPhostname, AsyncUDP* udpHandle, char* LFCPLogMessage) {
	if( ! udpHandle.connected()) {
		Serial.println("LFCP: UDP connecting to server");
		udp.connect(IPLFCPServer, LFCPport);
		
		if( ! udpHandle.connected()) {
			Serial.print("LFCP: LFCPHeartbeatClient: udp pseudo-connection not opened");
			LFCPnetworkOK = 0;
			return LFCP_UDPFAIL;
		}
		Serial.println("LFCP: UDP connected to server");
	}
  if(udpHandle.connected()) {

	// compose the message

	char *LFCPfullpayload;
	LFCPfullpayload = malloc(strlen(LFCPhostname)+1+strlen(",LOG,")+strlen(LFCPLogMessage)+strlen(",0,0,0,0,0"));
	strcpy(LFCPfullpayload, LFCPhostname);
	strcat(LFCPfullpayload, ",LOG,");
	strcat(LFCPfullpayload, LFCPLogMessage);
	strcat(LFCPfullpayload, ",0,0,0,0,0");


        Serial.println("LFCP: UDP sending to server A");
        //Send unicast
        udpHandle.print(LFCPfullpayload);  // simple protocol with 8 fields

        
        if ( LFCPnetworkOK > 0 ) { // decrease LFCPnetworkOK until zero is reached, on receiving ACK the counter is reset to a number > 0. This behaviour is that of a watchdog counter.
          LFCPnetworkOK = LFCPnetworkOK - 1;
        }
        udp.onPacket([](AsyncUDPPacket packet) {
            Serial.print("UDP Packet Type: ");
            Serial.print(packet.isBroadcast()?"Broadcast":packet.isMulticast()?"Multicast":"Unicast");
            Serial.print(", From: ");
            Serial.print(packet.remoteIP());
            Serial.print(":");
            Serial.print(packet.remotePort());
            Serial.print(", To: ");
            Serial.print(packet.localIP());
            Serial.print(":");
            Serial.print(packet.localPort());
            Serial.print(", Length: ");
            Serial.print(packet.length());
            Serial.print(", Data: ");
            Serial.write(packet.data(), packet.length());
            Serial.println();
            
            // packet response processing START
            LFCPresetudpData();
            // uint8_t* LFCPpktData[] = {packet.data()};
            int mysplitresult = LFCPfillinudpData(packet.data() ,packet.length()); // splits the received data and fills in the LFCPdataSplit[][] with the data
            if (mysplitresult == 0) { // correctly split the data
              LFCPprocessudpCommandOld(packet); // processes the udp layer 5 control protocol commands // we also let the function access packet in order to enventually send a reply
            }
            
            // packet response processing END
            
            
        });

    	
    }
if ( LFCPnetworkOK == 0 ) {
	return LFCP_SERVUNREACH;
}
return 0;
} //END LFCPLogClient()

*/











// general usage LFCP command handling

int LFCPcommand(LFCPconf *LFCPdatastruct) {

IPAddress IPLFCPServer = LFCPdatastruct->server;
int LFCPport = LFCPdatastruct->port;
char *LFCPhostname;
LFCPhostname = (char*)malloc(strlen(LFCPdatastruct->myhostname)+1);
strcpy(LFCPhostname, LFCPdatastruct->myhostname);
AsyncUDP* udpHandle = LFCPdatastruct->currentudphandle;

Serial.print("LFCP: command received for server: ");
Serial.print(LFCPdatastruct->server);
Serial.print(" and request: ");
Serial.println(LFCPdatastruct->command);

  if( ! udpHandle->connected()) {
    Serial.print("LFCP: no session: establishing one to server: ");
    Serial.print(IPLFCPServer);
    Serial.print(" and port: ");
    Serial.println(LFCPport);
    
    udpHandle->connect(IPLFCPServer, LFCPport);
    
    if( ! udpHandle->connected()) {
      Serial.print("LFCP: LFCPcommand: udp pseudo-connection not opened");
      LFCPnetworkOK = 0;
      return LFCP_UDPFAIL;
    }
    
    Serial.println("LFCP: UDP connected to server");
    
  }
  if(udpHandle->connected()) {

  // compose the message

  char *LFCPfullpayload;
  LFCPfullpayload = (char*)malloc(strlen(LFCPhostname)+1+strlen(",")+strlen(LFCPdatastruct->command)+strlen(",")+strlen(LFCPdatastruct->param1)+strlen(",")+strlen(LFCPdatastruct->param2)+strlen(",")+strlen(LFCPdatastruct->param3)+strlen(",")+strlen(LFCPdatastruct->param4)+strlen(",")+strlen(LFCPdatastruct->param5)+strlen(",")+strlen(LFCPdatastruct->param6));
  strcpy(LFCPfullpayload, LFCPhostname);
  strcat(LFCPfullpayload, ",");
  strcat(LFCPfullpayload, LFCPdatastruct->command);
  strcat(LFCPfullpayload, ",");
  strcat(LFCPfullpayload, LFCPdatastruct->param1);
  strcat(LFCPfullpayload, ",");
  strcat(LFCPfullpayload, LFCPdatastruct->param2);
  strcat(LFCPfullpayload, ",");
  strcat(LFCPfullpayload, LFCPdatastruct->param3);
  strcat(LFCPfullpayload, ",");
  strcat(LFCPfullpayload, LFCPdatastruct->param4);
  strcat(LFCPfullpayload, ",");
  strcat(LFCPfullpayload, LFCPdatastruct->param5);
  strcat(LFCPfullpayload, ",");
  strcat(LFCPfullpayload, LFCPdatastruct->param6);
  


        Serial.print("LFCP: UDP sending to server: ");
        Serial.print(LFCPdatastruct->server);
        Serial.print(" payload is: ");
        Serial.println(LFCPfullpayload);
        //Send unicast
        udpHandle->print(LFCPfullpayload);  // simple protocol with 8 fields

        
        if ( LFCPnetworkOK > 0 ) { // decrease LFCPnetworkOK until zero is reached, on receiving ACK the counter is reset to a number > 0. This behaviour is that of a watchdog counter.
          LFCPnetworkOK = LFCPnetworkOK - 1;
          Serial.print("LFCP: network watchdog decreasing: no packet received from server yet: ");
          Serial.println(LFCPnetworkOK);
        }
        udpHandle->onPacket([&](AsyncUDPPacket packet) {
            Serial.print("UDP Packet Type: ");
            Serial.print(packet.isBroadcast()?"Broadcast":packet.isMulticast()?"Multicast":"Unicast");
            Serial.print(", From: ");
            Serial.print(packet.remoteIP());
            Serial.print(":");
            Serial.print(packet.remotePort());
            Serial.print(", To: ");
            Serial.print(packet.localIP());
            Serial.print(":");
            Serial.print(packet.localPort());
            Serial.print(", Length: ");
            Serial.print(packet.length());
            Serial.print(", Data: ");
            Serial.write(packet.data(), packet.length());
            Serial.println();
            
            // packet response processing START
            LFCPresetudpData();
            // uint8_t* LFCPpktData[] = {packet.data()};
            int mysplitresult = LFCPfillinudpData(packet.data() ,packet.length()); // splits the received data and fills in the LFCPdataSplit[][] with the data
            if (mysplitresult == 0) { // correctly split the data
              LFCPprocessudpCommand(packet, LFCPdatastruct); // processes the udp layer 5 control protocol commands // we also let the function access packet in order to enventually send a reply
            }
            udpHandle->close(); // at this point, we can close.
            // packet response processing END
            
            
        });

      
    }
if ( LFCPnetworkOK == 0 ) {
  return LFCP_SERVUNREACH;
}

LFCPdatastruct->alarmEnable = LFCPalarmEnabled; // return the current value of the alarm enable variable 
return 0;

} //END LFCPcommand()


void LFCPsendACKsimple(LFCPconf *myLFCPconf) {
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
