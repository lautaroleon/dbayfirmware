#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <Wire.h>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
//#include <SPI.h>
#include <errno.h>
#include <limits.h>
//#include "LTC268x.h"
//#include "PCA9557.h"
#include "dbay_4triacDAC.h"
#include "dbay_32DAC.h"
#include "dbay_4triaxADC.h"
#include "MCP23S08.h"


#define MAX_MSG_LENGTH 1024
//#define MAX_MSG_LENGTH 32
#define LEN(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))
#define ETHERNET
#define DELAY 10

/////digital pin on the VME Bus, is one is used please write it down on a comment here//////
#define D00 23
#define D01 22
#define D02 21
#define D03 24
#define D04 1
#define D05 14
#define D06 4
#define D07 15
#define D13 3
#define D14 5
#define D15 20

///teensy pins connected to the bus, only one is connected per module
#define BADD0 33
#define BADD1 36
#define BADD2 37
#define BADD3 38
#define BADD4 32
#define BADD5 31
#define BADD6 30
#define BADD7 39

#define GPIOi2cAdr0 0
#define GPIOi2cAdr1 1
#define GPIOi2cAdr2 3

//multipropouse auxiliar pins. Write here if you use one of them
#define AUX1 2
#define AUX2 3
#define AUX3 4
#define AUX4 5
#define AUX5 6
#define AUX6 23
#define AUX7 22
#define AUX8 21
#define AUX9 20
#define AUX10 9

/* Base address for the PCA9557. This base address is modified by the three
 * least significant bits set by the DIP switches on each board. */
#define BASE_ADDR 0x18

/* Note: The DIP switches are backwards from what you'd think, i.e. if the
 * switches are in position 0,0,1, then you need to talk to address 0b100. I've
 * ordered the list here so they behave as expected, i.e. if the switches are
 * in position 0,0,1, then you should use bus[1]. */
#define MAXMODULES 8

/* Which module boards are present. For the final setup this should be all
 * 1's. However when testing a small number of boards, we have no way to test
 * which boards are connected since the Wire library resets the whole Teensy if
 * it fails to communicate, so we need to manually keep track of which boards
 * are present. */
int rv=0;

int boardsactive[MAXMODULES]={0};

dbayDev *module[MAXMODULES] = {nullptr};

MCP23S08 *busaddressGPIO[MAXMODULES]={0}; //we will use 8 pins as CS. Only one connect every module
int busaddrarray[MAXMODULES];
int GPIOi2cmap[3] = {GPIOi2cAdr0, GPIOi2cAdr1, GPIOi2cAdr2};

bool debug = false;
char cmd[MAX_MSG_LENGTH];
char err[MAX_MSG_LENGTH];
char msg[MAX_MSG_LENGTH];
int k = 0;

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:

/*
 avoid:
x2-xx-xx-xx-xx-xx
x6-xx-xx-xx-xx-xx
xA-xx-xx-xx-xx-xx
xE-xx-xx-xx-xx-xx
 */


byte mac[] = { 0xFA, 0xAA, 0xAA, 0xAA, 0xAD, 0xAC  };

unsigned int localPort = 8880;      // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,

EthernetUDP Udp;

bool parseInt(const char* str, int& outVal) {
    char* endptr;
    outVal = strtol(str, &endptr, 10);
    return (*endptr == '\0');
}

bool parseDouble(const char* str, double& outVal) {
    char* endptr;
    outVal = strtod(str, &endptr);
    return (*endptr == '\0');
}

bool checkModuleInit(int board, const char* expectedType) {
    if (module[board] == nullptr) {
        sprintf(err, "Board %d not initialized. Use SETDEV.", board);
        return false;
    }
    if (strcmp(module[board]->deviceTypeToString(), expectedType)) {
        sprintf(err, "Expected board %d to be '%s', but found '%s'", board, expectedType, module[board]->deviceTypeToString());
        return false;
    }
    return true;
}

std::vector<std::string> tokenizeCommand(const char* cmd) {
    std::vector<std::string> tokens;
    std::stringstream ss(cmd);
    std::string token;
    while (ss >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

/*
int mystrtoi(const char *str, int *value)
{
    long lvalue;
    char *endptr;
    errno = 0;
    lvalue = strtol(str,&endptr,0);
    if (errno)
        return -1;
    if (endptr == str)
        return -1;
    if (lvalue > INT_MAX || lvalue < INT_MIN)
        return -1;
    *value = (int) lvalue;
    return 0;
}

double mystrtod(const char *nptr, double *value)
{
    char *endptr;
    errno = 0;
    *value = strtod(nptr,&endptr);

    if (endptr == nptr) {
        sprintf(err, "error converting '%s' to a double", nptr);
        return -1;
    } else if (errno != 0) {
        sprintf(err, "error converting '%s' to a double", nptr);
        return -1;
    }

    return 0;
}
*/
int strtobool(const char *str, bool *value)
{
    if (!strcasecmp(str,"on") || !strcasecmp(str,"1"))
        *value = true;
    else if (!strcasecmp(str,"off") || !strcasecmp(str,"0"))
        *value = false;
    else
        return -1;
    return 0;
}

void scanI2C(){
  byte error;
  int address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 24; address < 33; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !  ");
      for (int i = 0; i < MAXMODULES; i++){
            //Serial.println(BASE_ADDR+i, HEX);
            if(address==BASE_ADDR+i){
                  boardsactive[i]=1;
                  //Serial.println(i);Serial.println(address);
                  }
      }
      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

int reset(){

    #ifdef ETHERNET

        Serial.println("initializing ethernet, please wait");
        // start the Ethernet
        Ethernet.begin(mac);
      
        // Check for Ethernet hardware present
        if (Ethernet.hardwareStatus() == EthernetNoHardware) {
            Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
            while (true) {
                delay(1); // do nothing, no point running without Ethernet hardware
            }
        }
        if (debug)Serial.println("there is ethernet hardware");
        if (Ethernet.linkStatus() == LinkOFF) {
            Serial.println("Ethernet cable is not connected.");
        }else if (debug)Serial.println("cable connected");
        if (debug)Serial.println("start udp");
        // start UDP
        Udp.begin(localPort);
        Serial.println("UDP init done");
    #endif
  
    for(int i =0; i<MAXMODULES; i++){
       if(busaddressGPIO[i] == nullptr)busaddressGPIO[i] = new MCP23S08(busaddrarray[i]); 
       busaddressGPIO[i]->begin();
       for(int j = 0 ; j<3; j++){
          busaddressGPIO[i]->pinModeIO(GPIOi2cmap[j], OUTPUT);
          if( (i & 0x01<<j)>>j )busaddressGPIO[i]->digitalWriteIO(GPIOi2cmap[j], true);
          else busaddressGPIO[i]->digitalWriteIO(GPIOi2cmap[j], false);
        }
      }
      
      scanI2C();
      
      for(int i =0; i<MAXMODULES; i++){
        if(boardsactive[i]){
          Serial.print("board active: ");Serial.println(i);
            if(module[i] != nullptr){
                switch(module[i]->thisDeviceType){
                    case(DAC4D): 
                        module[i]->reset();
                        break;
                    default: continue;
                }
            }
        }
    }

    return 0;
}

int setdevicetype( int channel, char *devtypestr){
  
  if( channel <0 || channel >= MAXMODULES){
    Serial.print("channel out of range");
    return -1;
  }
  deviceType devtype=dbayDev::deviceTypeFromString(devtypestr);
  
  Serial.print("deviceType: ");Serial.println(devtype);
  
  if(boardsactive[channel] == 0){
    Serial.println("board is not active");
    return -1;
  }else switch(devtype){
    case NODEV:
      Serial.println("NODEV selected"); 
      return 0;

    case DAC4D: 
      if(module[channel] == nullptr){
        Serial.println("DAC4D created");
        module[channel] = new dbay4triacDAC(BASE_ADDR+channel, &Wire);         
        return 0;

      }else if( module[channel]->thisDeviceType != devtype){
        Serial.println("DAC4D replaced");
        delete module[channel];
        module[channel] = new dbay4triacDAC(BASE_ADDR+channel, &Wire);
        return 0;

      }else return 0;
    case DAC16D:
      if(module[channel] == nullptr){
        Serial.println("DAC16D created");
        module[channel] = new dbay32DAC(BASE_ADDR+channel, &Wire);         
        return 0;

      }else if( module[channel]->thisDeviceType != devtype){
          Serial.println("DAC16D replaced");
          delete module[channel];
          module[channel] = new dbay32DAC(BASE_ADDR+channel, &Wire);
          return 0;

      }else return 0;
      case ADC4D: 
      if(module[channel] == nullptr){
        Serial.println("ADC4D created");
        module[channel] = new dbay4triaxADC(BASE_ADDR+channel, &Wire);         
        return 0;

      }else if( module[channel]->thisDeviceType != devtype){
        Serial.println("DAC4D replaced");
        delete module[channel];
        module[channel] = new dbay4triaxADC(BASE_ADDR+channel, &Wire);
        return 0;

      }else return 0;
    default:
      Serial.println("wrong devtype");
      return -1;
      break;
  }
}

/*
int do_command(char *cmd, float *value){
    int ntok = 0;
    char *tokens[10];
    char *tok;
    int channel, board ;
    double voltage;
    bool ison;
    
    if (cmd[strlen(cmd)-1] == '\n')
        cmd[strlen(cmd)-1] = '\0';

    if (debug) {
        sprintf(msg, "received command: %s\n", cmd);
        Serial.print(msg);
    }

    tok = strtok(cmd, " ");
    while (tok != NULL && ntok < (int) LEN(tokens)) {
        tokens[ntok++] = tok;
        tok = strtok(NULL, " ");
        sprintf(msg, "tok:: %s\t", tok);
        //Serial.print(msg);
    }
    if(!strcmp(tokens[0], "SETDEV")){
        if (ntok != 3) {
            sprintf(err, "SETDEV command expects 2 arguments: [address] [device type]");
            return -1;
        }else if(mystrtoi(tokens[1],&channel)) {
              sprintf(err, "expected argument 2 to be integer but got '%s'", tokens[3]);
              return -1;
            }else if(channel<0 || channel >7) {
              sprintf(err, "channel out of range");
              return -1;
            }else if(setdevicetype(channel, tokens[2]))return -1;
    }else if (!strcmp(tokens[0], "DAC4D")){ 
        if (ntok != 5) {
            sprintf(err, "DAC4D command expects 4 arguments: [VS/VSD] [board] [channel] [voltage]");
            return -1;
        }else if(!strcmp(tokens[1], "VS")){
            if(mystrtoi(tokens[2],&board)) {
              sprintf(err, "expected argument 1 to be integer but got '%s'", tokens[2]);
              return -1;
            }else if(mystrtoi(tokens[3],&channel)) {
              sprintf(err, "expected argument 2 to be integer but got '%s'", tokens[3]);
              return -1;
            }else if (mystrtod(tokens[4],&voltage)) {
              sprintf(err, "expected argument 3 to be double but got '%s'", tokens[4]);
              return -1;
            }else if(module[board] == nullptr){
              sprintf(err, "DAC4D, VS, board is not initialized yet. Use SETDEV.");
              return -1;
            }else if(strcmp(module[board]->deviceTypeToString() , "DAC4D")){
              sprintf(err, "Calling DAC4D command but board is initialized as '%s'", module[board]->deviceTypeToString());
              return -1;
            }else if(module[board]->SetVoltage(channel, voltage))return -1;
        }else if(!strcmp(tokens[1], "VSD")){
            if(mystrtoi(tokens[2],&board)) {
              sprintf(err, "expected argument 1 to be integer but got '%s'", tokens[2]);
              return -1;
            }else if(mystrtoi(tokens[3],&channel)) {
              sprintf(err, "expected argument 2 to be integer but got '%s'", tokens[3]);
              return -1;
            }else if (mystrtod(tokens[4],&voltage)) {
              sprintf(err, "expected argument 3 to be double but got '%s'", tokens[4]);
              return -1;
            }else if(module[board] == nullptr){
              sprintf(err, "DAC4D, VSD, board is not initialized yet. Use SETDEV.");
              return -1;
            }else if(strcmp(module[board]->deviceTypeToString() , "DAC4D")){
              sprintf(err, "Calling DAC4D command but board is initialized as '%s'", module[board]->deviceTypeToString());
              return -1;
            }else if(module[board]->SetVoltageDiff(channel, voltage))return -1;          
        }

    }else if (!strcmp(tokens[0], "DAC16D")){

        
            if(!strcmp(tokens[1], "VS")){
                if (ntok != 5){
                  sprintf(err, "VS command expects 3 arguments: [board] [channel] [voltage]");
                  return -1;
                }else if(mystrtoi(tokens[2],&board)) {
                  sprintf(err, "expected argument 1 to be integer but got '%s'", tokens[2]);
                  return -1;
                }else if(mystrtoi(tokens[3],&channel)) {
                  sprintf(err, "expected argument 2 to be integer but got '%s'", tokens[3]);
                  return -1;
                }else if (mystrtod(tokens[4],&voltage)) {
                  sprintf(err, "expected argument 3 to be double but got '%s'", tokens[4]);
                  return -1;
                }else if(module[board] == nullptr){
                  sprintf(err, "DAC4D, VS, board is not initialized yet. Use SETDEV.");
                  return -1;
                }else if(strcmp(module[board]->deviceTypeToString() , "DAC16D")){
                  sprintf(err, "Calling DAC16D command but board is initialized as '%s'", module[board]->deviceTypeToString());
                  return -1;
                }else if(module[board]->SetVoltage(channel, voltage))return -1;
            }else if(!strcmp(tokens[1], "VSD")){
                if (ntok != 5){
                  sprintf(err, "VSD command expects 3 arguments: [board] [channel] [voltage]");
                  return -1;
                }else if(mystrtoi(tokens[2],&board)) {
                  sprintf(err, "expected argument 1 to be integer but got '%s'", tokens[2]);
                  return -1;
                }else if(mystrtoi(tokens[3],&channel)) {
                  sprintf(err, "expected argument 2 to be integer but got '%s'", tokens[3]);
                  return -1;
                }else if (mystrtod(tokens[4],&voltage)) {
                  sprintf(err, "expected argument 3 to be double but got '%s'", tokens[4]);
                  return -1;
                }else if(module[board] == nullptr){
                  sprintf(err, "DAC16D, VSD, board is not initialized yet. Use SETDEV.");
                  return -1;
                }else if(strcmp(module[board]->deviceTypeToString() , "DAC16D")){
                  sprintf(err, "Calling DAC16D command but board is initialized as '%s'", module[board]->deviceTypeToString());
                  return -1;
                }else if(module[board]->SetVoltageDiff(channel, voltage))return -1;          
            }else if(!strcmp(tokens[1], "VSB")){
                if (ntok != 4){
                  sprintf(err, "VSB command expects 2 arguments: [board] [voltage] %d",ntok);
                  return -1;
                }else if(mystrtoi(tokens[2],&board)) {
                  sprintf(err, "expected argument 1 to be integer but got '%s'", tokens[2]);
                  return -1;
                }else if (mystrtod(tokens[3],&voltage)) {
                   sprintf(err, "expected argument 2 to be double but got '%s'", tokens[4]);
                   return -1;
                }else if(module[board] == nullptr){
                  sprintf(err, "DAC16D, VSB, board is not initialized yet. Use SETDEV.");
                  return -1;
                }else if(strcmp(module[board]->deviceTypeToString() , "DAC16D")){
                  sprintf(err, "Calling DAC16D command but board is initialized as '%s'", module[board]->deviceTypeToString());
                  return -1;
                }else if(module[board]->SetVoltage(-1, voltage))return -1; 
            }else if(!strcmp(tokens[1], "VR")){
                if (ntok != 3){
                  sprintf(err, "VR command expects 1 arguments: [board]");
                  return -1;
                }else if(mystrtoi(tokens[2],&board)) {
                    sprintf(err, "expected argument 1 to be integer but got '%s'", tokens[2]);
                    return -1;
                }else if(module[board] == nullptr){
                  sprintf(err, "DAC16D, VR, board is not initialized yet. Use SETDEV.");
                  return -1;
                }else if(strcmp(module[board]->deviceTypeToString() , "DAC16D")){
                  sprintf(err, "Calling DAC16D command but board is initialized as '%s'", module[board]->deviceTypeToString());
                  return -1;
                }else{
                  //Serial.println("hey");
                    double chP = module[board]->ReadVoltage(0);
                   // double chN = module[board]->ReadVoltage(1);
                    *value = (float)chP;
                    return 2;
                }

            }
            else{
              sprintf(err, "Unknown parameter for DAC16D");
              return -1;
            }
        
    }else if (!strcmp(tokens[0], "debug")) {
          if (ntok != 2) {
              sprintf(err, "debug command expects 1 argument: debug [on/off]");
              return -1;
          }
  
          if (strtobool(tokens[1],&ison)) {
              sprintf(err, "expected argument 1 to be yes/no but got '%s'", tokens[1]);
              return -1;
          }
          debug = ison;
    }else if (!strcmp(tokens[0], "reset")) {
          
          return (reset());
    }else if (!strcmp(tokens[0], "help")) {
        sprintf(err,"SetDac [board] [channel] [voltage]\n"
                      "help\n"
                      "debug");
                   
        return -1;
    }else {
        sprintf(err, "unknown command '%s'", tokens[0]);
        //sprintf(err, "error : message '%s'", cmd);
        return -1;
    }
 
    return 0;
}
*/
int do_command(char *cmd, float *value) {
    std::vector<std::string> tokens = tokenizeCommand(cmd);
    if (tokens.empty()) {
        sprintf(err, "Empty command received.");
        return -1;
    }

    const std::string& command = tokens[0];
    deviceType devtype = dbayDev::deviceTypeFromString(command.c_str());
    int board, channel;
    double voltage;


    if (command == "SETDEV") {
        if (tokens.size() != 3) {
            sprintf(err, "SETDEV requires 2 arguments: [address] [device type]");
            return -1;
        }
        if (!parseInt(tokens[1].c_str(), board) || board < 0 || board > 7) {
            sprintf(err, "Invalid board number: %s", tokens[1].c_str());
            return -1;
        }
        return setdevicetype(board, tokens[2].c_str()) ? -1 : 0;
    }
 
    else if(devtype != NODEV ){
      const std::string& func = tokens[1];
      if (!parseInt(tokens[2].c_str(), board) || board < 0 || board > 7) {
        sprintf(err, "Invalid board number: %s", tokens[2].c_str());
        return -1;
      }
      else if(boardsactive[board] == 0){
        Serial.println("board is not active");
        return -1;
      }else if( module[board]->thisDeviceType != devtype)Serial.println("use SETDEV with the proper board type");

      else switch(devtype){
        case DAC4D:    //DAC 4 diff channels (triax)
          if(func == "VS"){
            if(tokens.size() != 5){
              sprintf(err, "DAC4D VS requires 5 arguments, type help");
              return -1;
            }else if (!parseInt(tokens[3].c_str(), channel)|| channel < 0 || channel > 7) {
              sprintf(err, "Invalid channel number: %s", tokens[3].c_str());
              return -1;
            }else if (!parseDouble(tokens[4].c_str(), voltage)) {
              sprintf(err, "Invalid voltage value: %s", tokens[4].c_str());
              return -1;
            }else return module[board]->SetVoltage(channel, voltage) ? -1 : 0;
          }else if(func == "VSD"){
            if(tokens.size() != 5){
              sprintf(err, "DAC4D VSD requires 5 arguments, type help");
              return -1;
            }else if (!parseInt(tokens[3].c_str(), channel)|| channel < 0 || channel > 3) {
              sprintf(err, "Invalid channel number: %s", tokens[3].c_str());
              return -1;
            }else if (!parseDouble(tokens[4].c_str(), voltage)) {
              sprintf(err, "Invalid voltage value: %s", tokens[4].c_str());
              return -1;
            }else return module[board]->SetVoltageDiff(channel, voltage) ? -1 : 0;
          }else sprintf(err, "unknown command for DAC4D");
        
        case DAC16D:  //DAC 16 differential + 500mA 5V + 8V 1mA + 1 diff ADC
          if(func == "VS"){
            if(tokens.size() != 5){
              sprintf(err, "DAC16D VS requires 5 arguments, type help");
              return -1;
            }else if (!parseInt(tokens[3].c_str(), channel)|| channel < 0 || channel > 31) {
              sprintf(err, "Invalid channel number: %s", tokens[3].c_str());
              return -1;
            }else if (!parseDouble(tokens[4].c_str(), voltage)) {
              sprintf(err, "Invalid voltage value: %s", tokens[4].c_str());
              return -1;
            }else return module[board]->SetVoltage(channel, voltage) ? -1 : 0;
          }else if(func == "VSD"){
            if(tokens.size() != 5){
              sprintf(err, "DAC16D VSD requires 5 arguments, type help");
              return -1;
            }else if (!parseInt(tokens[3].c_str(), channel)|| channel < -3 || channel > 15) {
              sprintf(err, "Invalid channel number: %s", tokens[3].c_str());
              return -1;
            }else if (!parseDouble(tokens[4].c_str(), voltage)) {
              sprintf(err, "Invalid voltage value: %s", tokens[4].c_str());
              return -1;
            }else return module[board]->SetVoltageDiff(channel, voltage) ? -1 : 0;
          }else if(func == "VR"){
            if(tokens.size() != 3){
              sprintf(err, "DAC16D VR requires 3 arguments, type help");
              return -1;
            }else{
              double chP = module[board]->ReadVoltage(0);
              double chN = module[board]->ReadVoltage(1);
              
              *value = (float)(chP-chN);
              return 2;
            } 
          }else if(func == "VSB"){
            if(tokens.size() != 4){
              sprintf(err, "DAC16D VSB requires 4 arguments, type help");
              return -1;
            }else if (!parseDouble(tokens[3].c_str(), voltage)) {
              sprintf(err, "Invalid voltage value: %s", tokens[4].c_str());
              return -1;
            }else return module[board]->SetVoltage(-1, voltage) ? -1 : 0;
          }else{
            sprintf(err, "unknown command for DAC16D");
            return -1;
          }
        
        case FAFD:   return 0;//4ADC + 4DAC
        case HIC4:  return 0; //4DAC high current
        case ADC4D: 
        if(func == "VRD"){
          if(tokens.size() != 4){
            sprintf(err, "DAC16D VSD requires 4 arguments, type help");
            return -1;
          }else if (!parseInt(tokens[3].c_str(), channel)|| channel < 0 || channel > 4) {
            sprintf(err, "Invalid channel number: %s", tokens[3].c_str());
            return -1;
          }else {
            double chP = module[board]->ReadVoltage(2*channel+1);
            double chN = module[board]->ReadVoltage(2*channel);
            
            *value = (float)(chP-chN);
            return 2;
          }
        }else {
          sprintf(err, "unknown command for DAC4D");
          return -1;
        }
      }
    }
/*

    else if (command == "DAC4D" || command == "DAC16D") {
        if (tokens.size() < 5) {
            sprintf(err, "%s requires at least 4 arguments.", command.c_str());
            return -1;
        }

        const std::string& mode = tokens[1];
        int board, channel;
        double voltage;

        if (!parseInt(tokens[2].c_str(), board)) {
            sprintf(err, "Invalid board number: %s", tokens[2].c_str());
            return -1;
        }
        if (!parseInt(tokens[3].c_str(), channel)) {
            sprintf(err, "Invalid channel number: %s", tokens[3].c_str());
            return -1;
        }
        if (!parseDouble(tokens[4].c_str(), voltage)) {
            sprintf(err, "Invalid voltage value: %s", tokens[4].c_str());
            return -1;
        }

        if (!checkModuleInit(board, command.c_str())) {
            return -1;
        }

        if (mode == "VS") {
            return module[board]->SetVoltage(channel, voltage) ? -1 : 0;
        } else if (mode == "VSD") {
            return module[board]->SetVoltageDiff(channel, voltage) ? -1 : 0;
        } else {
            sprintf(err, "Unknown mode: %s", mode.c_str());
            return -1;
        }
    }
*/
    else if (command == "debug") {
        if (tokens.size() != 2) {
            sprintf(err, "debug command expects 1 argument: debug [on/off]");
            return -1;
        }
        bool ison;
        if (strtobool(tokens[1].c_str(), &ison)) {
            sprintf(err, "Invalid argument for debug: %s", tokens[1].c_str());
            return -1;
        }
        debug = ison;
        return 0;
    }

    else if (command == "reset") {
        return reset();
    }

    else if (command == "help") {
        sprintf(err, "Available commands:\n"
                    "SETDEV [board] [device]\n"
                    "DAC4D [VS/VSD] [board] [channel] [voltage]\n"
                    "DAC16D [VS/VSD] [board] [channel] [voltage]\n"
                    "DAC16D VSB [board] [voltage]\n"
                    "DAC16D VR\n"
                    "debug [on/off]\n"
                    "reset\n"
                    "help"
                    "debug\n"
                    "\n"
                    "ip: %d.%d.%d.%d\n"
                    "MAC: %x-%x-%x-%x-%x-%x",
                    Ethernet.localIP()[0],
                    Ethernet.localIP()[1],
                    Ethernet.localIP()[2],
                    Ethernet.localIP()[3],
                    mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
        return -1;
    }

    else {
        sprintf(err, "Unknown command: %s", command.c_str());
        return -1;
    }

    return 0;
}
void format_message(int rv, float value)
{
    //Serial.print("return from the func on f m: ");
    //Serial.println(rv);
    if (rv < 0) {
        sprintf(msg, "-%s\n", err);
    } else if (rv == 1) {
        sprintf(msg, ":%i\n", (int) value);
    } else if (rv == 2) {
        char floatmsg[64];
        dtostrf(value, 6, 6, floatmsg);
        //Serial.print("floatmsg: ");
        //Serial.println(floatmsg);
        //sprintf(msg, ",%.18f\n", value);
        sprintf(msg, ",%s\n", floatmsg);
        //Serial.print("msg: ");
        //Serial.println(msg);

    } else {
        sprintf(msg, "+ok\n");
    }
}

void setup()
{
  Serial.begin(9600);
  if (debug)Serial.println("init");
  SPI.begin();
  Wire.begin();
  
//pinMode(18, INPUT_PULLUP);
//pinMode(19, INPUT_PULLUP);

  
  pinMode(D01,OUTPUT);
  pinMode(D02,OUTPUT);
  pinMode(D03,OUTPUT);
  pinMode(D04,OUTPUT);
  pinMode(D05,OUTPUT);
  pinMode(D06,OUTPUT);
  pinMode(D07,OUTPUT);
  pinMode(D13,OUTPUT);
  pinMode(D14,OUTPUT);
  pinMode(D15,OUTPUT);

  busaddrarray[0]=BADD0;
  busaddrarray[1]=BADD1;
  busaddrarray[2]=BADD2;
  busaddrarray[3]=BADD3;
  busaddrarray[4]=BADD4;
  busaddrarray[5]=BADD5;
  busaddrarray[6]=BADD6;
  busaddrarray[7]=BADD7;

  for(int i =0; i<MAXMODULES; i++){
    pinMode(busaddrarray[i],OUTPUT);
    digitalWrite(busaddrarray[i], HIGH);
  }

//multipropouse auxiliar pins. Write here if you use one of them
  pinMode(AUX1,OUTPUT);
  pinMode(AUX2,OUTPUT);
  pinMode(AUX3,OUTPUT);
  pinMode(AUX4,OUTPUT);
  pinMode(AUX5,OUTPUT);
  pinMode(AUX6,OUTPUT);
  pinMode(AUX7,OUTPUT);
  pinMode(AUX8,OUTPUT);
  pinMode(AUX9,OUTPUT);
  pinMode(AUX10,OUTPUT);


  reset();
  //int board = 0;
}

void loop()
{
    float temp = 0;
    while (Serial.available() > 0) {
        if (k >= (int) LEN(cmd) - 1) {
            Serial.print("Error: too many characters in command!\n");
            k = 0;
          while (Serial.available()) Serial.read();  // Flush excess characters
          return;
        }
        cmd[k++] = Serial.read();
        if (cmd[0] == '\0' || isspace(cmd[0])) {
          Serial.println("Ignoring empty command.");
          k = 0;
          return;
        }
        if (cmd[k-1] == '\n') {
            cmd[k-1] = '\0';
            temp = 0;
            int rv = do_command(cmd, &temp);
            format_message(rv,temp);
            Serial.print(msg);
            
            if(debug){
              Serial.print("command on serial port: ");
              Serial.println(cmd);
            }
            k = 0;
        }
    }
  /*while (Serial.available() > 0) {
        char incomingChar = Serial.read();

        if (incomingChar == '\n' || incomingChar == '\r') {  // End of command
            if (k > 0) {  // Ensure buffer is not empty
                cmd[k] = '\0';  // Null-terminate the string
                 int rv = do_command(cmd, &temp);
                 format_message(rv,temp);
                 Serial.print(msg);
                k = 0;  // Reset buffer for next command
            }
        } else if (k < MAX_MSG_LENGTH - 1) {  // Prevent buffer overflow
            cmd[k++] = incomingChar;
        } else {
            Serial.println("Error: Command too long");
            k = 0;  // Reset buffer to avoid overflow
        }
    }*/

#ifdef ETHERNET
    // if there's data available, read a packet
    int packetSize = Udp.parsePacket();
    //if (debug)Serial.print("read from LAN:  ");
    if (debug && packetSize)Serial.println(packetSize);
    if (packetSize) {
        if (packetBuffer[packetSize-1] == '\n')
            packetBuffer[packetSize-1] = '\0';

        packetBuffer[packetSize] = '\0';

        if (debug) {
            Serial.print("Received packet of size ");
            Serial.println(packetSize);
            Serial.print("From ");
            IPAddress remote = Udp.remoteIP();
            for (int i=0; i < 4; i++) {
                Serial.print(remote[i], DEC);
                if (i < 3) {
                    Serial.print(".");
                }
            }

            Serial.print(", port ");
            Serial.println(Udp.remotePort());
        }

        // read the packet into packetBufffer
        Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
        //Serial.println("Contents:");
        //Serial.println(packetBuffer);

        temp = 0;
        rv = do_command(packetBuffer, &temp);
        format_message(rv,temp);
        
          Serial.print(msg);
        

        // send a reply to the IP address and port that sent us the packet we received
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(msg);
        Udp.endPacket();
    }
#endif
}
