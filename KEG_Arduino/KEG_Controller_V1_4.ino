#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "driver/adc.h"

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#include "rom/ets_sys.h"
#include "esp_timer.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>


const char* host = "esp32";
//const char* ssid = "chum bucket";
//const char* password = "buckethat";

WebServer server(80);



// Start Login Index Page
const char* loginIndex = 
 "<form name='loginForm'>"
    "<table width='20%' bgcolor='A09F9F' align='center'>"
        "<tr>"
            "<td colspan=2>"
                "<center><font size=4><b>ESP32 Login Page</b></font></center>"
                "<br>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<td>Username:</td>"
        "<td><input type='text' size=25 name='userid'><br></td>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td>Password:</td>"
            "<td><input type='Password' size=25 name='pwd'><br></td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
            "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
        "</tr>"
    "</table>"
"</form>"
"<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='admin' && form.pwd.value=='admin')"
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username')/*displays error message*/"
    "}"
    "}"
"</script>";
// Start Login Index Page
 
// Start Server Index Page
const char* serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')" 
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";
 // End Server Index Page




#include <Preferences.h>

Preferences preferences;

// Start BLE Setup partg 1
#define bleServerName "KEG_V1_4_G"
bool deviceConnected = false;
String BLEpassword;
int password_correct = 0;
String receivedMSG;
char AnalogMSG[19] = {' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};
char AnalogCalibMSG[59];
char AnalogDeadzoneMSG[47];
char DigitalMappingMSG[35];
char ipAddy[16];
int savedCalib = 0;
int savedDeadzones = 0;
int savedButtonMapping = 0;
int passWriteFlag = 0;
int resetPasswordFlag = 0;
unsigned long bt_millis_count;
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_2 "da1e7d98-916b-11ed-a1eb-0242ac120002"
BLECharacteristic Ch1(CHARACTERISTIC_UUID,BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
BLEDescriptor Dp1(BLEUUID((uint16_t)0x2902));
BLECharacteristic Ch2(CHARACTERISTIC_UUID_2,BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
BLEDescriptor Dp2(BLEUUID((uint16_t)0x2902));
//Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    password_correct = 0; // reset for next connection
  }
};
// End BLE Setup Part 1



#define ZeroZero 4
#define OneOne 55
#define ZeroOne 52
#define OneZero 7
#define Stop 63

#define ProbeReplyLen 13
#define InGameReplyLen 33

#define OriginEndLen 9

#define ProbeLen 5
#define OriginLen 5
#define PollLen 13

#define DigitalInLen 16
#define AnalogInLen 6



#define RXD2 16
#define TXD2 17
#define buffer_holder_len 45

#define LEDPin 2

hw_timer_t *My_timer = NULL;
int timer_flag = 1;
int triggered = 0;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

int prog_t1 = 0;
int prog_t2 = 0;
int program_flag = 0;
int stick_cal_flag = 0;

int dump = 0;

int buffer_holder[buffer_holder_len];
int readVal = 0;
int buff_index = 0;

int probe1[] = {ZeroZero,ZeroZero,ZeroZero,ZeroZero,Stop};
int probe2[] = {OneOne,OneOne,OneOne,OneOne,Stop};

int origin1[] = {ZeroOne,ZeroZero,ZeroZero,ZeroOne,Stop};
int origin2[] = {ZeroOne,ZeroZero,ZeroZero,OneZero,Stop};

int ProbeReply[ProbeReplyLen] = {ZeroZero, ZeroZero, OneZero, ZeroOne, ZeroZero, ZeroZero, ZeroZero, ZeroZero, ZeroZero, ZeroZero, ZeroZero, OneOne,Stop};

int InGameReply[InGameReplyLen] = {ZeroZero, ZeroZero, ZeroZero, ZeroZero, // 0 0 0 Start Y X B A
                                       OneZero, ZeroZero, ZeroZero, ZeroZero, // 1 L R Z D-U D-D D-R D-L
                                       OneZero, ZeroZero, ZeroZero, ZeroZero, // Analog X
                                       OneZero, ZeroZero, ZeroZero, ZeroZero, // Analog Y
                                       OneZero, ZeroZero, ZeroZero, ZeroZero, // C-Stick X
                                       OneZero, ZeroZero, ZeroZero, ZeroZero, // C-Stick Y
                                       ZeroZero, ZeroZero, ZeroZero, ZeroZero, // L-Trigger
                                       ZeroZero, ZeroZero, ZeroZero, ZeroZero, // R-Trigger
                                       Stop};


int OriginEnd[OriginEndLen] = {ZeroZero, ZeroZero, ZeroZero, ZeroZero, // Null
                               ZeroZero, ZeroZero, ZeroZero, ZeroZero, // Null
                               Stop}; 

int buttons_in[DigitalInLen] = {0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0}; // 0 0 0 Start Y X B A 1 L R Z D-U D-D D-R D-L
int analogs_in[AnalogInLen] = {128,128,128,128,0,0}; // Analog-X Analog-Y C-Stick-X C-Stick-Y L-Trigger R-Trigger

int digital_input_pins[DigitalInLen] = {-1,-1,-1,21,4,18,23,22,-1,5,19,15,32,25,33,14}; // should save pin 2 to use for rumble (it's connected to the onboard LED), pin 2 on the devboard is attatched to a pull down resistor, Note gpio5 is connected to a pull up resistor
int analog_input_pins[AnalogInLen] = {34,39,35,13,26,27}; // maybe change 12 for 36 (aka vp) or 39 (aka vn) as 12 needs to be low during boot but currently is not??

int digital_mapping[DigitalInLen] = {-1,-1,-1,21,4,18,23,22,-1,5,19,15,32,25,33,14};
int digital_toggling[DigitalInLen] = {0,0,0,1,1,1,1,1,0,1,1,1,1,1,1,1};



int read_counter = 0;
int read_counter1 = 0;
int loop_counter = 0;

int timer_arr_1[] = {0,0};
int stick_cal_timer[] = {0,0};



int AX = 0;
int AY = 0;
int CX = 0;
int CY = 0;
int AL = 0;
int AR = 0;

int ax = 0;
int ay = 0;
int cx = 0;
int cy = 0;
int al = 0;
int ar = 0;

int aX = 0;
int aY = 0;
int cX = 0;
int cY = 0;
int aL = 0;
int aR = 0;

// start stick cal values
int AXNeutch;
int AXHigh;
int AXLow;

int AYNeutch;
int AYHigh;
int AYLow;

int CXNeutch;
int CXHigh;
int CXLow;

int CYNeutch;
int CYHigh;
int CYLow;
// end stick cal values

// start stick deadzone values
int AXDeadzone[3];
int AYDeadzone[3];
int CXDeadzone[3];
int CYDeadzone[3];
// end stick deadzone values

// start trigger cal values
int LTLow = 0;
int LTHigh = 255;

int RTLow = 0;
int RTHigh = 255;
//end trigger cal values

int flags = 0;

int stop_bit_9 = 0;
int command_byte = 0;

int num_probes = 0;
int num_origins = 0;
int num_polls = 0;
int num_misc = 0;
int num_updates = 0;
int uhohflag = 0;

int wifi_flag = 0;


TaskHandle_t ReadInputs;

void IRAM_ATTR onTimer(){
  triggered = 1;
//  digitalWrite(LEDPin, LOW);
}

void setup() {
  fill_buff_hold();

  readStickCalFromMem(); // read the stick calibration values from memory
  readStickDeadzonesFromMem(); // read the stick deadzone values from memory
  readButtonMappingFromMem(); // read button mapping from memory
  readBLEpasswordFromMem(); // read BLE Password from memory

  for(int i = 0; i<DigitalInLen; i++){
    if (digital_input_pins[i] !=  -1){
      pinMode(digital_input_pins[i],INPUT_PULLUP);
    }
  }

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); // GPIO 34
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11); // GPIO 39
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11); // GPIO 35
  adc2_config_channel_atten(ADC2_CHANNEL_4, ADC_ATTEN_DB_11); // GPIO 13
  adc2_config_channel_atten(ADC2_CHANNEL_9, ADC_ATTEN_DB_11); // GPIO 26
  adc2_config_channel_atten(ADC2_CHANNEL_7, ADC_ATTEN_DB_11); // GPIO 27

  // check to see if holding down x and y on boot
  // if yes set the reset password flag
  // allows users to change old passwords without knowing current password
  if(!digitalRead(18) && !digitalRead(4)){
    resetPasswordFlag = 1;
  }
  
  

  //Start BLE Setup Part 2
  BLEDevice::init(bleServerName);
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *Service1 = pServer->createService(SERVICE_UUID);
  Service1->addCharacteristic(&Ch1);
  Service1->addCharacteristic(&Ch2);
  Ch1.addDescriptor(&Dp1);
  if(resetPasswordFlag == 1){
    Ch1.setValue("Reset Password");
  }
  else{
    Ch1.setValue("Password Incorrect");
  }
  Ch2.addDescriptor(&Dp2);
  Ch2.setValue("Waiting for change to exactly: A");
  // Start the service
  Service1->start();
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  bt_millis_count = millis();
  // End BLE Setup Part 2

  
  
  pinMode(LEDPin,OUTPUT);

  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 115, false);
  
  Serial.begin(9600);

  
  
  
  

  xTaskCreatePinnedToCore(read_inputs,"Read Inputs",10000,NULL,0,&ReadInputs,  /* Task handle. */0 /*core #*/);
  
  Serial2.begin(800000, SERIAL_6N1, RXD2, TXD2);
  Serial2.setRxBufferSize(256);
}



void loop(){
    communications();
    if(wifi_flag == 1){
      server.handleClient();
    }
}

// The task called on core 0
// Calls the function to update the input state arrays and delays as to not setoff a watchdog timeout
void read_inputs( void * parameter) {
//void loop(){
  for(;;) {
    check_buttons();

    if(millis()-bt_millis_count>=90){
      BLEHandler();
      bt_millis_count = millis();
    }
    
    TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed=1;
    TIMERG0.wdt_wprotect=0;

    for(int i =0; i<10000; i++){
      __asm__("nop\n\t");
    }
    
    read_counter1++;
    if(read_counter1>= 10){
      update_reply();
      read_counter1 = 0;
    }

    if(num_polls >= 1000){
//      Serial.println();
//      Serial.print("Num Probes = ");
//      Serial.println(num_probes);
//      Serial.print("Num Origins = ");
//      Serial.println(num_origins);
//      Serial.print("Num Polls = ");
//      Serial.println(num_polls);
//      Serial.print("Num Misc = ");
//      Serial.println(num_misc);
//      Serial.print("Num Updates = ");
//      Serial.println(num_updates);
//      Serial.print("Uh Oh flag = ");
//      Serial.println(uhohflag);
//      printArr(analogs_in,AnalogInLen);
      num_probes = 0;
      num_origins = 0;
      num_polls = 0;
      num_misc = 0;
      num_updates = 0;
      uhohflag = 0;
    }
  }
}

// reads the values from all the digital and analog inputs
// saves the read values into the appropriate arrays
void check_buttons(){
  for(int i = 0; i<DigitalInLen; i++){
    if (digital_input_pins[i] !=-1){
      buttons_in[i] = !digitalRead(digital_input_pins[i]);
    }
  }
  
  for(int i = 0; i<AnalogInLen; i++){
    switch (i){
      case 0:
        AX = adc1_get_raw(ADC1_CHANNEL_6);
//        analogs_in[i] = linearize(AX_SL,AX_SH,AX_N,AX);
        ax+=AX;///16;
        break;
      case 1:
        AY = adc1_get_raw(ADC1_CHANNEL_3);
//        analogs_in[i] = linearize(i,AY);
        ay+=AY;///16;
        break;
      case 2:
        CX = adc1_get_raw(ADC1_CHANNEL_7);
//        analogs_in[i] = linearize(i,CX);
        cx+=CX;
        break;
      case 3:
        adc2_get_raw(ADC2_CHANNEL_4,ADC_WIDTH_BIT_12,&CY);
//        analogs_in[i] = linearize(i,CY);
        cy+=CY;
        break;
      case 4:
        adc2_get_raw(ADC2_CHANNEL_9,ADC_WIDTH_BIT_9,&AL);
//        analogs_in[i] = 0; //AL/2;
        al+=AL/2;
        break;
      case 5:
        adc2_get_raw(ADC2_CHANNEL_7,ADC_WIDTH_BIT_9,&AR);
//        analogs_in[i] = 0; //AR/2;ar
        ar+=AR/2;
        break;
      default:
        Serial.println("No Analog Read");
        break;
    }
  }
}

// loops through the digital input array and converts the readings to 6 bit UART encodings
// loops through analog input array and converts the readings to 6 bit UART encodings
// these are then used to update the output signal from the controller
void update_reply(){
  int next_val = 0;
  int j = 0;
  int count = 0;
  int val1 = 0;
  int val2 = 0;
  
  // check for mechanical L+R+A+Start depression
  // if they are depressed then changes the buttons_in array to reflect that
  checkForLRAStart();
  
  for (int i = 0; i<DigitalInLen; i=i+2){
    j = i+1;
    val1 = buttons_in[i];
    val2 = buttons_in[j];
    next_val = TwoBitsToMessage(val1, val2);
    InGameReply[count] = next_val;
    count++;
  }
  aX = ax/read_counter1;
  analogs_in[0] = mapStickVals(AXLow,AXHigh,AXNeutch,aX,AXDeadzone);
  ax = 0;
  aY = ay/read_counter1;
  analogs_in[1] = mapStickVals(AYLow,AYHigh,AYNeutch,aY,AYDeadzone);
  ay = 0;
  cX = cx/read_counter1;
  analogs_in[2] = mapStickVals(CXLow,CXHigh,CXNeutch,cX,CXDeadzone);
  cx = 0;
  cY = cy/read_counter1;
  analogs_in[3] = mapStickVals(CYLow,CYHigh,CYNeutch,cY,CYDeadzone);
  cy = 0;
  aL = al/read_counter1;
//  analogs_in[4] = mapTriggerVals(LTHigh,LTLow,aL);
  analogs_in[4] = aL;
  al = 0;
  aR = ar/read_counter1;
//  analogs_in[5] = mapTriggerVals(RTHigh,RTLow,aR);
  analogs_in[5] = aR;
  ar = 0;

  
  int analog_value = 0;
  for (int k = 0; k<AnalogInLen; k++){
    analog_value = analogs_in[k];
    for (int i = 0; i<7; i=i+2){
      j = i+1;
      val1 = bitRead(analog_value,7-i);
      val2 = bitRead(analog_value,7-j);
      next_val = TwoBitsToMessage(val1, val2);
      InGameReply[count] = next_val;
      count++;
    }
  }
//  if(InGameReply[1]!=ZeroZero){
//    Serial.println(InGameReply[1]);
//  }
//  num_updates++;
}


// writes data over 6 bit uart line serial 2
// Absolutely crucial that this function have the portENTER_CRITICAL_ISR wrapping the serial writes
// this solved the "start" bug as it seems something was interupting the serial writes
void writeData(int dat[], int len){
  portENTER_CRITICAL_ISR(&timerMux);
  for (int i = 0; i<len; i++){
    Serial2.write(dat[i]);
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}


// initialize the buffer holder with all 0's
void fill_buff_hold(){
  for(int i =0; i<buffer_holder_len; i++){
    buffer_holder[i] = 0;
  }
}


// read all values in the input buffer into a dump variable
void Clear_UART_Rx(){
  while(true){
    if(Serial2.available()){
      dump = Serial2.read();
      if(dump == 63){
        break;
      }
    }  
  }
}

// prints the contents of array arr to the serial monitor
void printArr(int arr[], int len){
  for (int i=0; i<len; i++){
    Serial.println(arr[i]);
  }
}

// prints the contents of an char array arr to the serial monitor
void printCharArr(char arr[], int len){
  for (int i=0; i<len; i++){
    Serial.print(arr[i]);
  }
  Serial.println();
}

// get a strange insertion of -1 into some of the read data
// this function looks through the buffer holder and removes any single -1 that is present
// it then shifts the rest of the values into place
// think the -1 is when it reads but there is no data available, which seems like it should never be possibole but we'll keep that on the backburner
void cleanUpBufferRead(int len){
  int j = -1;
  for(int i = 0; i<len; i++){
    if (buffer_holder[i] == -1){
      j = i;
    }
  }
  if (j!=-1){
    for (int i=j; i<len-1; i++){
      buffer_holder[i] = buffer_holder[i+1];
    }
    buff_index = buff_index - 1;
  }
}

// Takes 2 bit input and returns the equivalent message integer value
int TwoBitsToMessage(int val1, int val2){
  int next_val = -1;
  if (val1 == 0 && val2 == 0){
      next_val = ZeroZero;
    }
    if (val1 == 0 && val2 == 1){
      next_val = ZeroOne;
    }
    if (val1 == 1 && val2 == 0){
      next_val = OneZero;
    }
    if (val1 == 1 && val2 == 1){
      next_val = OneOne;
    }
    return next_val;
}



//// blinks the onboard LED for t milliseconds
//void BlinkLed(int t){
//  digitalWrite(LEDPin,HIGH);
//  delay(t);
//  digitalWrite(LEDPin,LOW);
//}


// swaps 2 values in the digital_input_pins array
// used for button remapping
void swap_buttons(int ind1, int ind2){
  int temp = digital_input_pins[ind1];
  digital_input_pins[ind1] = digital_input_pins[ind2];
  digital_input_pins[ind2] = temp;
}






int charToInt(char c){
  int ret = c-'0';
  return ret;
}


// checks the serial buffer, if data available read it into buffer_holder until reach stop bit (63)
// parse the message and respond accordingly
// A timer is used to ensure a faster reply to the poll request, firing the timer every 115 us after the first byte of the poll request is available
void communications(){
  if (Serial2.available()>0) {
    if(timer_flag == 0){
//      digitalWrite(LEDPin,HIGH);
      timerWrite(My_timer,0);
      timerAlarmEnable(My_timer);
      timer_flag = 1;
    }
    readVal = Serial2.read();
//    Serial.println(readVal);
    if(buff_index >= buffer_holder_len){
      buff_index = 0;
    }
    buffer_holder[buff_index] = readVal;
    buff_index++;

    if(triggered == 1){
//      digitalWrite(LEDPin, LOW);
      cleanUpBufferRead(buff_index);
      choose_and_reply();
      Clear_UART_Rx();
      buff_index = 0;
      triggered = 0;
      timer_flag = 0;
    }

    if (readVal == Stop){
      timerAlarmDisable(My_timer);
      cleanUpBufferRead(buff_index);
      choose_and_reply();
      buff_index = 0; 
      timer_flag = 0;
    }
  }
}

// switch case statement that compares the command byte to known commands
// based on the command bytes value a different message is sent
// the uart buffer is then cleared as to not have a read of the sent data occur
void choose_and_reply(){
  to_int();
  switch(command_byte){
    case 0b00000000:
      if(stop_bit_9){
        writeData(ProbeReply,ProbeReplyLen);
        num_probes++;
        Serial2.flush();
        Clear_UART_Rx();
      }
      break;
    case 0b01000001:
      if(stop_bit_9){
        writeData(InGameReply,InGameReplyLen-1);
        writeData(OriginEnd,OriginEndLen);
        num_origins++;
        Serial2.flush();
        Clear_UART_Rx();
      }
      break;
    case 0b01000000:
        writeData(InGameReply,InGameReplyLen);
        timer_flag = 0;
        num_polls++;
        Serial2.flush();
        Clear_UART_Rx();
      break;
    default:
      num_misc++;
      break;
  }
}

// converts the first 8 GameCube bits of data to a command byte
// also sets the stop_bit_9 variable which tells us if the 9th GameCube bit was a stop bit or not
void to_int(){
  int j = 0;
  for(int i=0;i<4;i++){
    j = i*2;
    switch(buffer_holder[i]){
      case ZeroZero:
        bitWrite(command_byte,8-j-1,0);
        bitWrite(command_byte,8-j-2,0);
        break;
      case ZeroOne:
        bitWrite(command_byte,8-j-1,0);
        bitWrite(command_byte,8-j-2,1);
        break;
      case OneZero:
        bitWrite(command_byte,8-j-1,1);
        bitWrite(command_byte,8-j-2,0);
        break;
      case OneOne:
        bitWrite(command_byte,8-j-1,1);
        bitWrite(command_byte,8-j-2,1);
        break;
      default:
        break;   
    }
  }
  if(buffer_holder[4]==Stop){
    stop_bit_9 = 1;
  }
  else{
    stop_bit_9 = 0;
  }
}


void BLEHandler(){
  if (deviceConnected) {
    receivedMSG = (String) Ch2.getValue().c_str();
//    Serial.println(receivedMSG);
    char fifthChar = receivedMSG[4];
    char fourthChar = receivedMSG[3];
    char thirdChar = receivedMSG[2];
    int isX = !digitalRead(18);
    int isY = !digitalRead(4);
    char firstChar = receivedMSG[0];
    
//    Serial.println(fifthChar == ',');
//    Serial.println();
    if(resetPasswordFlag == 1){
      if(firstChar == 'P' && passWriteFlag == 0){
//        Serial.println(receivedMSG.substring(1));
        writeBLEpasswordToMem(receivedMSG.substring(1));
        passWriteFlag = 1;
        resetPasswordFlag = 0;
        Ch1.setValue("Password Reset");
        Ch1.notify();
      }
      else{
        Ch1.setValue("Reset Password");
        Ch1.notify();
      }
    }
    else{
      if(password_correct==0){
        if((String) Ch2.getValue().c_str() == BLEpassword && isX && isY){
          password_correct = 1;
          Ch1.setValue("Password Correct");
          Ch1.notify();
        }
        else{
          Ch1.setValue("Password Incorrect");
          Ch1.notify();
        }
      }
      else{
        if(receivedMSG == "A"){ // A means requesting Analog data
          sprintf(AnalogMSG, "%04d,%04d,%04d,%04d,%04d,%04d", aX, aY, cX, cY, aL, aR);
          Ch1.setValue(AnalogMSG);
          Ch1.notify();
        }
        else{
          if(fifthChar == ','){
    //        Serial.println("Parsing");
            ParseCalibrationString(receivedMSG);
    //        Serial.println("Done Parsing");
          }
          else{
            if(receivedMSG == "SAC" && savedCalib == 0){ // SAC means Save Analog Calibration Values
    //          Serial.println("Saving");
              writeStickCalToMem();
              writeStickDeadzonesToMem();
              savedCalib = 1;
    //          Serial.println("Done Saving");
            }
            else{
              if(receivedMSG =="RAC"){
                sprintf(AnalogCalibMSG,"%04d,%04d,%04d:%04d,%04d,%04d:%04d,%04d,%04d:%04d,%04d,%04d", AXNeutch, AXLow, AXHigh, AYNeutch, AYLow, AYHigh, CXNeutch, CXLow, CXHigh, CYNeutch, CYLow, CYHigh);
                Ch1.setValue(AnalogCalibMSG);
                Ch1.notify();
              }
              else{
                if(fourthChar == ','){
                  ParseDeadzoneString(receivedMSG);
                }
                else{
                  if(receivedMSG =="SSD" && savedDeadzones == 0){// SSD = Save Stick Deadzones
                    writeStickDeadzonesToMem();
    //                Serial.println("Would be saving deadzones");
                    savedDeadzones = 1;
                  }
                  else{
                    if(thirdChar == '.'){
                      ParseButtonMappingString(receivedMSG);
                    }
                    else{
                      if(receivedMSG == "RBM"){
                        fillDigitalMappingMessage();
                        Ch1.setValue(DigitalMappingMSG);
                        Ch1.notify();
                      }
                      else{
                        if(receivedMSG == "SBM" && savedButtonMapping == 0){
                          writeButtonMappingToMem();
    //                      Serial.println("Would be saving button mapping");
                          savedButtonMapping = 1;
                        }
                        else{
                          if(firstChar == 'W' && wifi_flag == 0){
                            wifiUploadEnabled(receivedMSG);
                            Ch1.setValue(ipAddy);
                            Ch1.notify();
                          }
                          else{
                            if(firstChar == 'P' && passWriteFlag == 0){
//                              Serial.println(receivedMSG.substring(1));
                              writeBLEpasswordToMem(receivedMSG.substring(1));
                              passWriteFlag = 1;
                            }
                            else{
                              if(receivedMSG == "RDC"){
                                sprintf(AnalogDeadzoneMSG,"%03d,%03d,%03d:%03d,%03d,%03d:%03d,%03d,%03d:%03d,%03d,%03d", AXDeadzone[0], AXDeadzone[1], AXDeadzone[2], AYDeadzone[0], AYDeadzone[1], AYDeadzone[2], CXDeadzone[0], CXDeadzone[1], CXDeadzone[2], CYDeadzone[0], CYDeadzone[1], CYDeadzone[2]);
                                Ch1.setValue(AnalogDeadzoneMSG);
                                Ch1.notify();
                              }
                            }
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
}

void ParseCalibrationString(String str){
  AXNeutch = str.substring(0,4).toInt();
  AXLow = str.substring(5,9).toInt();
  AXHigh = str.substring(10,14).toInt();

  AYNeutch = str.substring(15,19).toInt();
  AYLow = str.substring(20,24).toInt();
  AYHigh = str.substring(25,29).toInt();

  CXNeutch = str.substring(30,34).toInt();
  CXLow = str.substring(35,39).toInt();
  CXHigh = str.substring(40,44).toInt();

  CYNeutch = str.substring(45,49).toInt();
  CYLow = str.substring(50,54).toInt();
  CYHigh = str.substring(55,59).toInt();
}

// writes the current stick calibration values to memory
void writeStickCalToMem(){
  preferences.begin("AnalogCal", false);
  uint16_t toSave;
  
  toSave = AXNeutch;
  preferences.putUShort("AXNeutch", toSave);
  toSave = AXHigh;
  preferences.putUShort("AXHigh", toSave);
  toSave = AXLow;
  preferences.putUShort("AXLow", toSave);

  toSave = AYNeutch;
  preferences.putUShort("AYNeutch", toSave);
  toSave = AYHigh;
  preferences.putUShort("AYHigh", toSave);
  toSave = AYLow;
  preferences.putUShort("AYLow", toSave);

  toSave = CXNeutch;
  preferences.putUShort("CXNeutch", toSave);
  toSave = CXHigh;
  preferences.putUShort("CXHigh", toSave);
  toSave = CXLow;
  preferences.putUShort("CXLow", toSave);

  toSave = CYNeutch;
  preferences.putUShort("CYNeutch", toSave);
  toSave = CYHigh;
  preferences.putUShort("CYHigh", toSave);
  toSave = CYLow;
  preferences.putUShort("CYLow", toSave);
  
  preferences.end();
}

// reads the stick halibration values from memory
void readStickCalFromMem(){
  preferences.begin("AnalogCal", true);
  
  AXNeutch = preferences.getUShort("AXNeutch",0); // if key not there default to 0
  AXHigh = preferences.getUShort("AXHigh",0);
  AXLow = preferences.getUShort("AXLow",0);

  AYNeutch = preferences.getUShort("AYNeutch",0);
  AYHigh = preferences.getUShort("AYHigh",0);
  AYLow = preferences.getUShort("AYLow",0);

  CXNeutch = preferences.getUShort("CXNeutch",0);
  CXHigh = preferences.getUShort("CXHigh",0);
  CXLow = preferences.getUShort("CXLow",0);

  CYNeutch = preferences.getUShort("CYNeutch",0);
  CYHigh = preferences.getUShort("CYHigh",0);
  CYLow = preferences.getUShort("CYLow",0);
  
  preferences.end();
}

int mapStickVals(int low,int high,int neutch,int value, int dead[]){
    int mapped = 127;
    double lowS;
    double highS;
    int highSGood = 0; // for checking division by 0
    int lowSGood = 0; // for checking division by 0
    double val = (double) value;

    // make the slopes
    if((high-neutch)!=0){
      highS = 127.0/((double)high-(double)neutch);
      highSGood = 1;
    }
    if((neutch-low)!=0){
      lowS = 127.0/((double)neutch-(double)low);
      lowSGood = 1;
    }

    // map the value onto the line
    if(val <= neutch && lowSGood == 1){
        mapped = (int) (lowS*val-lowS*neutch+127.0);
    }
    if(val > neutch && highSGood == 1){
        mapped = (int) (highS*val-highS*neutch+127.0);
    }

    // check if the mapped value exceeds the bounds
    if(mapped<0){
        return 0;
    }
    if(mapped > 255){
        return 255;
    }
    
    if(mapped >= dead[0] && mapped <= dead[1]){
      mapped = dead[2];
    }
    
    return mapped;
}

int mapTriggerVals(int high, int low, int value){
  int mapped = -1;
  double val = (double) value;
  mapped = val/255.0*(high-low)+low;
  if(mapped <0){
    return 0;
  }
  if(mapped>255){
    return 255;
  }
  return mapped;
}

void ParseDeadzoneString(String str){
  AXDeadzone[0] = str.substring(0,3).toInt();
  AXDeadzone[1] = str.substring(4,7).toInt();
  AXDeadzone[2] = str.substring(8,11).toInt();

  AYDeadzone[0] = str.substring(12,15).toInt();
  AYDeadzone[1] = str.substring(16,19).toInt();
  AYDeadzone[2] = str.substring(20,23).toInt();

  CXDeadzone[0] = str.substring(24,27).toInt();
  CXDeadzone[1] = str.substring(28,31).toInt();
  CXDeadzone[2] = str.substring(32,35).toInt();

  CYDeadzone[0] = str.substring(36,39).toInt();
  CYDeadzone[1] = str.substring(40,43).toInt();
  CYDeadzone[2] = str.substring(44,47).toInt();

//  Serial.print("AX Deadzone = ");
//  Serial.print(AXDeadzone[0]);
//  Serial.print(", ");
//  Serial.print(AXDeadzone[1]);
//  Serial.print(", ");
//  Serial.print(AXDeadzone[2]);
//  Serial.println();
//  
//  Serial.print("AY Deadzone = ");
//  Serial.print(AYDeadzone[0]);
//  Serial.print(", ");
//  Serial.print(AYDeadzone[1]);
//  Serial.print(", ");
//  Serial.print(AYDeadzone[2]);
//  Serial.println();
//
//  Serial.print("CX Deadzone = ");
//  Serial.print(CXDeadzone[0]);
//  Serial.print(", ");
//  Serial.print(CXDeadzone[1]);
//  Serial.print(", ");
//  Serial.print(CXDeadzone[2]);
//  Serial.println();
//  
//  Serial.print("CY Deadzone = ");
//  Serial.print(CYDeadzone[0]);
//  Serial.print(", ");
//  Serial.print(CYDeadzone[1]);
//  Serial.print(", ");
//  Serial.print(CYDeadzone[2]);
//  Serial.println();
//  Serial.println();
  
}

// writes the current stick calibration values to memory
void writeStickDeadzonesToMem(){
  preferences.begin("AnalogDead", false);
  uint16_t toSave;
  
  toSave = AXDeadzone[0];
  preferences.putUShort("AXDeadLow", toSave);
  toSave = AXDeadzone[1];
  preferences.putUShort("AXDeadHigh", toSave);
  toSave = AXDeadzone[2];
  preferences.putUShort("AXDeadVal", toSave);

  toSave = AYDeadzone[0];
  preferences.putUShort("AYDeadLow", toSave);
  toSave = AYDeadzone[1];
  preferences.putUShort("AYDeadHigh", toSave);
  toSave = AYDeadzone[2];
  preferences.putUShort("AYDeadVal", toSave);

  toSave = CXDeadzone[0];
  preferences.putUShort("CXDeadLow", toSave);
  toSave = CXDeadzone[1];
  preferences.putUShort("CXDeadHigh", toSave);
  toSave = CXDeadzone[2];
  preferences.putUShort("CXDeadVal", toSave);

  toSave = CYDeadzone[0];
  preferences.putUShort("CYDeadLow", toSave);
  toSave = CYDeadzone[1];
  preferences.putUShort("CYDeadHigh", toSave);
  toSave = CYDeadzone[2];
  preferences.putUShort("CYDeadVal", toSave);
  
  preferences.end();
}

void readStickDeadzonesFromMem(){
  preferences.begin("AnalogDead", true);
  
  AXDeadzone[0] = preferences.getUShort("AXDeadLow",117); 
  AXDeadzone[1] = preferences.getUShort("AXDeadHigh",137);
  AXDeadzone[2] = preferences.getUShort("AXDeadVal",127); 

  AYDeadzone[0] = preferences.getUShort("AYDeadLow",117); 
  AYDeadzone[1] = preferences.getUShort("AYDeadHigh",137);
  AYDeadzone[2] = preferences.getUShort("AYDeadVal",127); 

  CXDeadzone[0] = preferences.getUShort("CXDeadLow",117); 
  CXDeadzone[1] = preferences.getUShort("CXDeadHigh",137);
  CXDeadzone[2] = preferences.getUShort("CXDeadVal",127); 

  CYDeadzone[0] = preferences.getUShort("CYDeadLow",117); 
  CYDeadzone[1] = preferences.getUShort("CYDeadHigh",137);
  CYDeadzone[2] = preferences.getUShort("CYDeadVal",127); 
  
  preferences.end();
}

void ParseButtonMappingString(String str){
  int pin;
  int toggle;
  String read_val_1;
  String read_val_2;

  read_val_1 = str.substring(0,1);
  read_val_2 = str.substring(1,2);
  pin = getPinFromChar(read_val_1);
  toggle = getToggleFromChar(read_val_2);
  digital_mapping[7] = pin;
  digital_toggling[7] = toggle;

  read_val_1 = str.substring(3,4);
  read_val_2 = str.substring(4,5);
  pin = getPinFromChar(read_val_1);
  toggle = getToggleFromChar(read_val_2);
  digital_mapping[6] = pin;
  digital_toggling[6] = toggle;

  read_val_1 = str.substring(6,7);
  read_val_2 = str.substring(7,8);
  pin = getPinFromChar(read_val_1);
  toggle = getToggleFromChar(read_val_2);
  digital_mapping[3] = pin;
  digital_toggling[3] = toggle;

  read_val_1 = str.substring(9,10);
  read_val_2 = str.substring(10,11);
  pin = getPinFromChar(read_val_1);
  toggle = getToggleFromChar(read_val_2);
  digital_mapping[5] = pin;
  digital_toggling[5] = toggle;

  read_val_1 = str.substring(12,13);
  read_val_2 = str.substring(13,14);
  pin = getPinFromChar(read_val_1);
  toggle = getToggleFromChar(read_val_2);
  digital_mapping[4] = pin;
  digital_toggling[4] = toggle;

  read_val_1 = str.substring(15,16);
  read_val_2 = str.substring(16,17);
  pin = getPinFromChar(read_val_1);
  toggle = getToggleFromChar(read_val_2);
  digital_mapping[11] = pin;
  digital_toggling[11] = toggle;

  read_val_1 = str.substring(18,19);
  read_val_2 = str.substring(19,20);
  pin = getPinFromChar(read_val_1);
  toggle = getToggleFromChar(read_val_2);
  digital_mapping[9] = pin;
  digital_toggling[9] = toggle;

  read_val_1 = str.substring(21,22);
  read_val_2 = str.substring(22,23);
  pin = getPinFromChar(read_val_1);
  toggle = getToggleFromChar(read_val_2);
  digital_mapping[10] = pin;
  digital_toggling[10] = toggle;

  read_val_1 = str.substring(24,25);
  read_val_2 = str.substring(25,26);
  pin = getPinFromChar(read_val_1);
  toggle = getToggleFromChar(read_val_2);
  digital_mapping[12] = pin;
  digital_toggling[12] = toggle;

  read_val_1 = str.substring(27,28);
  read_val_2 = str.substring(28,29);
  pin = getPinFromChar(read_val_1);
  toggle = getToggleFromChar(read_val_2);
  digital_mapping[14] = pin;
  digital_toggling[14] = toggle;

  read_val_1 = str.substring(30,31);
  read_val_2 = str.substring(31,32);
  pin = getPinFromChar(read_val_1);
  toggle = getToggleFromChar(read_val_2);
  digital_mapping[13] = pin;
  digital_toggling[13] = toggle;

  read_val_1 = str.substring(33,34);
  read_val_2 = str.substring(34,35);
  pin = getPinFromChar(read_val_1);
  toggle = getToggleFromChar(read_val_2);
  digital_mapping[15] = pin;
  digital_toggling[15] = toggle;

  updateDigitalInputPins();
}

int getPinFromChar(String char_read){
  int pin = -1;

  if(char_read == "A"){
    pin = 22;
  }
  else if(char_read == "B"){
    pin = 23;
  }
  else if(char_read == "S"){
    pin = 21;
  }
  else if(char_read == "X"){
    pin = 18;
  }
  else if(char_read == "Y"){
    pin = 4;
  }
  else if(char_read == "Z"){
    pin = 15;
  }
  else if(char_read == "L"){
    pin = 5;
  }
  else if(char_read == "R"){
    pin = 19;
  }
  else if(char_read == "u"){
    pin = 32;
  }
  else if(char_read == "r"){
    pin = 33;
  }
  else if(char_read == "d"){
    pin = 25;
  }
  else if(char_read == "l"){
    pin = 14;
  }
  return pin;
}

int getToggleFromChar(String char_read){
  int toggle = 1;

  if(char_read == "Y"){
    toggle = 1;
  }
  else if(char_read == "N"){
    toggle = 0;
  }
  
  return toggle;
}

void updateDigitalInputPins(){
  for(int i=0; i< DigitalInLen; i++){
    if(digital_toggling[i]==0){
      digital_input_pins[i] = -1;
    }
    else{
      digital_input_pins[i] = digital_mapping[i];
    }
  }
}

void fillDigitalMappingMessage(){
  DigitalMappingMSG[0] = getCharFromPin(digital_mapping[7]);
  DigitalMappingMSG[1] = getCharFromToggle(digital_toggling[7]);
  DigitalMappingMSG[2] = '.';
  DigitalMappingMSG[3] = getCharFromPin(digital_mapping[6]);
  DigitalMappingMSG[4] = getCharFromToggle(digital_toggling[6]);
  DigitalMappingMSG[5] = '.';
  DigitalMappingMSG[6] = getCharFromPin(digital_mapping[3]);
  DigitalMappingMSG[7] = getCharFromToggle(digital_toggling[3]);
  DigitalMappingMSG[8] = '.';
  DigitalMappingMSG[9] = getCharFromPin(digital_mapping[5]);
  DigitalMappingMSG[10] = getCharFromToggle(digital_toggling[5]);
  DigitalMappingMSG[11] = '.';
  DigitalMappingMSG[12] = getCharFromPin(digital_mapping[4]);
  DigitalMappingMSG[13] = getCharFromToggle(digital_toggling[4]);
  DigitalMappingMSG[14] = '.';
  DigitalMappingMSG[15] = getCharFromPin(digital_mapping[11]);
  DigitalMappingMSG[16] = getCharFromToggle(digital_toggling[11]);
  DigitalMappingMSG[17] = '.';
  DigitalMappingMSG[18] = getCharFromPin(digital_mapping[9]);
  DigitalMappingMSG[19] = getCharFromToggle(digital_toggling[9]);
  DigitalMappingMSG[20] = '.';
  DigitalMappingMSG[21] = getCharFromPin(digital_mapping[10]);
  DigitalMappingMSG[22] = getCharFromToggle(digital_toggling[10]);
  DigitalMappingMSG[23] = '.';
  DigitalMappingMSG[24] = getCharFromPin(digital_mapping[12]);
  DigitalMappingMSG[25] = getCharFromToggle(digital_toggling[12]);
  DigitalMappingMSG[26] = '.';
  DigitalMappingMSG[27] = getCharFromPin(digital_mapping[14]);
  DigitalMappingMSG[28] = getCharFromToggle(digital_toggling[14]);
  DigitalMappingMSG[29] = '.';
  DigitalMappingMSG[30] = getCharFromPin(digital_mapping[13]);
  DigitalMappingMSG[31] = getCharFromToggle(digital_toggling[13]);
  DigitalMappingMSG[32] = '.';
  DigitalMappingMSG[33] = getCharFromPin(digital_mapping[15]);
  DigitalMappingMSG[34] = getCharFromToggle(digital_toggling[15]);
}

char getCharFromPin(int pin){
  char char_out;
  switch (pin){
    case 22:
      char_out = 'A';
      break;
    case 23:
      char_out = 'B';
      break;
    case 21:
      char_out = 'S';
      break;
    case 18:
      char_out = 'X';
      break;
    case 4:
      char_out = 'Y';
      break;
    case 15:
      char_out = 'Z';
      break;
    case 5:
      char_out = 'L';
      break;
    case 19:
      char_out = 'R';
      break;
    case 32:
      char_out = 'u';
      break;
    case 33:
      char_out = 'r';
      break;
    case 25:
      char_out = 'd';
      break;
    case 14:
      char_out = 'l';
      break;
    default:
        break;
  }
  return char_out;
}

char getCharFromToggle(int toggle){
  char char_out;
  switch (toggle){
    case 1:
      char_out = 'Y';
      break;
    case 0:
      char_out = 'N';
      break;
    default:
        break;
  }
  return char_out;
}

void writeButtonMappingToMem(){
  preferences.begin("DigitalMap", false);
  uint16_t toSave;
  
  toSave = digital_mapping[7];
  preferences.putUShort("A", toSave);
  toSave = digital_toggling[7];
  preferences.putUShort("AT", toSave);

  toSave = digital_mapping[6];
  preferences.putUShort("B", toSave);
  toSave = digital_toggling[6];
  preferences.putUShort("BT", toSave);

  toSave = digital_mapping[3];
  preferences.putUShort("S", toSave);
  toSave = digital_toggling[3];
  preferences.putUShort("ST", toSave);

  toSave = digital_mapping[5];
  preferences.putUShort("X", toSave);
  toSave = digital_toggling[5];
  preferences.putUShort("XT", toSave);

  toSave = digital_mapping[4];
  preferences.putUShort("Y", toSave);
  toSave = digital_toggling[4];
  preferences.putUShort("YT", toSave);

  toSave = digital_mapping[11];
  preferences.putUShort("Z", toSave);
  toSave = digital_toggling[11];
  preferences.putUShort("ZT", toSave);

  toSave = digital_mapping[9];
  preferences.putUShort("L", toSave);
  toSave = digital_toggling[9];
  preferences.putUShort("LT", toSave);

  toSave = digital_mapping[10];
  preferences.putUShort("R", toSave);
  toSave = digital_toggling[10];
  preferences.putUShort("RT", toSave);

  toSave = digital_mapping[12];
  preferences.putUShort("u", toSave);
  toSave = digital_toggling[12];
  preferences.putUShort("uT", toSave);

  toSave = digital_mapping[14];
  preferences.putUShort("r", toSave);
  toSave = digital_toggling[14];
  preferences.putUShort("rT", toSave);

  toSave = digital_mapping[13];
  preferences.putUShort("d", toSave);
  toSave = digital_toggling[13];
  preferences.putUShort("dT", toSave);

  toSave = digital_mapping[15];
  preferences.putUShort("l", toSave);
  toSave = digital_toggling[15];
  preferences.putUShort("lT", toSave);

  preferences.end();
}

// 0 0 0 Start Y X B A 1 L R Z D-U D-D D-R D-L
//{-1,-1,-1,21,4,18,23,22,-1,5,19,15,32,25,33,14};
void readButtonMappingFromMem(){
  preferences.begin("DigitalMap", true);

  digital_mapping[7] = preferences.getUShort("A",22);
  digital_toggling[7] = preferences.getUShort("AT",1);

  digital_mapping[6] = preferences.getUShort("B",23);
  digital_toggling[6] = preferences.getUShort("BT",1);

  digital_mapping[3] = preferences.getUShort("S",21);
  digital_toggling[3] = preferences.getUShort("ST",1);

  digital_mapping[5] = preferences.getUShort("X",18);
  digital_toggling[5] = preferences.getUShort("XT",1);

  digital_mapping[4] = preferences.getUShort("Y",4);
  digital_toggling[4] = preferences.getUShort("YT",1);

  digital_mapping[11] = preferences.getUShort("Z",15);
  digital_toggling[11] = preferences.getUShort("ZT",1);

  digital_mapping[9] = preferences.getUShort("L",5);
  digital_toggling[9] = preferences.getUShort("LT",1);

  digital_mapping[10] = preferences.getUShort("R",19);
  digital_toggling[10] = preferences.getUShort("RT",1);

  digital_mapping[12] = preferences.getUShort("u",32);
  digital_toggling[12] = preferences.getUShort("uT",1);

  digital_mapping[14] = preferences.getUShort("r",33);
  digital_toggling[14] = preferences.getUShort("rT",1);

  digital_mapping[13] = preferences.getUShort("d",25);
  digital_toggling[13] = preferences.getUShort("dT",1);

  digital_mapping[15] = preferences.getUShort("l",14);
  digital_toggling[15] = preferences.getUShort("lT",1);
  
  preferences.end();

  updateDigitalInputPins();
  
}

void wifiUploadEnabled(String Message){
  char *token;
  char *mystring = (char*) Message.c_str();
  const char *delimiter ="/";
  int i = 0;
  char* ssid;
  char* password;
  ipAddy[0] = 'N';
  ipAddy[1] = 'O';
  
  token = strtok(mystring, delimiter);
  while (token != NULL) {
  //      Serial.println(token);
    switch(i){
      case 1:
        ssid = token;
        break;
      case 2:
        password = token;
        break;
    }
    token=strtok(NULL, delimiter);
    i++;
  }
   
  Serial.print("ssid = ");
  Serial.println(ssid);
  Serial.print("password = ");
  Serial.println(password);
  Serial.println();
  // Start Wifi setup
  // Connect to WiFi network
  WiFi.begin(ssid, password);
  Serial.println("");

  i = 0;
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    i++;
    if(i>=100){
      break;
    }
  }
  
  if(i<100){
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  
    /*use mdns for host name resolution*/
    if (!MDNS.begin(host)) { //http://esp32.local
      Serial.println("Error setting up MDNS responder!");
      while (1) {
        delay(1000);
      }
    }
    Serial.println("mDNS responder started");
    /*return index page which is stored in serverIndex */
    server.on("/", HTTP_GET, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/html", loginIndex);
    });
    server.on("/serverIndex", HTTP_GET, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/html", serverIndex);
    });
    /*handling uploading firmware file */
    server.on("/update", HTTP_POST, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
      ESP.restart();
    }, []() {
      HTTPUpload& upload = server.upload();
      if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("Update: %s\n", upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        /* flashing firmware to ESP*/
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) { //true to set the size to the current progress
          Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        } else {
          Update.printError(Serial);
        }
      }
    });
    server.begin();
    wifi_flag = 1;
    WiFi.localIP().toString().toCharArray(ipAddy,15);
  }
  
  // End Wifi Setup
}


void writeBLEpasswordToMem(String newPass){
  preferences.begin("Passwords", false);
  preferences.putString("BLEPass", newPass);
  preferences.end();
  BLEpassword = newPass;
}

void readBLEpasswordFromMem(){
  preferences.begin("Passwords", true);
  BLEpassword = preferences.getString("BLEPass","KEG CONCH");
  preferences.end();
}

// checkst the physical pins (5,19,22,21) which map to physical buttons (L,R,A,Start) respectively
// if they are all high the sent message is changed to be only those 4 buttons clicked
void checkForLRAStart(){
  int LRAStart = 0;
  LRAStart+= !digitalRead(5);
  LRAStart+= !digitalRead(19);
  LRAStart+= !digitalRead(22);
  LRAStart+= !digitalRead(21);
  if (LRAStart == 4){ // if they were all pressed change buttons_in array to be exactly those 4 buttons pushed
    for(int i=0;i<DigitalInLen;i++){
      if(i==3 || i==7 || i==9 || i==10 || i==8){
        buttons_in[i] = 1;
      }
      else{
        buttons_in[i] = 0;
      }
    }
  }
}


