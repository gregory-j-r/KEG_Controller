#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include <EEPROM.h>

// define the number of bytes you want to access
#define EEPROM_SIZE 22

#define ZeroZero 4
#define OneOne 55
#define ZeroOne 52
#define OneZero 7
#define Stop 63

#define ProbeReplyLen 13
#define OriginReplyLen 41
#define InGameReplyLen 33

#define ProbeLen 5
#define OriginLen 5
#define PollLen 13

#define DigitalInLen 16
#define AnalogInLen 6


#define RXD2 16
#define TXD2 17
#define buffer_holder_len 45

#define LEDPin 2

int prog_t1 = 0;
int prog_t2 = 0;
int program_flag = 0;

int dump = 0;

int buffer_holder[buffer_holder_len];
int readVal = 0;
int buff_index = 0;

int probe1[] = {ZeroZero,ZeroZero,ZeroZero,ZeroZero,Stop};
int probe2[] = {OneOne,OneOne,OneOne,OneOne,Stop};

int origin1[] = {ZeroOne,ZeroZero,ZeroZero,ZeroOne,Stop};
int origin2[] = {ZeroOne,ZeroZero,ZeroZero,OneZero,Stop};

int ProbeReply[ProbeReplyLen] = {ZeroZero, ZeroZero, OneZero, ZeroOne, ZeroZero, ZeroZero, ZeroZero, ZeroZero, ZeroZero, ZeroZero, ZeroZero, OneOne,Stop};
int OriginReply[OriginReplyLen] = {ZeroZero, ZeroZero, ZeroZero, ZeroZero, // 0 0 0 Start Y X B A
                                       OneZero, ZeroZero, ZeroZero, ZeroZero, // 1 L R Z D-U D-D D-R D-L
                                       OneZero, ZeroZero, ZeroZero, ZeroZero, // Analog X
                                       OneZero, ZeroZero, ZeroZero, ZeroZero, // Analog Y
                                       OneZero, ZeroZero, ZeroZero, ZeroZero, // C-Stick X
                                       OneZero, ZeroZero, ZeroZero, ZeroZero, // C-Stick Y
                                       ZeroZero, ZeroZero, ZeroZero, ZeroZero, // L-Trigger
                                       ZeroZero, ZeroZero, ZeroZero, ZeroZero, // R-Trigger
                                       ZeroZero, ZeroZero, ZeroZero, ZeroZero, // Null
                                       ZeroZero, ZeroZero, ZeroZero, ZeroZero, // Null
                                       Stop};                                                  // Note: Will need to change this as well as InGameReply Declaration once full controller assembled to poll for the initial values

int InGameReply[InGameReplyLen] = {ZeroZero, ZeroZero, ZeroZero, ZeroZero, // 0 0 0 Start Y X B A
                                       OneZero, ZeroZero, ZeroZero, ZeroZero, // 1 L R Z D-U D-D D-R D-L
                                       OneZero, ZeroZero, ZeroZero, ZeroZero, // Analog X
                                       OneZero, ZeroZero, ZeroZero, ZeroZero, // Analog Y
                                       OneZero, ZeroZero, ZeroZero, ZeroZero, // C-Stick X
                                       OneZero, ZeroZero, ZeroZero, ZeroZero, // C-Stick Y
                                       ZeroZero, ZeroZero, ZeroZero, ZeroZero, // L-Trigger
                                       ZeroZero, ZeroZero, ZeroZero, ZeroZero, // R-Trigger
                                       Stop};

int buttons_in[DigitalInLen] = {0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0}; // 0 0 0 Start Y X B A 1 L R Z D-U D-D D-R D-L
int analogs_in[AnalogInLen] = {128,128,128,128,0,0}; // Analog-X Analog-Y C-Stick-X C-Stick-Y L-Trigger R-Trigger

int digital_input_pins[DigitalInLen] = {-1,-1,-1,21,22,23,19,18,-1,5,4,15,32,33,25,14}; // should save pin 2 to use for rumble (it's connected to the onboard LED), pin 2 on the devboard is attatched to a pull down resistor, Note gpio5 is connected to a pull up resistor
int analog_input_pins[AnalogInLen] = {39,13,34,35,26,27}; // maybe change 12 for 36 (aka vp) or 39 (aka vn) as 12 needs to be low during boot but currently is not??

// There are 512 memory adresses each able to store 8 bits
int digital_pin_mem_locs[DigitalInLen] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
int analog_pin_mem_locs[AnalogInLen] = {16,17,18,19,20,21};

int read_counter = 0;
int loop_counter = 0;

int timer_arr_1[] = {0,0};

TaskHandle_t ReadInputs;

void setup() {
  fill_buff_hold();

  for(int i = 0; i<DigitalInLen; i++){
    if (digital_input_pins[i] !=  -1){
      pinMode(digital_input_pins[i],INPUT_PULLUP);
    }
  }

  EEPROM.begin(EEPROM_SIZE);
//  Save_Buttons_To_Memory(); // Should be run 1 time i.e. first upload then should be removed
  Read_Buttons_From_Memory();

  analogReadResolution(9);
  analogSetWidth(9);

  pinMode(LEDPin,OUTPUT);
  
  Serial.begin(9600);
  Serial2.begin(800000, SERIAL_6N1, RXD2, TXD2);
  Serial2.setRxBufferSize(256);

  xTaskCreatePinnedToCore(read_inputs,"Read Inputs",10000,NULL,0,&ReadInputs,  /* Task handle. */0 /*core #*/);
}



void loop(){
  if (program_flag == 0){
    communications();
  }
  else{
      Program_Mode();
  }
}

// The task called on core 0
// Calls the function to update the input state arrays and delays as to not setoff a watchdog timeout
void read_inputs( void * parameter) {
  for(;;) {
    check_buttons();
    update_reply();
    
    program_flag = check_sequence(18, 19, 21, 3000, timer_arr_1);
    vTaskDelay(2);
//    read_counter++;
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
    if (i != 2 && i != 3){ // testing only, didnt have a c stick to connect
      analogs_in[i] = analogRead(analog_input_pins[i])/2;
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
  for (int i = 0; i<DigitalInLen; i=i+2){
    j = i+1;
    val1 = buttons_in[i];
    val2 = buttons_in[j];
    next_val = TwoBitsToMessage(val1, val2);
    InGameReply[count] = next_val;
    count++;
  }
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
}

// checks the serial buffer, if data available read it into buffer_holder until reach stop bit (63)
// parse the message and respond accordingly
void communications(){
  if (Serial2.available()>0) {
    if(buff_index >= buffer_holder_len){
      buff_index = 0;
    }
    readVal = Serial2.read();
    buffer_holder[buff_index] = readVal;
    buff_index++;
    
    if (readVal == 63){
      cleanUpBufferRead(buff_index);

      if(buff_index == ProbeLen){
        if(compare_arrays(probe1,buffer_holder,ProbeLen)==1 || compare_arrays(probe2,buffer_holder,ProbeLen)==1){
          writeData(ProbeReply,ProbeReplyLen);
        }
        else{
          if(compare_arrays(origin1,buffer_holder,OriginLen)==1 || compare_arrays(origin2,buffer_holder,OriginLen)==1){
            writeData(OriginReply,OriginReplyLen);
          }
//          else{
//            Serial.println("A");
//          }
        }
      }
      
      else{
        if (buff_index == PollLen){
  //        if (buffer_holder[0] == ZeroOne){ // did not solve disconnect issue
          if(compare_arrays(ProbeReply,buffer_holder,ProbeReplyLen)==0){// check to make sure it's not the probe reply we send as it's the same length as the poll request
//            update_reply();
//            loop_counter++;
//            if (loop_counter == 500){
//              Serial.println(read_counter);
//              read_counter = 0;
//              loop_counter = 0;
//            }
            writeData(InGameReply,InGameReplyLen);
           }
//           else{
//            Serial.println("B");
//           }
        }
      }
      
      buff_index = 0;
    }
  }
}



// writes data over 6 bit uart line serial 2
void writeData(int dat[], int len){
  for (int i = 0; i<len; i++){
    Serial2.write(dat[i]);
  }
}


// initialize the buffer holder with all 0's
void fill_buff_hold(){
  for(int i =0; i<buffer_holder_len; i++){
    buffer_holder[i] = 0;
  }
}

// compares 2 arrays
// returns 1 if they are equvelent and 0 if not
int compare_arrays(int in1[], int in2[], int len){
  for (int i = 0; i<len; i++){
    if (in1[i] != in2[i]){
      return 0;
    }
  }
  return 1;
}

// read all values in the input buffer into a dump variable
void Clear_UART_Rx(int len){
  int i = 0;
  while (i < len){
    if (Serial2.available()>0){
      dump = Serial2.read();
      i++;
    }  
  }
}

// prints the contents of array arr to the serial monitor
void printArr(int arr[], int len){
  for (int i=0; i<len; i++){
    Serial.println(arr[i]);
  }
}

// get a strange insertion of -1 into some of the read data
// this function looks through the buffer holder and removes any single -1 that is present
// it then shifts the rest of the values into place
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

// Digital Button Remapping algorithm
// 
void Program_Mode(){
  BlinkLed(1000);
  int a = -1;
  int b = -1;
  int quit_flag = 0;
  int timer_arr_2[] = {0,0};
  program_flag = 0;
  while(program_flag == 0){
    if (sumArr(buttons_in, DigitalInLen) == 3){
      for (int k = 0; k<DigitalInLen; k++){
        if (digital_input_pins[k] != -1 && buttons_in[k] == 1){
          if (a == -1){
            a = k;
          }
          else{
            b = k;
          }
        }
      } 
      while(quit_flag == 0){
        quit_flag = check_sequence(digital_input_pins[a],digital_input_pins[b],-1,3000, timer_arr_2);
        if(quit_flag == 1){
          swap_buttons(a,b);
          BlinkLed(2000);
        }
        if(sumArr(buttons_in, DigitalInLen) != 3){
          quit_flag = 1;
        }
      }
    }
    quit_flag = 0;
    timer_arr_2[0] = 0;
    timer_arr_2[1] = 0;
    a = -1;
    b = -1;
  }
  Save_Buttons_To_Memory();
  program_flag = 0;
  BlinkLed(1000);
}

// blinks the onboard LED for t milliseconds
void BlinkLed(int t){
  digitalWrite(LEDPin,HIGH);
  delay(t);
  digitalWrite(LEDPin,LOW);
}

// swaps 2 values in the digital_input_pins array
// used for button remapping
void swap_buttons(int ind1, int ind2){
  int temp = digital_input_pins[ind1];
  digital_input_pins[ind1] = digital_input_pins[ind2];
  digital_input_pins[ind2] = temp;
}

// Save the current values of digital_inpu_pins[] to memory for button remapping saving
void Save_Buttons_To_Memory(){
  int addr = -1;
  int pin = -1;
  for(int i=0; i<DigitalInLen; i++){
    addr = digital_pin_mem_locs[i];
    pin = digital_input_pins[i];
    if(pin == -1){
      pin = 69;
    }
    EEPROM.write(addr,pin);
  }
  addr = -1;
  pin = -1;
  for(int i=0; i<AnalogInLen; i++){
    addr = analog_pin_mem_locs[i];
    pin = analog_input_pins[i];
    if(pin == -1){
      pin = 69;
    }
    EEPROM.write(addr,pin);
  }
  EEPROM.commit();
}

void Read_Buttons_From_Memory(){
  int addr = -1;
  int pin = -1;
  for(int i =0; i<DigitalInLen; i++){
    addr = digital_pin_mem_locs[i];
    pin = EEPROM.read(addr);
    if(pin == 69){
      pin = -1;
    }
    digital_input_pins[i] = pin;
  }
  addr = -1;
  pin = -1;
  for(int i =0; i<AnalogInLen; i++){
    addr = analog_pin_mem_locs[i];
    pin = EEPROM.read(addr);
    if(pin == 69){
      pin = -1;
    }
    analog_input_pins[i] = pin;
  }
}

// checks if 3 buttons are pushed simultaneously for t milliseconds
// returns 0 if the set of buttons has not been held down for at least t milliseconds and 1 if they have
// can be used for a 2 button sequence if -1 is passed for one of the button values
// an array, tArr, of 2 timers must also be passed
int check_sequence(int b1, int b2, int b3, int t, int tArr[]){ 
  int t1 = tArr[0];
  int t2 = tArr[1];
  int digiSumVal = 0;
  int pinNum = -1;
  int seq_flag = 0;
  for (int k = 0; k<DigitalInLen; k++){
    pinNum = digital_input_pins[k];
    if (pinNum == b1 || pinNum == b2 || pinNum == b3){
       digiSumVal += buttons_in[k];
    }
  }
  if (digiSumVal == 3){
    if (t1==0){
      t1 = millis();
    }
    else{
      t2 = millis();
    }
    if (t2-t1 >= t){
      seq_flag = 1;
      t1 = 0;
      t2 = 0;
    }
    else{
      seq_flag = 0;
    }
  }
  else{
    seq_flag = 0;
    t1 = 0;
    t2 = 0;
  }
  tArr[0] = t1;
  tArr[1] = t2;
  return seq_flag;
}

// checks to see if the integer val is inside the array arr of length len
// returns 1 if the value is in the array and 0 if not
int inArr(int arr[], int len, int val){
  for (int i =0; i<len; i++){
    if (arr[i] == val){
      return 1;
    }
  }
  return 0;
}

// returns the sum of all values within array arr of length len
int sumArr(int arr[], int len){
  int sum = 0;
  for (int i=0; i<len; i++){
    sum += arr[i];
  }
  return sum;
}



