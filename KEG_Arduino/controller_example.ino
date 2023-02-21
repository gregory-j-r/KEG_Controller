#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unordered_map>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "driver/adc.h"

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#include "rom/ets_sys.h"
#include "esp_timer.h"

#include <Preferences.h>

#define DigitalInLen 16
#define AnalogInLen 6

#define ProbeReplyLen 13
#define InGameReplyLen 33

#define BufferHolderLen 45

#define OriginEndLen 9

#define ButtonMappingMSGLen 35

#define ProbeLen 5
#define OriginLen 5
#define PollLen 13

#define LEDPin 2
#define RXD2 16
#define TXD2 17


// BLE setup
#define bleServerName "KEG_V1_4_G"
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_2 "da1e7d98-916b-11ed-a1eb-0242ac120002"

BLECharacteristic Ch1(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
BLEDescriptor Dp1(BLEUUID((uint16_t)0x2902));
BLECharacteristic Ch2(CHARACTERISTIC_UUID_2, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
BLEDescriptor Dp2(BLEUUID((uint16_t)0x2902));

char AnalogMSG[19] = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
char AnalogCalibMSG[59];
char AnalogDeadzoneMSG[47];
char DigitalMappingMSG[35];

String BLEpassword;
String receivedMSG;
bool bluetoothConnected = false;
int password_correct = 0;

// Setup callbacks onConnect and onDisconnect
class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        bluetoothConnected = true;
    };
    void onDisconnect(BLEServer *pServer)
    {
        bluetoothConnected = false;
        password_correct = 0; // reset for next connection
    }
};
// End of BLE Setup

// Wifi
int wifi_flag = 0;
WebServer server(80);
char ipAddy[16];
const char* host = "esp32";

// Start Login Index Page
const char *loginIndex =
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
const char *serverIndex =
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



///////////////////////  Utils //////////////////////

// Pointer to task ReadInputs running on core 0. For RTOS
TaskHandle_t ReadInputs;

// timer stuff
hw_timer_t *My_timer = NULL;
int timer_flag = 1;
int triggered = 0;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer()
{
	triggered = 1;
	//  digitalWrite(LEDPin, LOW);
}

// from boost (functional/hash):
// see http://www.boost.org/doc/libs/1_35_0/doc/html/hash/combine.html template
template <class T> inline void hash_combine(size_t &seed, T const &v) {
    seed ^= std::hash<T>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

struct pair_hash {
    template <class T1, class T2>
    size_t operator()(const std::pair<T1, T2> &p) const {
        size_t seed = 0;
        hash_combine(seed, p.first);
        hash_combine(seed, p.second);
        return seed;
    }
};

char getCharFromToggle(int toggle)
{
    char char_out;

    if (toggle == 1)
        {
            char_out = 'Y';
        }
    else if (toggle == 0)
        {
            char_out = 'N';
        }

    return char_out;
}

int getToggleFromChar(String char_read)
{
    int toggle = 1;

    if (char_read == "Y")
    {
        toggle = 1;
    }
    else if (char_read == "N")
    {
        toggle = 0;
    }

    return toggle;
}

int mapStickVals(int calVals[], int dead[], int value){
    int mapped = -1;
    double lowS;
    double highS;
    int highSGood = 0; // for checking division by 0
    int lowSGood = 0; // for checking division by 0
    double val = (double) value;

    int low = calVals[0];
    int neutch = calVals[1];
    int high = calVals[2];

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

// get a strange insertion of -1 into some of the read data
// this function looks through the buffer holder and removes any single -1 that is present
// it then shifts the rest of the values into place
// think the -1 is when it reads but there is no data available, which seems like it should never be possible but we'll keep that on the backburner
void cleanUpBufferRead(int (&buffer)[BufferHolderLen], int &buffer_index)
{
    int j = -1;
    for (int i = 0; i < buffer_index; i++)
    {
        if (buffer[i] == -1)
        {
            j = i;
            // Q: break here?
        }
    }
    if (j != -1)
    {
        for (int i = j; i < buffer_index - 1; i++)
        {
            buffer[i] = buffer[i + 1];
        }
        buffer_index -= 1;
    }
}

// read all values in the input buffer into a dump variable
void Clear_UART_Rx()
{
    int dump;
    while (true)
    {
        if (Serial2.available())
        {
            dump = Serial2.read();
            if (dump == 63) // NOTE: make this readable
            {
                break;
            }
        }
    }
}

// writes data over 6 bit uart line serial 2
// Absolutely crucial that this function have the portENTER_CRITICAL_ISR wrapping the serial writes
// this solved the "start" bug as it seems something was interupting the serial writes
void writeData(int dat[], int len)
{
    portENTER_CRITICAL_ISR(&timerMux);
    for (int i = 0; i < len; i++)
    {
        Serial2.write(dat[i]);
    }
    portEXIT_CRITICAL_ISR(&timerMux);
}

// prints the contents of array arr to the serial monitor
void printArr(int arr[], int len)
{
    for (int i = 0; i < len; i++)
    {
        Serial.println(arr[i]);
    }
}

// prints the contents of an char array arr to the serial monitor
void printCharArr(char arr[], int len)
{
    for (int i = 0; i < len; i++)
    {
        Serial.print(arr[i]);
    }
    Serial.println();
}

int charToInt(char c)
{
    int ret = c - '0';
    return ret;
}


/**
 * @brief This class defines the code to setup and run a KEG controller.
 *
 */
class KEGController
{

public:
    /**
     * @brief Initialize controller to run. Sets up bluetooth and serial communications, empties buffer, reads
     *        data from memory, configures adc channels
     */
    void setup()
    {
        setPinsToPullup(); // set the pins to input_pullup mode
        
        // probably don't need, already initialized to 0
        // fill_buff_hold();

        readStickCalFromMem();       // read the stick calibration values from memory
        readStickDeadzonesFromMem(); // read the stick deadzone values from memory
        readButtonMappingFromMem();  // read button mapping from memory
        readBLEpasswordFromMem();    // read BLE Password from memory


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
        if (!digitalRead(charToPinMap['x']) && !digitalRead(charToPinMap['y']))
        {
            resetPasswordFlag = 1;
        }

        setupBLE();

        pinMode(LEDPin, OUTPUT);

        My_timer = timerBegin(0, 80, true);
        timerAttachInterrupt(My_timer, &onTimer, true);
        timerAlarmWrite(My_timer, 115, false);

        Serial.begin(9600);

        xTaskCreatePinnedToCore(this->startTask, "Read Inputs", 10000, NULL, 0, &ReadInputs, /* Task handle. */ 0 /*core #*/);

        Serial2.begin(800000, SERIAL_6N1, RXD2, TXD2);
        Serial2.setRxBufferSize(256);
    };

    /**
     * @brief main controller loop
     */
    void loop()
    {
        if (Serial2.available() > 0)
        {
            if (timer_flag == 0)
            {
                timerWrite(My_timer, 0);
                timerAlarmEnable(My_timer);
                timer_flag = 1;
            }
            readVal = Serial2.read();

            if (buff_index >= BufferHolderLen)
            {
                buff_index = 0;
            }
            buffer_holder[buff_index] = readVal;
            buff_index++;

            if (triggered == 1)
            {
                cleanUpBufferRead(buffer_holder, buff_index);
                choose_and_reply();
                Clear_UART_Rx();
                buff_index = 0;
                triggered = 0;
                timer_flag = 0;
            }

            if (readVal == STOP)
            {
                timerAlarmDisable(My_timer);
                cleanUpBufferRead(buffer_holder, buff_index);
                choose_and_reply();
                buff_index = 0;
                timer_flag = 0;
            }
        }
        if (wifi_flag == 1)
        {
            server.handleClient();
        }
    }

private:
    // some BLE flags
    unsigned long bt_millis_count;
    int resetPasswordFlag = 0;
    int passWriteFlag = 0;
    int savedCalib = 0;
    int savedDeadzones = 0;
    int savedButtonMapping = 0;

    // serial buffer array
    int buffer_holder[BufferHolderLen] = {0};
    int buff_index = 0;
    int readVal = 0;

    // communications variables
    int command_byte = 0;
    int stop_bit_9 = 0;

    // esp32 helper to write to memory (save settings)
    Preferences preferences;

    /**
     * @brief Struct to hold the analog stick calibration params
     * 
     * @note the order for each arrray is low, neutch, high
     */
    struct StickCalibrationValues
    {
        int AX[3];
        int AY[3];
        int CX[3];
        int CY[3];
    } stickCalVals;

    /**
     * @brief Sruct to hold the analog stick deadzone values
     * 
     * @note the order for each arrray is low, neutch, high
     */
    struct StickDeadzoneValues
    {
        int AX[3];
        int AY[3];
        int CX[3];
        int CY[3];
    } stickDeadzVals;

    /**
     * @brief Struct to hold the trigger calibration values
     */
    struct TriggerCalibrationValues
    {
        int LTLow = 0;
        int LTHigh = 255;

        int RTLow = 0;
        int RTHigh = 255;
    } triggerCalVals;

    /**
     * @brief Struct to hold the sum of recent analog read values
     *
     * @note read_counter1 is the number of reads before averaging and updating reply
     */
    struct SummedAnalogReads
    {
        int ax = 0;
        int ay = 0;
        int cx = 0;
        int cy = 0;
        int al = 0;
        int ar = 0;
    } analogSums;

    /**
     * @brief Struct to hold the average of recent analog read values.
     *
     * @note These values are used to update the reply array
     */
    struct AveragedAnalogReads
    {
        int aX;
        int aY;
        int cX;
        int cY;
        int aL;
        int aR;

    } analogMeans;

    /**
     * @brief enum that maps two bit combination to message (hex value)
     */
    enum TwoBitMessage
    {
        ZeroZero = 4,
        OneOne = 55,
        ZeroOne = 52,
        OneZero = 7,
        STOP = 63
    };

    /**
     * @brief map to convert from two bits to the equivalent hex message
    */
    std::unordered_map<std::pair<int, int>, int, pair_hash> twoBitsToMessage =
        {{
          {std::make_pair(0, 0), ZeroZero},
          {std::make_pair(1,1), OneOne},
          {std::make_pair(0,1), ZeroOne},
          {std::make_pair(1,0), OneZero}
        }};

    /**
     * @brief enum to index each button in the buttons_in/digital_pins arrays. Similarly the 
    */
    enum PinIndex{
        START = 3,
        Y = 4,
        X = 5,
        B = 6,
        A = 7,
        L = 9,
        R = 10,
        Z = 11,
        DU = 12,
        DD = 13,
        DR = 14,
        DL = 15
    };

    // maps chars in the button mapping string to their corresponding buttons. Each pair of indices 3k, 3k+1
    // are for a different button
    std::unordered_map<int,int> buttonMappingMap = {{0, A}, {3, B}, {6, START},
                                                    {9, X}, {12, Y}, {15, Z},
                                                    {18, L}, {21, R}, {24, DU},
                                                    {27, DR}, {30, DD}, {33, DL}};

    // Maps for pin <--> char conversion
    std::unordered_map<int, char> pinToCharMap = {{22, 'A'}, {23, 'B'}, {21, 'S'}, {18, 'X'}, {4, 'Y'}, {15, 'Z'},
                                    {5, 'L'}, {19, 'R'}, {32, 'u'}, {33, 'r'}, {25, 'd'}, {14, 'l'}};
    
    std::unordered_map<char,int> charToPinMap = {{'A', 22}, {'B', 23}, {'S', 21}, {'X', 18}, {'Y', 4}, {'Z', 15},
                                {'L', 5}, {'R', 19}, {'u', 32}, {'r', 33}, {'d', 25}, {'l', 14}};

    // NOTE: enums to index these arrays? Gives names to random indexes floating around
    // Arrays holding pins of each digital and analog input
    int digital_pins[DigitalInLen] = {-1, -1, -1, 21, 4, 18, 23, 22, -1, 5, 19, 15, 32, 25, 33, 14}; // should save pin 2 to use for rumble (it's connected to the onboard LED), pin 2 on the devboard is attatched to a pull down resistor, Note gpio5 is connected to a pull up resistor

    // Q: what is/was this for?
    // int analog_input_pins[AnalogInLen] = {34, 39, 35, 13, 26, 27};                                         // maybe change 12 for 36 (aka vp) or 39 (aka vn) as 12 needs to be low during boot but currently is not??

    // Arrays to hold current digital toggling and mapping of buttons
    int digital_mapping[DigitalInLen] = {-1, -1, -1, 21, 4, 18, 23, 22, -1, 5, 19, 15, 32, 25, 33, 14};
    int digital_toggling[DigitalInLen] = {0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1};

    // Arrays to hold digital and analog inputs from controller
    // 0 0 0 Start Y X B A 1 L R Z D-U D-D D-R D-L
    // Analog-X Analog-Y C-Stick-X C-Stick-Y L-Trigger R-Trigger
    int buttons_in[DigitalInLen] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0};
    int analogs_in[AnalogInLen] = {128, 128, 128, 128, 0, 0};

    // Arrays for replies
    // InGameReply uses buttons_in and analogs_in to fill
    int InGameReply[InGameReplyLen] = {ZeroZero, ZeroZero, ZeroZero, ZeroZero, // 0 0 0 Start Y X B A
                                       OneZero, ZeroZero, ZeroZero, ZeroZero,  // 1 L R Z D-U D-D D-R D-L
                                       OneZero, ZeroZero, ZeroZero, ZeroZero,  // Analog X
                                       OneZero, ZeroZero, ZeroZero, ZeroZero,  // Analog Y
                                       OneZero, ZeroZero, ZeroZero, ZeroZero,  // C-Stick X
                                       OneZero, ZeroZero, ZeroZero, ZeroZero,  // C-Stick Y
                                       ZeroZero, ZeroZero, ZeroZero, ZeroZero, // L-Trigger
                                       ZeroZero, ZeroZero, ZeroZero, ZeroZero, // R-Trigger
                                       STOP};

    // todo:comment this
    int ProbeReply[ProbeReplyLen] = {ZeroZero, ZeroZero, OneZero, ZeroOne,
                                     ZeroZero, ZeroZero, ZeroZero, ZeroZero,
                                     ZeroZero, ZeroZero, ZeroZero, OneOne, STOP};

    int OriginEnd[OriginEndLen] = {ZeroZero, ZeroZero, ZeroZero, ZeroZero, // Null
                                   ZeroZero, ZeroZero, ZeroZero, ZeroZero, // Null
                                   STOP};
    // How many times to read inputs before averaging
    int read_counter1 = 0;

    //////// /* Private Methods*/ ////////

    void fillDigitalMappingMessage(int mapping[], int toggling[], char (&MappingMSG)[ButtonMappingMSGLen])
    {
        for (int i = 0; i < ButtonMappingMSGLen; i+=3){
            MappingMSG[i] = getCharFromPin(mapping[buttonMappingMap[i]]);
            MappingMSG[i+1] = getCharFromToggle(toggling[buttonMappingMap[i]]);
            MappingMSG[i+2] = '.';
        }
    }

    // set digital input pin modes to input_pullup
    void setPinsToPullup()
    {
        for (int i = 0; i < DigitalInLen; i++)
        {
            if (digital_pins[i] != -1)
            {
                pinMode(digital_pins[i], INPUT_PULLUP);
            }
        }
    }

    char getCharFromPin(int pin)
    {
        return pinToCharMap[pin];
    }

    int getPinFromChar(char char_read)
    {
        return charToPinMap[char_read];
    }

    char getCharFromToggle(int toggle)
    {
        char char_out;

        if (toggle == 1)
            {
                char_out = 'Y';
            }
        else if (toggle == 0)
            {
                char_out = 'N';
            }

        return char_out;
    }

    int getToggleFromChar(String char_read)
    {
        int toggle = 1;

        if (char_read == "Y")
        {
            toggle = 1;
        }
        else if (char_read == "N")
        {
            toggle = 0;
        }

        return toggle;
    }
    
    // Wrapper around read_inputs so we can start it as an rtos task. Needs to be static/global, not a class member func
    static void startTask(void *_this)
    {
        // (KEGController*)_this->read_inputs();
        static_cast<KEGController *>(_this)->read_inputs();
    }

    // The task called on core 0
    // Calls the function to update the input state arrays and delays as to not setoff a watchdog timeout
    // void read_inputs(void *parameter)
    void read_inputs()
    {
        for (;;)
        {
            check_buttons();

            if (millis() - bt_millis_count >= 90)
            {
                BLEHandler();
                bt_millis_count = millis();
            }

            TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
            TIMERG0.wdt_feed = 1;
            TIMERG0.wdt_wprotect = 0;

            for (int i = 0; i < 10000; i++)
            {
                __asm__("nop\n\t");
            }

            read_counter1++;
            if (read_counter1 >= 10)
            {
                update_reply();
                read_counter1 = 0;
            }
        }
    }

    // reads the values from all the digital and analog inputs
    // saves the read values into the appropriate arrays
    void check_buttons() // change name to read_inputs()?
    {
        for (int i = 0; i < DigitalInLen; i++)
        {
            if (digital_pins[i] != -1)
            {
                buttons_in[i] = !digitalRead(digital_pins[i]);
            }
        }

        int analogX = adc1_get_raw(ADC1_CHANNEL_6);
        analogSums.ax += analogX; /// 16;

        int analogY = adc1_get_raw(ADC1_CHANNEL_3);
        analogSums.ay += analogY; /// 16;

        int analogCX = adc1_get_raw(ADC1_CHANNEL_7);
        analogSums.cx += analogCX;

        int analogCY;
        adc2_get_raw(ADC2_CHANNEL_4, ADC_WIDTH_BIT_12, &analogCY);
        analogSums.cy += analogCY;

        int analogL;
        adc2_get_raw(ADC2_CHANNEL_9, ADC_WIDTH_BIT_9, &analogL);
        analogSums.al += analogL / 2;

        int analogR;
        adc2_get_raw(ADC2_CHANNEL_7, ADC_WIDTH_BIT_9, &analogR);
        analogSums.ar += analogR / 2;
    }

    // loops through the digital input array and converts the readings to 6 bit UART encodings
    // loops through analog input array and converts the readings to 6 bit UART encodings
    // these are then used to update the output signal from the controller
    void update_reply()
    {
        int next_val = 0;
        int j = 0;
        int count = 0;
        int val1 = 0;
        int val2 = 0;

        // check for mechanical L+R+A+Start depression
        // if they are depressed then changes the buttons_in array to reflect that
        checkForLRAStart();

        for (int i = 0; i < DigitalInLen; i = i + 2)
        {
            j = i + 1;
            val1 = buttons_in[i];
            val2 = buttons_in[j];
            next_val = twoBitsToMessage[{val1, val2}];
            InGameReply[count] = next_val;
            count++;
        }
        analogMeans.aX = analogSums.ax / read_counter1;
        analogs_in[0] = mapStickVals(stickCalVals.AX, stickDeadzVals.AX, analogMeans.aX);
        analogSums.ax = 0;

        analogMeans.aY = analogSums.ay / read_counter1;
        analogs_in[1] = mapStickVals(stickCalVals.AY, stickDeadzVals.AY, analogMeans.aY);
        analogSums.ay = 0;

        analogMeans.cX = analogSums.cx / read_counter1;
        analogs_in[2] = mapStickVals(stickCalVals.CX, stickDeadzVals.CX, analogMeans.cX);
        analogSums.cx = 0;

        analogMeans.cY = analogSums.cy / read_counter1;
        analogs_in[3] = mapStickVals(stickCalVals.CY, stickDeadzVals.CY, analogMeans.cY);
        analogSums.cy = 0;

        analogMeans.aL = analogSums.al / read_counter1;
        //  analogs_in[4] = mapTriggerVals(LTHigh,LTLow,aL);
        analogs_in[4] = analogMeans.aL;
        analogSums.al = 0;

        analogMeans.aR = analogSums.ar / read_counter1;
        //  analogs_in[5] = mapTriggerVals(RTHigh,RTLow,aR);
        analogs_in[5] = analogMeans.aR;
        analogSums.ar = 0;

        int analog_value = 0;
        for (int k = 0; k < AnalogInLen; k++)
        {
            analog_value = analogs_in[k];
            for (int i = 0; i < 7; i = i + 2)
            {
                j = i + 1;
                val1 = bitRead(analog_value, 7 - i);
                val2 = bitRead(analog_value, 7 - j);
                next_val = twoBitsToMessage[{val1, val2}];
                InGameReply[count] = next_val;
                count++;
            }
        }
    }

    // switch case statement that compares the command byte to known commands
    // based on the command bytes value a different message is sent
    // the uart buffer is then cleared as to not have a read of the sent data occur
    void choose_and_reply()
    {
        to_int();
        switch (command_byte)
        {
        case 0b00000000:
            if (stop_bit_9)
            {
                writeData(ProbeReply, ProbeReplyLen);
                Serial2.flush();
                Clear_UART_Rx();
            }
            break;
        case 0b01000001:
            if (stop_bit_9)
            {
                writeData(InGameReply, InGameReplyLen - 1);
                writeData(OriginEnd, OriginEndLen);
                Serial2.flush();
                Clear_UART_Rx();
            }
            break;
        case 0b01000000:
            writeData(InGameReply, InGameReplyLen);
            timer_flag = 0;
            Serial2.flush();
            Clear_UART_Rx();
            break;
        default:
            break;
        }
    }

    // checks the physical pins (5,19,22,21) which map to physical buttons (L,R,A,Start) respectively
    // if they are all high the sent message is changed to be only those 4 buttons clicked
    void checkForLRAStart()
    {
        int LRAStart = 0;
        LRAStart += !digitalRead(charToPinMap['L']);
        LRAStart += !digitalRead(charToPinMap['R']);
        LRAStart += !digitalRead(charToPinMap['A']);
        LRAStart += !digitalRead(charToPinMap['S']);
        if (LRAStart == 4)
        { // if they were all pressed change buttons_in array to be exactly those 4 buttons pushed
            for (int i = 0; i < DigitalInLen; i++)
            {
                if (i == START || i == A || i == L || i == R || i == 8)
                {
                    buttons_in[i] = 1;
                }
                else
                {
                    buttons_in[i] = 0;
                }
            }
        }
    }

    /* Writes */

    // writes the current stick calibration values to memory
    void writeStickCalToMem()
    {
        preferences.begin("AnalogCal", false);
        uint16_t toSave;

        toSave = stickCalVals.AX[1];
        preferences.putUShort("AXNeutch", toSave);
        toSave = stickCalVals.AX[2];
        preferences.putUShort("AXHigh", toSave);
        toSave = stickCalVals.AX[0];
        preferences.putUShort("AXLow", toSave);

        toSave = stickCalVals.AY[1];
        preferences.putUShort("AYNeutch", toSave);
        toSave = stickCalVals.AY[2];
        preferences.putUShort("AYHigh", toSave);
        toSave = stickCalVals.AY[0];
        preferences.putUShort("AYLow", toSave);

        toSave = stickCalVals.CX[1];
        preferences.putUShort("CXNeutch", toSave);
        toSave = stickCalVals.CX[2];
        preferences.putUShort("CXHigh", toSave);
        toSave = stickCalVals.CX[0];
        preferences.putUShort("CXLow", toSave);

        toSave = stickCalVals.CY[1];
        preferences.putUShort("CYNeutch", toSave);
        toSave = stickCalVals.CY[2];
        preferences.putUShort("CYHigh", toSave);
        toSave = stickCalVals.CY[0];
        preferences.putUShort("CYLow", toSave);

        preferences.end();
    }

    // writes the current stick calibration values to memory
    void writeStickDeadzonesToMem()
    {
        preferences.begin("AnalogDead", false);
        uint16_t toSave;

        toSave = stickDeadzVals.AX[0];
        preferences.putUShort("AXDeadLow", toSave);
        toSave = stickDeadzVals.AX[1];
        preferences.putUShort("AXDeadHigh", toSave);
        toSave = stickDeadzVals.AX[2];
        preferences.putUShort("AXDeadVal", toSave);

        toSave = stickDeadzVals.AY[0];
        preferences.putUShort("AYDeadLow", toSave);
        toSave = stickDeadzVals.AY[1];
        preferences.putUShort("AYDeadHigh", toSave);
        toSave = stickDeadzVals.AY[2];
        preferences.putUShort("AYDeadVal", toSave);

        toSave = stickDeadzVals.CX[0];
        preferences.putUShort("CXDeadLow", toSave);
        toSave = stickDeadzVals.CX[1];
        preferences.putUShort("CXDeadHigh", toSave);
        toSave = stickDeadzVals.CX[2];
        preferences.putUShort("CXDeadVal", toSave);

        toSave = stickDeadzVals.CY[0];
        preferences.putUShort("CYDeadLow", toSave);
        toSave = stickDeadzVals.CY[1];
        preferences.putUShort("CYDeadHigh", toSave);
        toSave = stickDeadzVals.CY[2];
        preferences.putUShort("CYDeadVal", toSave);

        preferences.end();
    }

    // writes the current button mapping to memory
    void writeButtonMappingToMem()
    {
        preferences.begin("DigitalMap", false);
        uint16_t toSave;

        toSave = digital_mapping[A];
        preferences.putUShort("A", toSave);
        toSave = digital_toggling[A];
        preferences.putUShort("AT", toSave);

        toSave = digital_mapping[B];
        preferences.putUShort("B", toSave);
        toSave = digital_toggling[B];
        preferences.putUShort("BT", toSave);

        toSave = digital_mapping[START];
        preferences.putUShort("S", toSave);
        toSave = digital_toggling[A];
        preferences.putUShort("ST", toSave);

        toSave = digital_mapping[X];
        preferences.putUShort("X", toSave);
        toSave = digital_toggling[X];
        preferences.putUShort("XT", toSave);

        toSave = digital_mapping[Y];
        preferences.putUShort("Y", toSave);
        toSave = digital_toggling[Y];
        preferences.putUShort("YT", toSave);

        toSave = digital_mapping[Z];
        preferences.putUShort("Z", toSave);
        toSave = digital_toggling[Z];
        preferences.putUShort("ZT", toSave);

        toSave = digital_mapping[L];
        preferences.putUShort("L", toSave);
        toSave = digital_toggling[L];
        preferences.putUShort("LT", toSave);

        toSave = digital_mapping[R];
        preferences.putUShort("R", toSave);
        toSave = digital_toggling[R];
        preferences.putUShort("RT", toSave);

        toSave = digital_mapping[DU];
        preferences.putUShort("u", toSave);
        toSave = digital_toggling[DU];
        preferences.putUShort("uT", toSave);

        toSave = digital_mapping[DR];
        preferences.putUShort("r", toSave);
        toSave = digital_toggling[DR];
        preferences.putUShort("rT", toSave);

        toSave = digital_mapping[DD];
        preferences.putUShort("d", toSave);
        toSave = digital_toggling[DD];
        preferences.putUShort("dT", toSave);

        toSave = digital_mapping[DL];
        preferences.putUShort("l", toSave);
        toSave = digital_toggling[DL];
        preferences.putUShort("lT", toSave);

        preferences.end();
    }

    void writeBLEpasswordToMem(String newPass)
    {
        preferences.begin("Passwords", false);
        preferences.putString("BLEPass", newPass);
        preferences.end();
        BLEpassword = newPass;
    }

    // reads the stick calibration values from memory
    void readStickDeadzonesFromMem()
    {
        preferences.begin("AnalogDead", true);

        stickDeadzVals.AX[0] = preferences.getUShort("AXDeadLow", 117);
        stickDeadzVals.AX[1] = preferences.getUShort("AXDeadHigh", 137);
        stickDeadzVals.AX[2] = preferences.getUShort("AXDeadVal", 127);

        stickDeadzVals.AY[0] = preferences.getUShort("AYDeadLow", 117);
        stickDeadzVals.AY[1] = preferences.getUShort("AYDeadHigh", 137);
        stickDeadzVals.AY[2] = preferences.getUShort("AYDeadVal", 127);

        stickDeadzVals.CX[0] = preferences.getUShort("CXDeadLow", 117);
        stickDeadzVals.CX[1] = preferences.getUShort("CXDeadHigh", 137);
        stickDeadzVals.CX[2] = preferences.getUShort("CXDeadVal", 127);

        stickDeadzVals.CY[0] = preferences.getUShort("CYDeadLow", 117);
        stickDeadzVals.CY[1] = preferences.getUShort("CYDeadHigh", 137);
        stickDeadzVals.CY[2] = preferences.getUShort("CYDeadVal", 127);

        preferences.end();
    }

    // reads the stick calibration values from memory
    void readStickCalFromMem()
    {
        preferences.begin("AnalogCal", true);

        stickCalVals.AX[1] = preferences.getUShort("AXNeutch", 0); // if key not there default to 0
        stickCalVals.AX[2] = preferences.getUShort("AXHigh", 0);
        stickCalVals.AX[0] = preferences.getUShort("AXLow", 0);

        stickCalVals.AY[1] = preferences.getUShort("AYNeutch", 0);
        stickCalVals.AY[2] = preferences.getUShort("AYHigh", 0);
        stickCalVals.AY[0] = preferences.getUShort("AYLow", 0);

        stickCalVals.CX[1] = preferences.getUShort("CXNeutch", 0);
        stickCalVals.CX[2] = preferences.getUShort("CXHigh", 0);
        stickCalVals.CX[3] = preferences.getUShort("CXLow", 0);

        stickCalVals.CY[1] = preferences.getUShort("CYNeutch", 0);
        stickCalVals.CY[2] = preferences.getUShort("CYHigh", 0);
        stickCalVals.CY[0] = preferences.getUShort("CYLow", 0);

        preferences.end();
    }

    //  0  0  0 Start Y  X   B   A   1   L  R   Z   D-U  D-D  D-R  D-L
    //{-1,-1,-1, 21,  4, 18, 23, 22, -1, 5, 19, 15, 32,  25,  33,  14};
    void readButtonMappingFromMem()
    {
        preferences.begin("DigitalMap", true);

        digital_mapping[A] = preferences.getUShort("A", 22);
        digital_toggling[A] = preferences.getUShort("AT", 1);

        digital_mapping[B] = preferences.getUShort("B", 23);
        digital_toggling[B] = preferences.getUShort("BT", 1);

        digital_mapping[START] = preferences.getUShort("S", 21);
        digital_toggling[START] = preferences.getUShort("ST", 1);

        digital_mapping[X] = preferences.getUShort("X", 18);
        digital_toggling[X] = preferences.getUShort("XT", 1);

        digital_mapping[Y] = preferences.getUShort("Y", 4);
        digital_toggling[Y] = preferences.getUShort("YT", 1);

        digital_mapping[Z] = preferences.getUShort("Z", 15);
        digital_toggling[Z] = preferences.getUShort("ZT", 1);

        digital_mapping[L] = preferences.getUShort("L", 5);
        digital_toggling[L] = preferences.getUShort("LT", 1);

        digital_mapping[R] = preferences.getUShort("R", 19);
        digital_toggling[R] = preferences.getUShort("RT", 1);

        digital_mapping[DU] = preferences.getUShort("u", 32);
        digital_toggling[DU] = preferences.getUShort("uT", 1);

        digital_mapping[DR] = preferences.getUShort("r", 33);
        digital_toggling[DR] = preferences.getUShort("rT", 1);

        digital_mapping[DD] = preferences.getUShort("d", 25);
        digital_toggling[DD] = preferences.getUShort("dT", 1);

        digital_mapping[DL] = preferences.getUShort("l", 14);
        digital_toggling[DL] = preferences.getUShort("lT", 1);

        preferences.end();

        updateDigitalInputPins();
    }

    void readBLEpasswordFromMem()
    {
        preferences.begin("Passwords", true);
        BLEpassword = preferences.getString("BLEPass", "KEG CONCH");
        preferences.end();
    }

    void swap_buttons(int ind1, int ind2)
    {
        int temp = digital_pins[ind1];
        digital_pins[ind1] = digital_pins[ind2];
        digital_pins[ind2] = temp;
    }

    // updates digital_pins with the currently mapped values
    void updateDigitalInputPins()
    {
        for (int i = 0; i < DigitalInLen; i++)
        {
            if (digital_toggling[i] == 0)
            {
                digital_pins[i] = -1;
            }
            else
            {
                digital_pins[i] = digital_mapping[i];
            }
        }
    }

    // loop through btnMappingStr and update 
    // why not save string to memory and then use this to update? Removes read
    void updateButtonMapping(String btnMappingStr)
    {
        int pin;
        int toggle;
        String read_val_1;
        String read_val_2;

        for (int i=0; i < btnMappingStr.length(); i+=3)
        {
            read_val_1 = btnMappingStr.charAt(i);
            read_val_2 = btnMappingStr.charAt(i+1);
            toggle = getToggleFromChar(read_val_2);
            digital_mapping[buttonMappingMap[i]] = pin;
            digital_toggling[buttonMappingMap[i]] = toggle;
        }
        updateDigitalInputPins();
    }

    // parses the calibration string and fills the stick calibration struct with the new values.
    // New values stored in struct but not saved to memory unless user chooses to
    void ParseCalibrationString(String str)
    {
        stickCalVals.AX[1] = str.substring(0, 4).toInt();
        stickCalVals.AX[0] = str.substring(5, 9).toInt();
        stickCalVals.AX[2] = str.substring(10, 14).toInt();

        stickCalVals.AY[1] = str.substring(15, 19).toInt();
        stickCalVals.AY[0] = str.substring(20, 24).toInt();
        stickCalVals.AY[2] = str.substring(25, 29).toInt();

        stickCalVals.CX[1] = str.substring(30, 34).toInt();
        stickCalVals.CX[3] = str.substring(35, 39).toInt();
        stickCalVals.CX[2] = str.substring(40, 44).toInt();

        stickCalVals.CY[1] = str.substring(45, 49).toInt();
        stickCalVals.CY[0] = str.substring(50, 54).toInt();
        stickCalVals.CY[2] = str.substring(55, 59).toInt();
    }

    // parses deadzone string and fills stick deadzone struct with new values.
    // similarly not saved to memory until user chooses to
    void ParseDeadzoneString(String str)
    {
        stickDeadzVals.AX[0] = str.substring(0, 3).toInt();
        stickDeadzVals.AX[1] = str.substring(4, 7).toInt();
        stickDeadzVals.AX[2] = str.substring(8, 11).toInt();

        stickDeadzVals.AY[0] = str.substring(12, 15).toInt();
        stickDeadzVals.AY[1] = str.substring(16, 19).toInt();
        stickDeadzVals.AY[2] = str.substring(20, 23).toInt();

        stickDeadzVals.CX[0] = str.substring(24, 27).toInt();
        stickDeadzVals.CX[1] = str.substring(28, 31).toInt();
        stickDeadzVals.CX[2] = str.substring(32, 35).toInt();

        stickDeadzVals.CY[0] = str.substring(36, 39).toInt();
        stickDeadzVals.CY[1] = str.substring(40, 43).toInt();
        stickDeadzVals.CY[2] = str.substring(44, 47).toInt();
    }

    void setupBLE()
    {
        BLEDevice::init(bleServerName);

        // Create the BLE Server
        BLEServer *pServer = BLEDevice::createServer();
        pServer->setCallbacks(new MyServerCallbacks());

        // Create the BLE Service
        BLEService *Service1 = pServer->createService(SERVICE_UUID);
        Service1->addCharacteristic(&Ch1);
        Service1->addCharacteristic(&Ch2);
        Ch1.addDescriptor(&Dp1);

        if (resetPasswordFlag == 1)
            Ch1.setValue("Reset Password");
        else
            Ch1.setValue("Password Incorrect");

        Ch2.addDescriptor(&Dp2);
        Ch2.setValue("Waiting for change to exactly: A");

        // Start advertising
        Service1->start();
        BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
        pAdvertising->addServiceUUID(SERVICE_UUID);
        pServer->getAdvertising()->start();
        bt_millis_count = millis();
    }

    // Handles all bluetooth requests coming from the received message
    void BLEHandler()
    {
        if (bluetoothConnected)
        {
            receivedMSG = (String)Ch2.getValue().c_str();
            //    Serial.println(receivedMSG);
            char fifthChar = receivedMSG[4];
            char fourthChar = receivedMSG[3];
            char thirdChar = receivedMSG[2];
            int isX = !digitalRead(18);
            int isY = !digitalRead(4);
            char firstChar = receivedMSG[0];

            //    Serial.println(fifthChar == ',');
            //    Serial.println();
            if (resetPasswordFlag == 1)
            {
                if (firstChar == 'P' && passWriteFlag == 0)
                {
                    //        Serial.println(receivedMSG.substring(1));
                    writeBLEpasswordToMem(receivedMSG.substring(1));
                    passWriteFlag = 1;
                    resetPasswordFlag = 0;
                    Ch1.setValue("Password Reset");
                    Ch1.notify();
                }
                else
                {
                    Ch1.setValue("Reset Password");
                    Ch1.notify();
                }
            }
            else
            {
                if (password_correct == 0)
                {
                    if ((String)Ch2.getValue().c_str() == BLEpassword && isX && isY)
                    {
                        password_correct = 1;
                        Ch1.setValue("Password Correct");
                        Ch1.notify();
                    }
                    else
                    {
                        Ch1.setValue("Password Incorrect");
                        Ch1.notify();
                    }
                }
                else
                {
                    if (receivedMSG == "A")
                    { // A means requesting Analog data
                        sprintf(AnalogMSG, "%04d,%04d,%04d,%04d,%04d,%04d", analogMeans.aX, analogMeans.aY,
                                analogMeans.cX, analogMeans.cY,
                                analogMeans.aL, analogMeans.aR);
                        Ch1.setValue(AnalogMSG);
                        Ch1.notify();
                    }
                    else if (fifthChar == ',')
                    {
                        ParseCalibrationString(receivedMSG);
                    }
                    else if (receivedMSG == "SAC" && savedCalib == 0)
                    { // SAC means Save Analog Calibration Values
                        writeStickCalToMem();
                        writeStickDeadzonesToMem();
                        savedCalib = 1;
                    }
                    else if (receivedMSG == "RAC")
                    {
                        sprintf(AnalogCalibMSG, "%04d,%04d,%04d:%04d,%04d,%04d:%04d,%04d,%04d:%04d,%04d,%04d", 
                                stickCalVals.AX[1], stickCalVals.AX[0], stickCalVals.AX[2],
                                stickCalVals.AY[1], stickCalVals.AY[0], stickCalVals.AY[2],
                                stickCalVals.CX[1], stickCalVals.CX[0], stickCalVals.CX[2],
                                stickCalVals.CY[1], stickCalVals.CY[0], stickCalVals.CY[2]);
                        Ch1.setValue(AnalogCalibMSG);
                        Ch1.notify();
                    }
                    else if (fourthChar == ',')
                    {
                        ParseDeadzoneString(receivedMSG);
                    }
                    else if (receivedMSG == "SSD" && savedDeadzones == 0)
                    { // SSD = Save Stick Deadzones
                        writeStickDeadzonesToMem();
                        savedDeadzones = 1;
                    }
                    else if (thirdChar == '.')
                    {
                        updateButtonMapping(receivedMSG);
                    }
                    else if (receivedMSG == "RBM")
                    {
                        fillDigitalMappingMessage(digital_mapping, digital_toggling, DigitalMappingMSG);
                        Ch1.setValue(DigitalMappingMSG);
                        Ch1.notify();
                    }
                    else if (receivedMSG == "SBM" && savedButtonMapping == 0)
                    {
                        writeButtonMappingToMem();
                        savedButtonMapping = 1;
                    }
                    else if (firstChar == 'W' && wifi_flag == 0)
                    {
                        wifiUploadEnabled(receivedMSG);
                        Ch1.setValue(ipAddy);
                        Ch1.notify();
                    }
                    else if (firstChar == 'P' && passWriteFlag == 0)
                    {
                        writeBLEpasswordToMem(receivedMSG.substring(1));
                        passWriteFlag = 1;
                    }
                    else if (receivedMSG == "RDC")
                    {
                        sprintf(AnalogDeadzoneMSG, "%03d,%03d,%03d:%03d,%03d,%03d:%03d,%03d,%03d:%03d,%03d,%03d", 
                                stickDeadzVals.AX[0], stickDeadzVals.AX[1], stickDeadzVals.AX[2],
                                stickDeadzVals.AY[0], stickDeadzVals.AY[1], stickDeadzVals.AY[2],
                                stickDeadzVals.CX[0], stickDeadzVals.CX[1], stickDeadzVals.CX[2],
                                stickDeadzVals.CY[0], stickDeadzVals.CY[1], stickDeadzVals.CY[2]
                        );
                        Ch1.setValue(AnalogDeadzoneMSG);
                        Ch1.notify();
                    }
                }
            }
        }
    }

    // converts the first 8 GameCube bits of data to a command byte
    // also sets the stop_bit_9 variable which tells us if the 9th GameCube bit was a stop bit or not
    void to_int()
    {
        int j = 0;
        for (int i = 0; i < 4; i++)
        {
            j = i * 2;
            switch (buffer_holder[i])
            {
            case ZeroZero:
                bitWrite(command_byte, 8 - j - 1, 0);
                bitWrite(command_byte, 8 - j - 2, 0);
                break;
            case ZeroOne:
                bitWrite(command_byte, 8 - j - 1, 0);
                bitWrite(command_byte, 8 - j - 2, 1);
                break;
            case OneZero:
                bitWrite(command_byte, 8 - j - 1, 1);
                bitWrite(command_byte, 8 - j - 2, 0);
                break;
            case OneOne:
                bitWrite(command_byte, 8 - j - 1, 1);
                bitWrite(command_byte, 8 - j - 2, 1);
                break;
            default:
                break;
            }
        }
        if (buffer_holder[4] == STOP)
        {
            stop_bit_9 = 1;
        }
        else
        {
            stop_bit_9 = 0;
        }
    }
};


