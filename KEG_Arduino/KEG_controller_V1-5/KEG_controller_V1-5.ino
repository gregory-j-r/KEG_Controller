#include <stdio.h>
#include <string.h>
#include <unordered_map>

#include <Preferences.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "driver/adc.h"

#include "utils.h"
#include "wireless.h"

#define ProbeReplyLen 13
#define InGameReplyLen 33

#define OriginEndLen 9

#define ButtonMappingMSGLen 139
#define DigitalInLen 16
#define AnalogInLen 6

#define RXD2 16
#define TXD2 17
#define BufferHolderLen 45

#define LEDPin 2


// Pointer to task ReadInputs running on core 0. For RTOS
TaskHandle_t ReadInputs;

// timer stuff
hw_timer_t *My_timer = NULL;
int timer_flag = 1;
int triggered = 0;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer(){
    triggered = 1;
}

int numCalibPoints = 7;

/**
 * @brief Use all of octagon notches to calibrate  
 * 
 * @note assumes calArray is formatted as
 *   - {west, notch1, notch2, neutch, notch3, notch4, east}
 *   - {south, notch1, notch2, neutch,  notch3, notch4, north}

*/
int mapStickVals(int calArray[], int sortedCalArray[], int dead[], int val){
    int mapped_val = 127;
    bool flipped_magnet_dir;

    int first = calArray[0];
    int last = calArray[6];
    flipped_magnet_dir = (first<last) ? false : true;

    // Find which section val is in
    int index = bisect(sortedCalArray, val, numCalibPoints);

    if (index == 0){
        mapped_val = 0;
    }
    else if (index == 1){
        int a = 0;
        int b = 37;

        int min_val = sortedCalArray[0];
        int max_val = sortedCalArray[1];
        mapped_val = (a + (val - min_val)*(b-a) / (max_val - min_val));
    }
    else if (index == 2){
        mapped_val = 37;
    }
    else if (index == 3){
        int a = 37;
        int b = 127;

        int min_val = sortedCalArray[2];
        int max_val = sortedCalArray[3];
        mapped_val = (a + (val - min_val)*(b-a) / (max_val - min_val));
    }
    else if (index == 4){
        int a = 127;
        int b = 218;

        int min_val = sortedCalArray[3];
        int max_val = sortedCalArray[4];
        mapped_val = (a + (val - min_val)*(b-a) / (max_val - min_val));
    }
    else if (index == 5){
        mapped_val = 218;
    }
    else if (index == 6){
        int a = 218;
        int b = 255;

        int min_val = sortedCalArray[5];
        int max_val = sortedCalArray[6];
        mapped_val = (a + (val - min_val)*(b-a) / (max_val - min_val));
    }
    else if (index == numCalibPoints){
        mapped_val = 255;
    }
    
    if(mapped_val >= dead[0] and mapped_val <= dead[1]){
        mapped_val = dead[2];
    }

    if (flipped_magnet_dir){
        if (index == 3 || index == 4){
            return 254 - mapped_val;
        }
        return 255-mapped_val;
    }
    
    return mapped_val;
}

/**
 * @brief This class defines the code to setup and run a KEG controller.
 *
 */
class KEGController{

public:
    /**
     * @brief Initialize controller to run. Sets up bluetooth and serial communications, empties buffer, reads
     *        data from memory, configures adc channels
     */
    void setup(){
        setPinsToPullup(digital_pins, DigitalInLen); // set the pins to input_pullup mode

        // check to see if holding down y and dpad-down on boot.
        // Will erase OTA firmware and reset to factory firmware
        if (!digitalRead(charToPinMap['Y']) && !digitalRead(charToPinMap['d'])){
            eraseOTA();
            delay(1000);
            ESP.restart();
        }

        readStickCalFromMem();       // read the stick calibration values from memory
        readStickDeadzonesFromMem(); // read the stick deadzone values from memory
        readButtonMappingFromMem();  // read button mapping from memory
        readBLEpasswordFromMem();    // read BLE Password from memory
        readBLENameFromMem();        // read BLEName from memory

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
        if (!digitalRead(charToPinMap['X']) && !digitalRead(charToPinMap['Y'])){
            resetPasswordFlag = 1;
        } 
        
        bluetooth.setupBLE(bt_millis_count, resetPasswordFlag);

        pinMode(LEDPin, OUTPUT);

        My_timer = timerBegin(0, 80, true);
        timerAttachInterrupt(My_timer, &onTimer, true);
        timerAlarmWrite(My_timer, 115, false);

        Serial.begin(9600);

        xTaskCreatePinnedToCore(this->startTask, "Read Inputs", 10000, this, 0, &ReadInputs, /* Task handle. */ 0 /*core #*/);

        Serial2.begin(800000, SERIAL_6N1, RXD2, TXD2);
        Serial2.setRxBufferSize(256);
    }

    /**
     * @brief main controller loop. This does one iteration of the loop so it can be called in
     *  the arduino loop function.
     */
    void loop(){
        if (Serial2.available() > 0){
            if (timer_flag == 0){
                timerWrite(My_timer, 0);
                timerAlarmEnable(My_timer);
                timer_flag = 1;
            }
            readVal = Serial2.read();

            if (buff_index >= BufferHolderLen){
                buff_index = 0;
            }
            buffer_holder[buff_index] = readVal;
            buff_index++;

            if (triggered == 1){
                cleanUpBufferRead(buffer_holder, buff_index);
                choose_and_reply();
                clearUARTrx();
                buff_index = 0;
                triggered = 0;
                timer_flag = 0;
            }

            if (readVal == STOP){
                timerAlarmDisable(My_timer);
                cleanUpBufferRead(buffer_holder, buff_index);
                choose_and_reply();
                buff_index = 0;
                timer_flag = 0;
            }
        }
        if (wifi_flag == 1){
            server.handleClient();
        }
    }

private:
    // BLE stuff
    String BLEpassword;
    int BLENameWriteFlag = 0;
    unsigned long bt_millis_count;
    int resetPasswordFlag = 0;
    int passWriteFlag = 0;
    int savedCalib = 0;
    int savedDeadzones = 0;
    int savedButtonMapping = 0;
    char DigitalMappingMSG[35];
    KEGBluetooth bluetooth;

    // serial buffer array
    int buffer_holder[BufferHolderLen] = {0};
    int buff_index = 0;
    int readVal = 0;

    // communications variables
    int command_byte = 0;
    int stop_bit_9 = 0;

    // esp32 class for writing to memory (save settings)
    Preferences preferences;

    /**
     * @brief Struct to hold the analog stick calibration params. 
     *
     * @note formatting is:
     *         x: west, southwest*, northwest*, neutch, northeast*, southeast*, east
     *         y: south, southwest*, southeast*, neutch, northwest*, northeast*, north
     */
    struct StickCalibrationValues{
        int AX[7] = {};
        int AY[7] = {};
        int CX[7] = {};
        int CY[7] = {};
    } stickCalVals;

    /**
     * @brief Struct to hold the analog stick calibration params in sorted ascending order
     *
     */
    struct SortedStickCalibrationValues{
        int AX[7] = {};
        int AY[7] = {};
        int CX[7] = {};
        int CY[7] = {};
    } sortedStickCalVals;

    /**
     * @brief Sruct to hold the analog stick deadzone values. Ordering is low, neutch, high
     *
     */
    struct StickDeadzoneValues{
        int AX[3] = {};
        int AY[3] = {};
        int CX[3] = {};
        int CY[3] = {};
    } stickDeadzVals;

    /**
     * @brief Struct to hold the trigger calibration values
     */
    struct TriggerCalibrationValues{
        int LTLow = 0;
        int LTHigh = 255;

        int RTLow = 0;
        int RTHigh = 255;
    } triggerCalVals;

    /**
     * @brief Struct to hold the sum of recent analog read values
     *
     */
    struct SummedAnalogReads{
        int ax = 0;
        int ay = 0;
        int cx = 0;
        int cy = 0;
        int al = 0;
        int ar = 0;
    } analogSums;

    /**
     * @brief Struct to hold the average of recent analog reads, the values that are sent back to the GC
     */
    struct AveragedAnalogReads{
        int aX = 0;
        int aY = 0;
        int cX = 0;
        int cY = 0;
        int aL = 0;
        int aR = 0;

    } analogMeans;

    /**
     * @brief enum that maps two bit combination to message
     */
    enum TwoBitMessage{
        ZeroZero = 4,
        OneOne = 55,
        ZeroOne = 52,
        OneZero = 7,
        STOP = 63
    };

    /**
     * @brief enum to index each button in the buttons_in/digital_pins arrays.
     */
    enum PinIndex{
        START = 3, Y = 4, X = 5,
        B = 6, A = 7, L = 9,
        R = 10, Z = 11, DU = 12,
        DD = 13, DR = 14,  DL = 15
    };

    /**
     * @brief maps to convert between two bits <--> equivalent message
     */
    std::unordered_map<std::pair<int, int>, int, pair_hash> twoBitsToMessage =
        {{{std::make_pair(0, 0), ZeroZero},
          {std::make_pair(1, 1), OneOne},
          {std::make_pair(0, 1), ZeroOne},
          {std::make_pair(1, 0), OneZero}}};

    std::unordered_map<int, std::pair<int, int>> messageToTwoBits =
        {{{ZeroZero, std::make_pair(0, 0)},
          {OneOne, std::make_pair(1, 1)},
          {ZeroOne, std::make_pair(0, 1)},
          {OneZero, std::make_pair(1, 0)}}};

    /**
     * @brief Maps index in the button mapping string to the corresponding buttons. Each pair of indices
     *        3k, 3k+1 are for a different button. For parsing.
     */
    std::unordered_map<int, int> buttonMappingMap = {{0, A}, {3, B}, {6, START}, {9, X}, {12, Y}, {15, Z}, {18, L}, {21, R}, {24, DU}, {27, DR}, {30, DD}, {33, DL}};

    /**
     * @brief LUT's for the original pin to button mapping.
     *        
     */
    std::unordered_map<int, char> pinToCharMap = {{22, 'A'}, {23, 'B'}, {21, 'S'}, {18, 'X'}, {4, 'Y'}, {15, 'Z'}, {5, 'L'}, {19, 'R'}, {32, 'u'}, {33, 'r'}, {25, 'd'}, {14, 'l'}};

    std::unordered_map<char, int> charToPinMap = {{'A', 22}, {'B', 23}, {'S', 21}, {'X', 18}, {'Y', 4}, {'Z', 15}, {'L', 5}, {'R', 19}, {'u', 32}, {'r', 33}, {'d', 25}, {'l', 14}};

    /**
     * @brief Arrays to hold digital and analog inputs from controller
     *
     * @note The formatting for these arrays:
     *
     *        buttons_in = [0, 0, 0, Start, Y, X, B, A, 1, L, R, Z, D-U, D-D, D-R, D-L]
     *        analogs_in = [Analog-X, Analog-Y, C-Stick-X, C-Stick-Y, L-Trigger, R-Trigger]
     */
    int buttons_in[DigitalInLen] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0};
    int analogs_in[AnalogInLen] = {128, 128, 128, 128, 0, 0};

    /**
     * @brief Array to hold digital pins for each button. This will be updated with the pins from
     *        the current mapping.
     *
     * @note  Should save pin 2 to use for rumble (it's connected to the onboard LED).
     *        Pin 2 on the devboard is attatched to a pull down resistor, and note that
     *        gpio5 is connected to a pull up resistor
     */
    int digital_pins[DigitalInLen] = {-1, -1, -1, 21, 4, 18, 23, 22, -1, 5, 19, 15, 32, 25, 33, 14};

    /**
     * @brief arrays to hold button mapping and toggling
     */
    int digital_mapping[DigitalInLen] = {-1, -1, -1, 21, 4, 18, 23, 22, -1, 5, 19, 15, 32, 25, 33, 14};
    int digital_toggling[DigitalInLen] = {0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1};

    /**
     * @brief Arrays to send replies to the gamecube. InGameReply filled with values from the
     *        buttons_in and analogs_in arrays
     */
    int InGameReply[InGameReplyLen] = {ZeroZero, ZeroZero, ZeroZero, ZeroZero, // 0 0 0 Start Y X B A
                                       OneZero, ZeroZero, ZeroZero, ZeroZero,  // 1 L R Z D-U D-D D-R D-L
                                       OneZero, ZeroZero, ZeroZero, ZeroZero,  // Analog X
                                       OneZero, ZeroZero, ZeroZero, ZeroZero,  // Analog Y
                                       OneZero, ZeroZero, ZeroZero, ZeroZero,  // C-Stick X
                                       OneZero, ZeroZero, ZeroZero, ZeroZero,  // C-Stick Y
                                       ZeroZero, ZeroZero, ZeroZero, ZeroZero, // L-Trigger
                                       ZeroZero, ZeroZero, ZeroZero, ZeroZero, // R-Trigger
                                       STOP};

    int ProbeReply[ProbeReplyLen] = {ZeroZero, ZeroZero, OneZero, ZeroOne,
                                     ZeroZero, ZeroZero, ZeroZero, ZeroZero,
                                     ZeroZero, ZeroZero, ZeroZero, OneOne, STOP};

    int OriginEnd[OriginEndLen] = {ZeroZero, ZeroZero, ZeroZero, ZeroZero, // Null
                                   ZeroZero, ZeroZero, ZeroZero, ZeroZero, // Null
                                   STOP};

    /**
     * @brief Counts number of reads before averaging
     */
    int read_counter = 0;


    /*******************************************************************
     *  PRIVATE METHODS
     *******************************************************************/

    /**
     * @brief Fills the button mapping message with values from the mapping and toggling arrays
     */
    void fillDigitalMappingMessage(){
        for (int i = 0; i < ButtonMappingMSGLen - 1; i += 3)
        {
            DigitalMappingMSG[i] = pinToCharMap[digital_mapping[buttonMappingMap[i]]];
            DigitalMappingMSG[i + 1] = getCharFromToggle(digital_toggling[buttonMappingMap[i]]);

            if (i < ButtonMappingMSGLen - 2)
            {
                DigitalMappingMSG[i + 2] = '.';
            }
        }
    }

    /**
     * @brief Called when stickCalVals is filled or updated so the two match
     * 
    */
    void sortStickCalVals() {
        sort(sortedStickCalVals.AX, numCalibPoints);
        sort(sortedStickCalVals.AY, numCalibPoints);
        sort(sortedStickCalVals.CX, numCalibPoints);
        sort(sortedStickCalVals.CY, numCalibPoints);
    }
 
    /**
     * @brief wrapper around the realtime task read_inputs. FreeRTOS task functions need to be static
     *        or global so this is a way around it being a class member function.
     */
    static void startTask(void *_this){
        static_cast<KEGController *>(_this)->read_inputs();
    }

    /**
     * @brief The realtime task loop attached to core 0.
     *        Inputs are read from the controller and summed in check_buttons, then the
     *        values are averaged and the reply is updated in update_reply
     *
     * @note  Delays so as to not set off a watchdog timeout
     */
    void read_inputs(){
        for (;;)
        {
            check_buttons();

            if (millis() - bt_millis_count >= 83)
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

            read_counter++;
            if (read_counter >= 10)
            {
                update_reply();
                read_counter = 0;
            }
        }
    }

    /**
     * @brief Reads the values from all the digital and analog inputs, and saves
     *        the read values into the appropriate arrays
     *        
     *        change name to read_inputs()?
     */
    void check_buttons(){
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
  
    /**
     * @brief Loops through the analog and digital input arrays and converts readings to 6 bit UART encodings.
     *        These are then used to update the output signal from the controller
     */
    void update_reply(){
        int next_val = 0;
        int j = 0;
        int count = 0;
        int val1 = 0;
        int val2 = 0;

        // check for mechanical L+R+A+Start depression
        // if they are depressed then changes the buttons_in array to reflect that
        checkForLRAStart();

        for (int i = 0; i < DigitalInLen; i = i + 2){
            j = i + 1;
            val1 = buttons_in[i];
            val2 = buttons_in[j];
            next_val = twoBitsToMessage[{val1, val2}];
            InGameReply[count] = next_val;
            count++;
        }
        analogMeans.aX = analogSums.ax / read_counter;
        analogs_in[0] = mapStickVals(stickCalVals.AX, sortedStickCalVals.AX, stickDeadzVals.AX, analogMeans.aX);
        analogSums.ax = 0;

        analogMeans.aY = analogSums.ay / read_counter;
        analogs_in[1] = mapStickVals(stickCalVals.AY, sortedStickCalVals.AY, stickDeadzVals.AY, analogMeans.aY);
        analogSums.ay = 0;

        analogMeans.cX = analogSums.cx / read_counter;
        analogs_in[2] = mapStickVals(stickCalVals.CX, sortedStickCalVals.CX, stickDeadzVals.CX, analogMeans.cX);
        analogSums.cx = 0;

        analogMeans.cY = analogSums.cy / read_counter;
        analogs_in[3] = mapStickVals(stickCalVals.CY, sortedStickCalVals.CY, stickDeadzVals.CY, analogMeans.cY);
        analogSums.cy = 0;

        analogMeans.aL = analogSums.al / read_counter;
        //  analogs_in[4] = mapTriggerVals(LTHigh,LTLow,aL);
        analogs_in[4] = analogMeans.aL;
        analogSums.al = 0;

        analogMeans.aR = analogSums.ar / read_counter;
        //  analogs_in[5] = mapTriggerVals(RTHigh,RTLow,aR);
        analogs_in[5] = analogMeans.aR;
        analogSums.ar = 0;

        int analog_value = 0;
        for (int k = 0; k < AnalogInLen; k++){
            analog_value = analogs_in[k];
            for (int i = 0; i < 7; i = i + 2){
                j = i + 1;
                val1 = bitRead(analog_value, 7 - i);
                val2 = bitRead(analog_value, 7 - j);
                next_val = twoBitsToMessage[{val1, val2}];
                InGameReply[count] = next_val;
                count++;
            }
        }
    }
 
    /**
     * @brief Switch statement that compares the command byte to known commands.
     *        Based on the command bytes value a different message is sent.
     *        The uart buffer is then cleared as to not have a read of the sent data occur
     */
    void choose_and_reply(){
        to_int();
        switch (command_byte){
        case 0b00000000:
            if (stop_bit_9){
                writeData(ProbeReply, ProbeReplyLen);
                Serial2.flush();
                clearUARTrx();
            }
            break;
        case 0b01000001:
            if (stop_bit_9){
                writeData(InGameReply, InGameReplyLen - 1);
                writeData(OriginEnd, OriginEndLen);
                Serial2.flush();
                clearUARTrx();
            }
            break;
        case 0b01000000:
            writeData(InGameReply, InGameReplyLen);
            timer_flag = 0;
            Serial2.flush();
            clearUARTrx();
            break;
        default:
            break;
        }
    }

    /**
     * @brief checks the physical pins (5,19,22,21) which map to physical buttons (L,R,A,Start) respectively
     *        if they are all high the sent message is changed to be only those 4 buttons clicked
     */
    void checkForLRAStart(){
        int LRAStart = 0;
        LRAStart += !digitalRead(charToPinMap['L']);
        LRAStart += !digitalRead(charToPinMap['R']);
        LRAStart += !digitalRead(charToPinMap['A']);
        LRAStart += !digitalRead(charToPinMap['S']);
        // if they were all pressed change buttons_in array to be exactly those 4 buttons pushed
        if (LRAStart == 4){ 
            for (int i = 0; i < DigitalInLen; i++){
                if (i == START || i == A || i == L || i == R || i == 8){
                    buttons_in[i] = 1;
                }
                else{
                    buttons_in[i] = 0;
                }
            }
        }
    }

    /**
     * @brief writes the current stick calibration values
     */
    void writeStickCalToMem(){
        preferences.begin("AnalogCal", false);
        // uint16_t toSave;

        for (int i = 0; i <= numCalibPoints; i++) {
            String id = String(i);
            String axID = "AX" + id;
            String ayID = "AY" + id;
            String cxID = "CX" + id;
            String cyID = "CY" + id;

            preferences.putUShort(axID.c_str(), (uint16_t)stickCalVals.AX[i]);
            preferences.putUShort(ayID.c_str(), (uint16_t)stickCalVals.AY[i]);
            preferences.putUShort(cxID.c_str(), (uint16_t)stickCalVals.CX[i]);
            preferences.putUShort(cyID.c_str(), (uint16_t)stickCalVals.CY[i]);
        }
        
        preferences.end();
    }

    /**
     * @brief writes the current stick deadzone values to memory
     */
    void writeStickDeadzonesToMem(){
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

    /**
     * @brief write the current button mapping to memory
     */
    void writeButtonMappingToMem(){
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

    /**
     * @brief write BLEpassword to memory
     */
    void writeBLEpasswordToMem(String newPass){
        preferences.begin("Passwords", false);
        preferences.putString("BLEPass", newPass);
        preferences.end();
        BLEpassword = newPass;
    }
    
    /**
     * @brief write BLEName to memory
     */
    void writeBLEName(String newName){
        preferences.begin("Passwords", false);
        preferences.putString("BLEName", newName);
        preferences.end();
    }

    /**
     * @brief Reads the stick deadzone values from memory
     */
    void readStickDeadzonesFromMem(){
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

    /**
     * @brief reads the stick calibration values from memory
     */
    void readStickCalFromMem(){
        preferences.begin("AnalogCal", true);

        for (int i = 0; i <= numCalibPoints; i++) {
            String id = String(i);
            String axID = "AX" + id;
            String ayID = "AY" + id;
            String cxID = "CX" + id;
            String cyID = "CY" + id;
            
            stickCalVals.AX[i] = preferences.getUShort(axID.c_str(), 0);
            sortedStickCalVals.AX[i] = preferences.getUShort(axID.c_str(), 0);
            
            stickCalVals.AY[i] = preferences.getUShort(ayID.c_str(), 0);
            sortedStickCalVals.AY[i] = preferences.getUShort(axID.c_str(), 0);
            
            stickCalVals.CX[i] = preferences.getUShort(cxID.c_str(), 0);
            sortedStickCalVals.CX[i] = preferences.getUShort(axID.c_str(), 0);
            
            stickCalVals.CY[i] = preferences.getUShort(cyID.c_str(), 0);
            sortedStickCalVals.CY[i] = preferences.getUShort(axID.c_str(), 0);

        }
        preferences.end();
        sortStickCalVals();
    }

    /**
     * @brief load digital_mapping and digital_toggling arrays with saved mapping from memory
     *
     */
    void readButtonMappingFromMem(){
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
       
    /**
     * @brief read BLEpassword from memory
     */
    void readBLEpasswordFromMem(){
        preferences.begin("Passwords", true);
        BLEpassword = preferences.getString("BLEPass", "KEG CONCH");
        preferences.end();
    }

    /**
     * @brief read BLEName from memory
     */
    void readBLENameFromMem(){
        preferences.begin("Passwords", true);
        bluetooth.bleServerName = preferences.getString("BLEName", "KEG_BLE").c_str();
        preferences.end();
    }
    

    /**
     * @brief updates digital_pins array with currently stored mapping
     */
    void updateDigitalInputPins(){
        for (int i = 0; i < DigitalInLen; i++){
            if (digital_toggling[i] == 0){
                digital_pins[i] = -1;
            }
            else{
                digital_pins[i] = digital_mapping[i];
            }
        }
    }

    /**
     * @brief loop through btnMappingStr and update pins
     */
    void ParseButtonMappingString(String btnMappingStr){
        int pin;
        int toggle;
        char read_val_1;
        char read_val_2;

        for (int i = 0; i < ButtonMappingMSGLen; i += 3){
            read_val_1 = btnMappingStr.charAt(i);
            read_val_2 = btnMappingStr.charAt(i + 1);
            pin = charToPinMap[read_val_1];
            toggle = getToggleFromChar(read_val_2);
            digital_mapping[buttonMappingMap[i]] = pin;
            digital_toggling[buttonMappingMap[i]] = toggle;
        }
        updateDigitalInputPins();
    }

    /**
     * @brief parses the calibration string and fills the stick calibration struct with the new values.
     *        New values stored in struct but not saved to memory unless user chooses to
     * calvals.ax = {west, n1, n2, neutch....}
     */
    void ParseCalibrationString(String str){
        for (int i = 0; i <= 30; i+=5) {
            stickCalVals.AX[i / 5] = str.substring(i, i+4).toInt();
            sortedStickCalVals.AX[i / 5] = str.substring(i, i+4).toInt();
        }
        for (int i = 35; i <= 65; i+=5) {
            stickCalVals.AY[(i-35) / 5] = str.substring(i, i+4).toInt();
            sortedStickCalVals.AY[(i-35) / 5] = str.substring(i, i+4).toInt();
        }
        for (int i = 70; i <= 100; i+=5) {
            stickCalVals.CX[(i-70) / 5] = str.substring(i, i+4).toInt();
            sortedStickCalVals.CX[(i-70) / 5] = str.substring(i, i+4).toInt();
        }
        for (int i = 105; i <= 135; i+=5) {
            stickCalVals.CY[(i-105) / 5] = str.substring(i, i+4).toInt();
            sortedStickCalVals.CY[(i-105) / 5] = str.substring(i, i+4).toInt();
        }
        sortStickCalVals();
    }
    /**
     * @brief Parses deadzone string and fills stick deadzone struct with new values.
     *        Values stored but not saved to memory unless user chooses
     */
    void ParseDeadzoneString(String str){
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

    /**
     * @brief Handles all bluetooth requests coming from the received message
     */
    void BLEHandler(){
        if (bluetoothConnected){
            String receivedMSG = bluetooth.getMessageString();
            char fifthChar = receivedMSG[4];
            char fourthChar = receivedMSG[3];
            char thirdChar = receivedMSG[2];
            int isX = !digitalRead(charToPinMap['X']);
            int isY = !digitalRead(charToPinMap['Y']);
            char firstChar = receivedMSG[0];

            if (resetPasswordFlag == 1){
                if (firstChar == 'P' && passWriteFlag == 0){
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
                return;
            }
            
            if (password_correct == 0){
                // this is just received msg?
                // if receivedMSG == BLEpassword && isX && isY)
                if ((String)Ch2.getValue().c_str() == BLEpassword && isX && isY){
                    password_correct = 1;
                    Ch1.setValue("Password Correct");
                    Ch1.notify();
                }
                else{
                    Ch1.setValue("Password Incorrect");
                    Ch1.notify();
                }
                return;
            }

            if (receivedMSG == "A"){ // A means requesting Analog data
                char AnalogMSG[19] = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
                sprintf(AnalogMSG, "%04d,%04d,%04d,%04d,%04d,%04d", analogMeans.aX, analogMeans.aY,
                        analogMeans.cX, analogMeans.cY,
                        analogMeans.aL, analogMeans.aR);
                Ch1.setValue(AnalogMSG);
                Ch1.notify();
            }
            else if (fifthChar == ','){
                ParseCalibrationString(receivedMSG);
            }
            else if (receivedMSG == "SAC" && savedCalib == 0){ // SAC means Save Analog Calibration Values
                writeStickCalToMem();
                writeStickDeadzonesToMem();
                savedCalib = 1;
            }
            else if (receivedMSG == "RAC"){
                char AnalogCalibMSG[179];
                sprintf(AnalogCalibMSG,
                        "%04d,%04d,%04d,%04d,%04d,%04d,%04d:%04d,%04d,%04d,%04d,%04d,%04d,%04d:%04d,%04d,%04d,%04d,%04d,%04d,%04d:%04d,%04d,%04d,%04d,%04d,%04d,%04d",
                        stickCalVals.AX[0], stickCalVals.AX[1], stickCalVals.AX[2], stickCalVals.AX[3],
                        stickCalVals.AX[4], stickCalVals.AX[5], stickCalVals.AX[6],
                        stickCalVals.AY[0], stickCalVals.AY[1], stickCalVals.AY[2], stickCalVals.AY[3],
                        stickCalVals.AY[4], stickCalVals.AY[5], stickCalVals.AY[6],
                        stickCalVals.CX[0], stickCalVals.CX[1], stickCalVals.CX[2], stickCalVals.CX[3],
                        stickCalVals.CX[4], stickCalVals.CX[5], stickCalVals.CX[6], 
                        stickCalVals.CY[0], stickCalVals.CY[1], stickCalVals.CY[2], stickCalVals.CY[3],
                        stickCalVals.CY[4], stickCalVals.CY[5], stickCalVals.CY[6]
                        );
                Ch1.setValue(AnalogCalibMSG);
                Ch1.notify();
            }
            else if (fourthChar == ','){
                ParseDeadzoneString(receivedMSG);
            }
            else if (receivedMSG == "SSD" && savedDeadzones == 0){ // SSD = Save Stick Deadzones
                writeStickDeadzonesToMem();
                savedDeadzones = 1;
            }
            else if (thirdChar == '.'){
                ParseButtonMappingString(receivedMSG);
            }
            else if (receivedMSG == "RBM"){
                fillDigitalMappingMessage();
                Ch1.setValue(DigitalMappingMSG);
                Ch1.notify();
            }
            else if (receivedMSG == "SBM" && savedButtonMapping == 0){
                writeButtonMappingToMem();
                savedButtonMapping = 1;
            }
            else if (firstChar == 'W' && wifi_flag == 0){
                wifiUploadEnabled(receivedMSG);
                Ch1.setValue(ipAddy);
                Ch1.notify();
            }
            else if (firstChar == 'P' && passWriteFlag == 0){
                writeBLEpasswordToMem(receivedMSG.substring(1));
                passWriteFlag = 1;
            }
            else if (receivedMSG == "RDC"){
                char AnalogDeadzoneMSG[47];
                sprintf(AnalogDeadzoneMSG, "%03d,%03d,%03d:%03d,%03d,%03d:%03d,%03d,%03d:%03d,%03d,%03d",
                        stickDeadzVals.AX[0], stickDeadzVals.AX[1], stickDeadzVals.AX[2],
                        stickDeadzVals.AY[0], stickDeadzVals.AY[1], stickDeadzVals.AY[2],
                        stickDeadzVals.CX[0], stickDeadzVals.CX[1], stickDeadzVals.CX[2],
                        stickDeadzVals.CY[0], stickDeadzVals.CY[1], stickDeadzVals.CY[2]
                );
                Ch1.setValue(AnalogDeadzoneMSG);
                Ch1.notify();
            }
            else if (firstChar == 'B' && BLENameWriteFlag == 0){
                writeBLEName(receivedMSG.substring(1));
                BLENameWriteFlag = 1;
            }            
        }
    }

    /**
     * @brief converts the first 8 GameCube bits of data to a command byte and also
     *        sets stop_bit_9 which tells us if the 9th GameCube bit was a stop bit or not
     */
    void to_int(){
        int j = 0;
        stop_bit_9 = 0;

        for (int i = 0; i < 4; i++){
            j = i * 2;
            auto msg = buffer_holder[i];
            auto twobits = messageToTwoBits[msg];
            bitWrite(command_byte, 8 - j - 1, twobits.first);
            bitWrite(command_byte, 8 - j - 2, twobits.second);

        }
        if (buffer_holder[4] == STOP){
            stop_bit_9 = 1;
        }
    }
};

KEGController controller;

void setup(){
    controller.setup();
}

void loop(){
    controller.loop();
}