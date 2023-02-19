#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define DigitalInLen 16
#define AnalogInLen 6

#define ProbeReplyLen 13
#define InGameReplyLen 33

#define OriginEndLen 9

#define ProbeLen 5
#define OriginLen 5
#define PollLen 13

#define bleServerName "KEG_V1_4_G"
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_2 "da1e7d98-916b-11ed-a1eb-0242ac120002"


// Some BLE setup

BLECharacteristic Ch1(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
BLEDescriptor Dp1(BLEUUID((uint16_t)0x2902));
BLECharacteristic Ch2(CHARACTERISTIC_UUID_2, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
BLEDescriptor Dp2(BLEUUID((uint16_t)0x2902));

char AnalogMSG[19] = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
char AnalogCalibMSG[59];
char AnalogDeadzoneMSG[47];
char DigitalMappingMSG[35];
char ipAddy[16];

String BLEpassword;
String receivedMSG;
bool deviceConnected = false;
int password_correct = 0;

// Setup callbacks onConnect and onDisconnect
class MyServerCallbacks : public BLEServerCallbacks
{
	void onConnect(BLEServer *pServer)
	{
		deviceConnected = true;
	};
	void onDisconnect(BLEServer *pServer)
	{
		deviceConnected = false;
		password_correct = 0; // reset for next connection
	}
};
// End of BLE Setup


/**
 * @brief This class defines the code to setup and run a KEG controller.
 * 
 * @note The naming convention is:
 *          - left-hand stick is the analog (a) stick,
 *          - right hand is the c-stick (c).
 *          - Both have x and y axes: ax, ay, cx, cy 
 *          - The triggers are analog right/left: ar, al
 *          
 *          - The case is just to differentiate them in names
 */
class KEGController
{

public:
    // TODO
    void setup();
    
    // TODO
    void loop();

private:

    // some BLE flags
    unsigned long bt_millis_count;
    int resetPasswordFlag = 0;
    int passWriteFlag = 0;
    int savedCalib = 0;
    int savedDeadzones = 0;
    int savedButtonMapping = 0;

    /**
     * @brief Struct to hold the analog stick calibration params
     */
    struct StickCalibrationValues
    {
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
    } stickCalVals;

    /**
     * @brief Sruct to hold the analog stick deadzone values
     */
    struct StickDeadzoneValues
    {
        int AXDeadzone[3];
        int AYDeadzone[3];
        int CXDeadzone[3];
        int CYDeadzone[3];
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
    struct AveragedAnalogReads{
        int aX;
        int aY;
        int cX;
        int cY;
        int aL;
        int aR;

    } analogMeans;

    enum TwoBitMessage
    {
        ZeroZero = 4,
        OneOne = 55,
        ZeroOne = 52,
        OneZero = 7,
        STOP = 63
    };

    std::unordered_map<std::pair<int, int>, int> twoBitsToMessage = {{{0,0}, ZeroZero}, {{1,1}, OneOne},
                                                          {{0,1}, ZeroOne}, {{1,0}, OneZero}};
    

    // NOTE: enums to index these arrays?
    // Arrays holding pins of each digital and analog input
    int digital_input_pins[DigitalInLen] = {-1, -1, -1, 21, 4, 18, 23, 22, -1, 5, 19, 15, 32, 25, 33, 14}; // should save pin 2 to use for rumble (it's connected to the onboard LED), pin 2 on the devboard is attatched to a pull down resistor, Note gpio5 is connected to a pull up resistor
    int analog_input_pins[AnalogInLen] = {34, 39, 35, 13, 26, 27};										   // maybe change 12 for 36 (aka vp) or 39 (aka vn) as 12 needs to be low during boot but currently is not??

    // Arrays to hold current digital toggling and mapping of buttons
    int digital_mapping[DigitalInLen] = {-1, -1, -1, 21, 4, 18, 23, 22, -1, 5, 19, 15, 32, 25, 33, 14};
    int digital_toggling[DigitalInLen] = {0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1};

    // Arrays to hold digital (button) and analog inputs from controller
    int buttons_in[DigitalInLen] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0}; // 0 0 0 Start Y X B A 1 L R Z D-U D-D D-R D-L
    int analogs_in[AnalogInLen] = {128, 128, 128, 128, 0, 0};						 // Analog-X Analog-Y C-Stick-X C-Stick-Y L-Trigger R-Trigger

    // Array to hold in-game reply  (uses buttons_in and analogs_in to fill)
    int InGameReply[InGameReplyLen] = {ZeroZero, ZeroZero, ZeroZero, ZeroZero, // 0 0 0 Start Y X B A
                                    OneZero, ZeroZero, ZeroZero, ZeroZero,  // 1 L R Z D-U D-D D-R D-L
                                    OneZero, ZeroZero, ZeroZero, ZeroZero,  // Analog X
                                    OneZero, ZeroZero, ZeroZero, ZeroZero,  // Analog Y
                                    OneZero, ZeroZero, ZeroZero, ZeroZero,  // C-Stick X
                                    OneZero, ZeroZero, ZeroZero, ZeroZero,  // C-Stick Y
                                    ZeroZero, ZeroZero, ZeroZero, ZeroZero, // L-Trigger
                                    ZeroZero, ZeroZero, ZeroZero, ZeroZero, // R-Trigger
                                    STOP};
    
    // How many times to read inputs before averaging
    int read_counter1 = 0;


    //////// /* Private Methods*/ ////////

    // The task called on core 0
    // Calls the function to update the input state arrays and delays as to not setoff a watchdog timeout
    void read_inputs(void *parameter)
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
            if (digital_input_pins[i] != -1)
            {
                buttons_in[i] = !digitalRead(digital_input_pins[i]);
            }
        }

        int analogX = adc1_get_raw(ADC1_CHANNEL_6);
        analogSums.ax += analogX; /// 16;

        int analogY = adc1_get_raw(ADC1_CHANNEL_3);
        analogSums.ay += analogY; /// 16;

        int analogCX = adc1_get_raw(ADC1_CHANNEL_7);
        analogSums.cx += analogCX;

        // Q: Why is adc2 different like this? Why pass as reference?
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
            next_val = TwoBitsToMessage(val1, val2);
            InGameReply[count] = next_val;
            count++;
        }
        analogMeans.aX = analogSums.ax / read_counter1;
        analogs_in[0] = mapStickVals(stickCalVals.AXLow, stickCalVals.AXHigh,
                                     stickCalVals.AXNeutch, analogMeans.aX, stickDeadzVals.AXDeadzone);
        analogSums.ax = 0;
        
        analogMeans.aY = analogSums.ay / read_counter1;
        analogs_in[1] = mapStickVals(stickCalVals.AYLow, stickCalVals.AYHigh,
                                     stickCalVals.AYNeutch, analogMeans.aY, stickDeadzVals.AYDeadzone);
        analogSums.ay = 0;
        
        analogMeans.cX = analogSums.cx / read_counter1;
        analogs_in[2] = mapStickVals(stickCalVals.CXLow, stickCalVals.CXHigh,
                                     stickCalVals.CXNeutch, analogMeans.cX, stickDeadzVals.CXDeadzone);;
        analogSums.cx = 0;
        
        analogMeans.cY = analogSums.cy / read_counter1;
        analogs_in[3] = mapStickVals(stickCalVals.CYLow, stickCalVals.CYHigh,
                                     stickCalVals.CYNeutch, analogMeans.cY, stickDeadzVals.CYDeadzone);
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

    // checks the physical pins (5,19,22,21) which map to physical buttons (L,R,A,Start) respectively
    // if they are all high the sent message is changed to be only those 4 buttons clicked
    void checkForLRAStart()
    {
        // maybe make all the pins and indexes named enums or constants
        int LRAStart = 0;
        LRAStart += !digitalRead(5);
        LRAStart += !digitalRead(19);
        LRAStart += !digitalRead(22);
        LRAStart += !digitalRead(21);
        if (LRAStart == 4)
        { // if they were all pressed change buttons_in array to be exactly those 4 buttons pushed
            for (int i = 0; i < DigitalInLen; i++)
            {
                if (i == 3 || i == 7 || i == 9 || i == 10 || i == 8)
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

    // parses the calibration string and fills the stick calibration struct with the new values.
    // New values stored in struct but not saved to memory unless user chooses to
    void ParseCalibrationString(String str, )
    {
        stickCalVals.AXNeutch = str.substring(0, 4).toInt();
        stickCalVals.AXLow = str.substring(5, 9).toInt();
        stickCalVals.AXHigh = str.substring(10, 14).toInt();

        stickCalVals.AYNeutch = str.substring(15, 19).toInt();
        stickCalVals.AYLow = str.substring(20, 24).toInt();
        stickCalVals.AYHigh = str.substring(25, 29).toInt();

        stickCalVals.CXNeutch = str.substring(30, 34).toInt();
        stickCalVals.CXLow = str.substring(35, 39).toInt();
        stickCalVals.CXHigh = str.substring(40, 44).toInt();

        stickCalVals.CYNeutch = str.substring(45, 49).toInt();
        stickCalVals.CYLow = str.substring(50, 54).toInt();
        stickCalVals.CYHigh = str.substring(55, 59).toInt();
    }
    
    // parses deadzone string and fills stick deadzone struct with new values.
    // similarly not saved to memory until user chooses to
    void ParseDeadzoneString(String str)
    {
        stickDeadzVals.AXDeadzone[0] = str.substring(0, 3).toInt();
        stickDeadzVals.AXDeadzone[1] = str.substring(4, 7).toInt();
        stickDeadzVals.AXDeadzone[2] = str.substring(8, 11).toInt();

        stickDeadzVals.AYDeadzone[0] = str.substring(12, 15).toInt();
        stickDeadzVals.AYDeadzone[1] = str.substring(16, 19).toInt();
        stickDeadzVals.AYDeadzone[2] = str.substring(20, 23).toInt();

        stickDeadzVals.CXDeadzone[0] = str.substring(24, 27).toInt();
        stickDeadzVals.CXDeadzone[1] = str.substring(28, 31).toInt();
        stickDeadzVals.CXDeadzone[2] = str.substring(32, 35).toInt();

        stickDeadzVals.CYDeadzone[0] = str.substring(36, 39).toInt();
        stickDeadzVals.CYDeadzone[1] = str.substring(40, 43).toInt();
        stickDeadzVals.CYDeadzone[2] = str.substring(44, 47).toInt();
    }

    // Handles all bluetooth requests coming from the received message
    void BLEHandler()
    {
        if (deviceConnected)
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
                                                                            analogMeans.aL,  analogMeans.aR);
                        Ch1.setValue(AnalogMSG);
                        Ch1.notify();
                    }
                    else if (fifthChar == ',')
                    {
                        //        Serial.println("Parsing");
                        ParseCalibrationString(receivedMSG);
                        //        Serial.println("Done Parsing");
                    }
                    else if (receivedMSG == "SAC" && savedCalib == 0)
                    { // SAC means Save Analog Calibration Values
                        //          Serial.println("Saving");
                        writeStickCalToMem();
                        writeStickDeadzonesToMem();
                        savedCalib = 1;
                        //          Serial.println("Done Saving");
                    }
                    else if (receivedMSG == "RAC")
                    {
                        sprintf(AnalogCalibMSG, "%04d,%04d,%04d:%04d,%04d,%04d:%04d,%04d,%04d:%04d,%04d,%04d", AXNeutch, AXLow, AXHigh, AYNeutch, AYLow, AYHigh, CXNeutch, CXLow, CXHigh, CYNeutch, CYLow, CYHigh);
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
                        //                Serial.println("Would be saving deadzones");
                        savedDeadzones = 1;
                    }
                    else if (thirdChar == '.')
                    {
                        ParseButtonMappingString(receivedMSG);
                    }
                    else if (receivedMSG == "RBM")
                    {
                        fillDigitalMappingMessage();
                        Ch1.setValue(DigitalMappingMSG);
                        Ch1.notify();
                    }
                    else if (receivedMSG == "SBM" && savedButtonMapping == 0)
                    {
                        writeButtonMappingToMem();
                        //                      Serial.println("Would be saving button mapping");
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
                        //                              Serial.println(receivedMSG.substring(1));
                        writeBLEpasswordToMem(receivedMSG.substring(1));
                        passWriteFlag = 1;
                    }
                    else if (receivedMSG == "RDC")
                    {
                        sprintf(AnalogDeadzoneMSG, "%03d,%03d,%03d:%03d,%03d,%03d:%03d,%03d,%03d:%03d,%03d,%03d", AXDeadzone[0], AXDeadzone[1], AXDeadzone[2], AYDeadzone[0], AYDeadzone[1], AYDeadzone[2], CXDeadzone[0], CXDeadzone[1], CXDeadzone[2], CYDeadzone[0], CYDeadzone[1], CYDeadzone[2]);
                        Ch1.setValue(AnalogDeadzoneMSG);
                        Ch1.notify();
                    }
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


    // writes the current stick calibration values to memory
    void writeStickDeadzonesToMem()
    {
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
    
    // writes the current button mapping to memory
    void writeButtonMappingToMem()
    {
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

        stickDeadzVals.AXDeadzone[0] = preferences.getUShort("AXDeadLow", 117);
        stickDeadzVals.AXDeadzone[1] = preferences.getUShort("AXDeadHigh", 137);
        stickDeadzVals.AXDeadzone[2] = preferences.getUShort("AXDeadVal", 127);

        stickDeadzVals.AYDeadzone[0] = preferences.getUShort("AYDeadLow", 117);
        stickDeadzVals.AYDeadzone[1] = preferences.getUShort("AYDeadHigh", 137);
        stickDeadzVals.AYDeadzone[2] = preferences.getUShort("AYDeadVal", 127);

        stickDeadzVals.CXDeadzone[0] = preferences.getUShort("CXDeadLow", 117);
        stickDeadzVals.CXDeadzone[1] = preferences.getUShort("CXDeadHigh", 137);
        stickDeadzVals.CXDeadzone[2] = preferences.getUShort("CXDeadVal", 127);

        stickDeadzVals.CYDeadzone[0] = preferences.getUShort("CYDeadLow", 117);
        stickDeadzVals.CYDeadzone[1] = preferences.getUShort("CYDeadHigh", 137);
        stickDeadzVals.CYDeadzone[2] = preferences.getUShort("CYDeadVal", 127);

        preferences.end();
    }

    // reads the stick halibration values from memory
    void readStickCalFromMem()
    {
        preferences.begin("AnalogCal", true);

        stickCalVals.AXNeutch = preferences.getUShort("AXNeutch", 0); // if key not there default to 0
        stickCalVals.AXHigh = preferences.getUShort("AXHigh", 0);
        stickCalVals.AXLow = preferences.getUShort("AXLow", 0);

        stickCalVals.AYNeutch = preferences.getUShort("AYNeutch", 0);
        stickCalVals.AYHigh = preferences.getUShort("AYHigh", 0);
        stickCalVals.AYLow = preferences.getUShort("AYLow", 0);

        stickCalVals.CXNeutch = preferences.getUShort("CXNeutch", 0);
        stickCalVals.CXHigh = preferences.getUShort("CXHigh", 0);
        stickCalVals.CXLow = preferences.getUShort("CXLow", 0);

        stickCalVals.CYNeutch = preferences.getUShort("CYNeutch", 0);
        stickCalVals.CYHigh = preferences.getUShort("CYHigh", 0);
        stickCalVals.CYLow = preferences.getUShort("CYLow", 0);

        preferences.end();
    }

    // 0 0 0 Start Y X B A 1 L R Z D-U D-D D-R D-L
    //{-1,-1,-1,21,4,18,23,22,-1,5,19,15,32,25,33,14};
    void readButtonMappingFromMem()
    {
        preferences.begin("DigitalMap", true);

        digital_mapping[7] = preferences.getUShort("A", 22);
        digital_toggling[7] = preferences.getUShort("AT", 1);

        digital_mapping[6] = preferences.getUShort("B", 23);
        digital_toggling[6] = preferences.getUShort("BT", 1);

        digital_mapping[3] = preferences.getUShort("S", 21);
        digital_toggling[3] = preferences.getUShort("ST", 1);

        digital_mapping[5] = preferences.getUShort("X", 18);
        digital_toggling[5] = preferences.getUShort("XT", 1);

        digital_mapping[4] = preferences.getUShort("Y", 4);
        digital_toggling[4] = preferences.getUShort("YT", 1);

        digital_mapping[11] = preferences.getUShort("Z", 15);
        digital_toggling[11] = preferences.getUShort("ZT", 1);

        digital_mapping[9] = preferences.getUShort("L", 5);
        digital_toggling[9] = preferences.getUShort("LT", 1);

        digital_mapping[10] = preferences.getUShort("R", 19);
        digital_toggling[10] = preferences.getUShort("RT", 1);

        digital_mapping[12] = preferences.getUShort("u", 32);
        digital_toggling[12] = preferences.getUShort("uT", 1);

        digital_mapping[14] = preferences.getUShort("r", 33);
        digital_toggling[14] = preferences.getUShort("rT", 1);

        digital_mapping[13] = preferences.getUShort("d", 25);
        digital_toggling[13] = preferences.getUShort("dT", 1);

        digital_mapping[15] = preferences.getUShort("l", 14);
        digital_toggling[15] = preferences.getUShort("lT", 1);

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
        int temp = digital_input_pins[ind1];
        digital_input_pins[ind1] = digital_input_pins[ind2];
        digital_input_pins[ind2] = temp;
    }

    // updates digital inputs with their currently mapped values
    void updateDigitalInputPins()
    {
        for (int i = 0; i < DigitalInLen; i++)
        {
            if (digital_toggling[i] == 0)
            {
                digital_input_pins[i] = -1;
            }
            else
            {
                digital_input_pins[i] = digital_mapping[i];
            }
        }
    }

    // rename to updateButtonMapping(String btn_mapping_str)
    void ParseButtonMappingString(String str)
    {
        int pin;
        int toggle;
        String read_val_1;
        String read_val_2;

        read_val_1 = str.substring(0, 1);
        read_val_2 = str.substring(1, 2);
        pin = getPinFromChar(read_val_1);
        toggle = getToggleFromChar(read_val_2);
        digital_mapping[7] = pin;
        digital_toggling[7] = toggle;

        read_val_1 = str.substring(3, 4);
        read_val_2 = str.substring(4, 5);
        pin = getPinFromChar(read_val_1);
        toggle = getToggleFromChar(read_val_2);
        digital_mapping[6] = pin;
        digital_toggling[6] = toggle;

        read_val_1 = str.substring(6, 7);
        read_val_2 = str.substring(7, 8);
        pin = getPinFromChar(read_val_1);
        toggle = getToggleFromChar(read_val_2);
        digital_mapping[3] = pin;
        digital_toggling[3] = toggle;

        read_val_1 = str.substring(9, 10);
        read_val_2 = str.substring(10, 11);
        pin = getPinFromChar(read_val_1);
        toggle = getToggleFromChar(read_val_2);
        digital_mapping[5] = pin;
        digital_toggling[5] = toggle;

        read_val_1 = str.substring(12, 13);
        read_val_2 = str.substring(13, 14);
        pin = getPinFromChar(read_val_1);
        toggle = getToggleFromChar(read_val_2);
        digital_mapping[4] = pin;
        digital_toggling[4] = toggle;

        read_val_1 = str.substring(15, 16);
        read_val_2 = str.substring(16, 17);
        pin = getPinFromChar(read_val_1);
        toggle = getToggleFromChar(read_val_2);
        digital_mapping[11] = pin;
        digital_toggling[11] = toggle;

        read_val_1 = str.substring(18, 19);
        read_val_2 = str.substring(19, 20);
        pin = getPinFromChar(read_val_1);
        toggle = getToggleFromChar(read_val_2);
        digital_mapping[9] = pin;
        digital_toggling[9] = toggle;

        read_val_1 = str.substring(21, 22);
        read_val_2 = str.substring(22, 23);
        pin = getPinFromChar(read_val_1);
        toggle = getToggleFromChar(read_val_2);
        digital_mapping[10] = pin;
        digital_toggling[10] = toggle;

        read_val_1 = str.substring(24, 25);
        read_val_2 = str.substring(25, 26);
        pin = getPinFromChar(read_val_1);
        toggle = getToggleFromChar(read_val_2);
        digital_mapping[12] = pin;
        digital_toggling[12] = toggle;

        read_val_1 = str.substring(27, 28);
        read_val_2 = str.substring(28, 29);
        pin = getPinFromChar(read_val_1);
        toggle = getToggleFromChar(read_val_2);
        digital_mapping[14] = pin;
        digital_toggling[14] = toggle;

        read_val_1 = str.substring(30, 31);
        read_val_2 = str.substring(31, 32);
        pin = getPinFromChar(read_val_1);
        toggle = getToggleFromChar(read_val_2);
        digital_mapping[13] = pin;
        digital_toggling[13] = toggle;

        read_val_1 = str.substring(33, 34);
        read_val_2 = str.substring(34, 35);
        pin = getPinFromChar(read_val_1);
        toggle = getToggleFromChar(read_val_2);
        digital_mapping[15] = pin;
        digital_toggling[15] = toggle;

        updateDigitalInputPins();
    }
};

