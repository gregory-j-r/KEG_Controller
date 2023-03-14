#pragma once

#include <stdlib.h>
#include <Arduino.h>

/** Bluetooth Setup */
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_2 "da1e7d98-916b-11ed-a1eb-0242ac120002"

BLECharacteristic Ch1(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
BLEDescriptor Dp1(BLEUUID((uint16_t)0x2902));
BLECharacteristic Ch2(CHARACTERISTIC_UUID_2, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
BLEDescriptor Dp2(BLEUUID((uint16_t)0x2902));

bool bluetoothConnected = false;
int password_correct = 0;

class MyServerCallbacks : public BLEServerCallbacks{

    void onConnect(BLEServer *pServer){
        bluetoothConnected = true;
    }
    void onDisconnect(BLEServer *pServer){
        bluetoothConnected = false;
        password_correct = 0; // reset for next connection
        BLEDevice::startAdvertising();
    }
};

class KEGBluetooth{
public:
    std::string bleServerName;

    void setupBLE(unsigned long &bt_millis_count, int resetPasswordFlag){

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

    String getMessageString(){
        String message = (String)Ch2.getValue().c_str();
        return message;
    }
};

/** Wifi - Wireless Upload */

WebServer server(80);
char ipAddy[16];
const char *host = "esp32";
int wifi_flag = 0;

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

void wifiUploadEnabled(String Message){
    char *token;
    char *mystring = (char *)Message.c_str();
    const char *delimiter = "/";
    int i = 0;
    char *ssid;
    char *password;
    ipAddy[0] = 'N';
    ipAddy[1] = 'O';

    token = strtok(mystring, delimiter);
    while (token != NULL){
        //      Serial.println(token);
        switch (i){
        case 1:
            ssid = token;
            break;
        case 2:
            password = token;
            break;
        }
        token = strtok(NULL, delimiter);
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
    while (WiFi.status() != WL_CONNECTED){
        delay(500);
        Serial.print(".");
        i++;
        if (i >= 100){
            break;
        }
    }

    if (i < 100){
        Serial.println("");
        Serial.print("Connected to ");
        Serial.println(ssid);
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());

        /*use mdns for host name resolution*/
        if (!MDNS.begin(host)){ // http://esp32.local
            Serial.println("Error setting up MDNS responder!");
            while (1){
                delay(1000);
            }
        }
        Serial.println("mDNS responder started");
        server.on("/", HTTP_GET, [](){
            server.sendHeader("Connection", "close");
            server.send(200, "text/html", serverIndex); 
            }
        );
        /*handling uploading firmware file */
        server.on("/update", HTTP_POST, [](){
            server.sendHeader("Connection", "close");
            server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
            ESP.restart(); 
            }, [](){
                HTTPUpload &upload = server.upload();
                if (upload.status == UPLOAD_FILE_START){
                    Serial.printf("Update: %s\n", upload.filename.c_str());
                    if (!Update.begin(UPDATE_SIZE_UNKNOWN)){ // start with max available size
                        Update.printError(Serial);
                    }
                }
                else if (upload.status == UPLOAD_FILE_WRITE){
                    /* flashing firmware to ESP*/
                    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize){
                        Update.printError(Serial);
                    }
                }
                else if (upload.status == UPLOAD_FILE_END){
                    if (Update.end(true)){ // true to set the size to the current progress
                        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
                    }
                    else{
                        Update.printError(Serial);
                    }
                }
                }
        );
        server.begin();
        wifi_flag = 1;
        WiFi.localIP().toString().toCharArray(ipAddy, 15);
    }
}