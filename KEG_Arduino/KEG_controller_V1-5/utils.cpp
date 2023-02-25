#include "utils.h"


extern portMUX_TYPE timerMux;

/**
 * @brief Writes data over 6 bit uart line serial 2
 *        Absolutely crucial that this function have the portENTER_CRITICAL_ISR wrapping the serial writes.
 *        This solved the "start" bug as it seems something was interrupting the serial writes
*/
void writeData(int dat[], int len)
{
    portENTER_CRITICAL_ISR(&timerMux);
    for (int i = 0; i < len; i++)
    {
        Serial2.write(dat[i]);
    }
    portEXIT_CRITICAL_ISR(&timerMux);
}

/**
 * @brief Set the used digital input pin modes to input_pullup
 */
void setPinsToPullup(int digital_pins[], int arrLen)
{
    for (int i = 0; i < arrLen; i++)
    {
        if (digital_pins[i] != -1)
        {
            pinMode(digital_pins[i], INPUT_PULLUP);
        }
    }
}

int mapStickVals(int calVals[], int dead[], int value)
{
    int mapped_val = -1;
    double lowS;
    double highS;
    int highSGood = 0; // for checking division by 0
    int lowSGood = 0;  // for checking division by 0
    double val = (double)value;

    int low = calVals[0];
    int neutch = calVals[1];
    int high = calVals[2];

    // make the slopes
    if ((high - neutch) != 0)
    {
        highS = 127.0 / ((double)high - (double)neutch);
        highSGood = 1;
    }
    if ((neutch - low) != 0)
    {
        lowS = 127.0 / ((double)neutch - (double)low);
        lowSGood = 1;
    }

    // map the value onto the line
    if (val <= neutch && lowSGood == 1)
    {
        mapped_val = (int)(lowS * val - lowS * neutch + 127.0);
    }
    if (val > neutch && highSGood == 1)
    {
        mapped_val = (int)(highS * val - highS * neutch + 127.0);
    }

    // check if mapped_val exceeds the bounds
    if (mapped_val < 0)
    {
        return 0;
    }
    if (mapped_val > 255)
    {
        return 255;
    }

    if (mapped_val >= dead[0] && mapped_val <= dead[1])
    {
        mapped_val = dead[2];
    }

    return mapped_val;
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

int getToggleFromChar(char char_read)
{
    int toggle = 1;

    if (char_read == 'Y')
    {
        toggle = 1;
    }
    else if (char_read == 'N')
    {
        toggle = 0;
    }

    return toggle;
}

void clearUARTrx()
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

void eraseOTA()
{
    const esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "otadata");
    assert(partition != NULL);

    static char store_data[] = "ESP-IDF Partition Operations Example (Read, Erase, Write)";
    static char read_data[sizeof(store_data)];

    // Erase entire partition
    memset(read_data, 0xFF, sizeof(read_data));
    ESP_ERROR_CHECK(esp_partition_erase_range(partition, 0, partition->size));

    // Write the data, starting from the beginning of the partition
    ESP_ERROR_CHECK(esp_partition_write(partition, 0, store_data, sizeof(store_data)));
    ESP_LOGI(TAG, "Written data: %s", store_data);

    // Read back the data, checking that read data and written data match
    ESP_ERROR_CHECK(esp_partition_read(partition, 0, read_data, sizeof(read_data)));
    assert(memcmp(store_data, read_data, sizeof(read_data)) == 0);
    ESP_LOGI(TAG, "Read data: %s", read_data);

    // Erase the area where the data was written. Erase size shoud be a multiple of SPI_FLASH_SEC_SIZE
    // and also be SPI_FLASH_SEC_SIZE aligned
    ESP_ERROR_CHECK(esp_partition_erase_range(partition, 0, SPI_FLASH_SEC_SIZE));

    // Read back the data (should all now be 0xFF's)
    memset(store_data, 0xFF, sizeof(read_data));
    ESP_ERROR_CHECK(esp_partition_read(partition, 0, read_data, sizeof(read_data)));
    assert(memcmp(store_data, read_data, sizeof(read_data)) == 0);
}

// Bluetooth stuff

int password_correct = 0;
bool bluetoothConnected = false;

void MyServerCallbacks::onConnect(BLEServer *pServer)
{
    bluetoothConnected = true;
}
void MyServerCallbacks::onDisconnect(BLEServer *pServer)
{
    bluetoothConnected = false;
    password_correct = 0; // reset for next connection
}


// Wifi Stuff

WebServer server(80);
char ipAddy[16];
const char *host = "esp32";
int wifi_flag = 0;

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

void wifiUploadEnabled(String Message)
{
    char *token;
    char *mystring = (char *)Message.c_str();
    const char *delimiter = "/";
    int i = 0;
    char *ssid;
    char *password;
    ipAddy[0] = 'N';
    ipAddy[1] = 'O';

    token = strtok(mystring, delimiter);
    while (token != NULL)
    {
        //      Serial.println(token);
        switch (i)
        {
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
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
        i++;
        if (i >= 100)
        {
            break;
        }
    }

    if (i < 100)
    {
        Serial.println("");
        Serial.print("Connected to ");
        Serial.println(ssid);
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());

        /*use mdns for host name resolution*/
        if (!MDNS.begin(host))
        { // http://esp32.local
            Serial.println("Error setting up MDNS responder!");
            while (1)
            {
                delay(1000);
            }
        }
        Serial.println("mDNS responder started");
        /*return index page which is stored in serverIndex */
        server.on("/", HTTP_GET, []()
                  {
      server.sendHeader("Connection", "close");
      server.send(200, "text/html", loginIndex); });
        server.on("/serverIndex", HTTP_GET, []()
                  {
      server.sendHeader("Connection", "close");
      server.send(200, "text/html", serverIndex); });
        /*handling uploading firmware file */
        server.on(
            "/update", HTTP_POST, []()
            {
      server.sendHeader("Connection", "close");
      server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
      ESP.restart(); },
            []()
            {
                HTTPUpload &upload = server.upload();
                if (upload.status == UPLOAD_FILE_START)
                {
                    Serial.printf("Update: %s\n", upload.filename.c_str());
                    if (!Update.begin(UPDATE_SIZE_UNKNOWN))
                    { // start with max available size
                        Update.printError(Serial);
                    }
                }
                else if (upload.status == UPLOAD_FILE_WRITE)
                {
                    /* flashing firmware to ESP*/
                    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
                    {
                        Update.printError(Serial);
                    }
                }
                else if (upload.status == UPLOAD_FILE_END)
                {
                    if (Update.end(true))
                    { // true to set the size to the current progress
                        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
                    }
                    else
                    {
                        Update.printError(Serial);
                    }
                }
            });
        server.begin();
        wifi_flag = 1;
        WiFi.localIP().toString().toCharArray(ipAddy, 15);
    }
}