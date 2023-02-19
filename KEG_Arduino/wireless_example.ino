#include <string.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>

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

	// End Wifi Setup
}

