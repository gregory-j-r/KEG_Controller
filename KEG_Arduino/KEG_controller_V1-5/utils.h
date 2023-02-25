

#include <Arduino.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <stdlib.h>

// from boost (functional/hash): http://www.boost.org/doc/libs/1_35_0/doc/html/hash/combine.html template
// for hashing pairs in unordered map for fast lookup
template <class T>
inline void hash_combine(size_t &seed, T const &v)
{
    seed ^= std::hash<T>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

struct pair_hash
{
    template <class T1, class T2>
    size_t operator()(const std::pair<T1, T2> &p) const
    {
        size_t seed = 0;
        hash_combine(seed, p.first);
        hash_combine(seed, p.second);
        return seed;
    }
};


// get a strange insertion of -1 into some of the read data. this function looks through the buffer
//  and removes any single -1 that is present. It then shifts the rest of the values into place.
//  Think the -1 is when it reads but there is no data available, which seems like it should
//  never be possible but we'll keep that on the backburner
template <typename T, unsigned int SIZE> void cleanUpBufferRead(T (&buffer)[SIZE], int &buffer_index)
{
    int j = -1;
    for (int i = 0; i < buffer_index; i++)
    {
        if (buffer[i] == -1)
        {
            j = i;
            break;
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

void setPinsToPullup(int digital_pins[], int arrLen);

void writeData(int dat[], int len);

int mapStickVals(int calVals[], int dead[], int value);

char getCharFromToggle(int toggle);

int getToggleFromChar(char char_read);

void clearUARTrx();

void eraseOTA();

void wifiUploadEnabled(String Message);

// Setup callbacks onConnect and onDisconnect
class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer);
    void onDisconnect(BLEServer *pServer);
};

