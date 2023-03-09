#pragma once

#include <Arduino.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>

// #include <BLEDevice.h>
// #include <BLEServer.h>
// #include <BLEUtils.h>
// #include <BLE2902.h>

#include <stdlib.h>


/**
 * @brief Helper functions from Boost (http://www.boost.org/doc/libs/1_35_0/doc/html/hash/combine.html)
 * for hashing pairs in unordered map
*/
template <class T>
inline void hash_combine(size_t &seed, T const &v){
    seed ^= std::hash<T>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

struct pair_hash{
    template <class T1, class T2>
    size_t operator()(const std::pair<T1, T2> &p) const
    {
        size_t seed = 0;
        hash_combine(seed, p.first);
        hash_combine(seed, p.second);
        return seed;
    }
};


/**
 * @brief Remove values of -1 from buffer
 * 
 * @note Get a strange insertion of -1 into some of the read data. Function removes any single -1
 *       and shifts rest of the values into place. Think the -1 is when it reads but there is no
 *       data available, which seems like it should never be possible, keeping that on the backburner
*/
template <typename T, unsigned int SIZE> void cleanUpBufferRead(T (&buffer)[SIZE], int &buffer_index)
{
    int j = -1;
    for (int i = 0; i < buffer_index; i++){
        if (buffer[i] == -1){
            j = i;
            break;
        }
    }
    if (j != -1){
        for (int i = j; i < buffer_index - 1; i++){
            buffer[i] = buffer[i + 1];
        }
        buffer_index -= 1;
    }
}


extern portMUX_TYPE timerMux;

/**
 * @brief Writes data over 6 bit uart line serial 2
 *        Absolutely crucial that this function have the portENTER_CRITICAL_ISR wrapping the serial writes.
 *        This solved the "start" bug as it seems something was interrupting the serial writes
*/
void writeData(int dat[], int len){
    portENTER_CRITICAL_ISR(&timerMux);
    for (int i = 0; i < len; i++){
        Serial2.write(dat[i]);
    }
    portEXIT_CRITICAL_ISR(&timerMux);
}

/**
 * @brief Set the used digital input pin modes to input_pullup
 */
void setPinsToPullup(int digital_pins[], int arrLen){
    for (int i = 0; i < arrLen; i++){
        if (digital_pins[i] != -1){
            pinMode(digital_pins[i], INPUT_PULLUP);
        }
    }
}

/**
 * @brief Converts binary toggle to Y/N characters
*/
char getCharFromToggle(int toggle){
    char char_out;

    if (toggle == 1){
        char_out = 'Y';
    }
    else if (toggle == 0){
        char_out = 'N';
    }

    return char_out;
}

/**
 * @brief Converts Y/N characters to binary toggle
*/
int getToggleFromChar(char char_read){
    int toggle = 1;

    if (char_read == 'Y'){
        toggle = 1;
    }
    else if (char_read == 'N'){
        toggle = 0;
    }

    return toggle;
}

/**
 * @brief Clear/flush the UART serial port
*/
void clearUARTrx(){
    int dump;
    while (true){
        if (Serial2.available()){
            dump = Serial2.read();
            // 63 is STOP 
            if (dump == 63){
                break;
            }
        }
    }
}

/**
 * @brief Erase the partition holding the OTA update to boot back into factory in case of messup.
*/
void eraseOTA(){
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
