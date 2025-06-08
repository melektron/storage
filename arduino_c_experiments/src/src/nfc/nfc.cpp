/*
ELEKTRON © 2025 - now
Written by melektron
www.elektron.work
17.04.25, 15:20

NFC Tag Reader Tests based on
https://randomnerdtutorials.com/esp32-mfrc522-rfid-reader-arduino/
https://github.com/OSSLibraries/Arduino_MFRC522v2?utm_source=platformio&utm_medium=piohome
*/

#include <Arduino.h>

#include <NfcAdapter.h>
#include <PN532.h>
#include <PN532_HSU.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "logging.hpp"

#include "nfc.hpp"


namespace nfc
{
    //static PN532_HSU interface(Serial1, 36, 35);    // RX=36, TX=35
    //static PN532_HSU interface(Serial1, 47, 48);    // RX=47, TX=48
    static PN532_HSU interface(Serial1, 11, 12);    // RX=11, TX=12
    static PN532 reader = PN532(interface);
    
    static uint8_t password[4] = { 0x12, 0x34, 0x56, 0x78 };
    static uint8_t buf[4];
    static uint8_t uid[7];
    static uint8_t uidLength;

    static constexpr size_t TASK_STACK_SIZE = 10000;
    static constexpr uint8_t TASK_CORE = 0;
    static constexpr uint8_t TASK_PRIORITY = 1;
    static StackType_t task_stack[TASK_STACK_SIZE];
    static StaticTask_t task_tcb;
    static TaskHandle_t task_handle;

    void task_fn(void *param);

} // namespace nfc


void nfc::initialize()
{
    reader.begin();

    uint32_t versiondata = reader.getFirmwareVersion();
    if (!versiondata)
    {
        logging::info("Didn't find PN53x board");
        while (1); // halt
    }
    // Got ok data, print it out!
    logging::info("Found chip PN5%hhx", (versiondata >> 24) & 0xFF);
    logging::info("Firmware ver. %d.%d", (versiondata >> 16) & 0xFF, (versiondata >> 8) & 0xFF);

    // configure board to read RFID tags
    reader.SAMConfig();

    logging::info("Looking for chip...");

    task_handle = xTaskCreateStatic(
        task_fn,
        "nfc",
        TASK_STACK_SIZE,
        nullptr,
        TASK_PRIORITY,
        task_stack,
        &task_tcb
    );
    if (task_handle == NULL)
    {
        logging::error("Could not create NFC task, aborting");
        abort();
    }
}

void nfc::task_fn(void *param)
{
    (void)param;

    for (;;)
    {
        uint8_t success;
        uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
        uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

        // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
        // 'uid' will be populated with the UID, and uidLength will indicate
        // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
        success = reader.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

        if (success)
        {
            // Display some basic information about the card
            logging::verb("Found an ISO14443A card");
            logging::verb("  UID Length: %d bytes", uidLength);
            logging::verb("  UID Value: ");
            logging::binary(uid, uidLength);

            if (uidLength == 4)
            {
            // We probably have a Mifare Classic card ... 
                logging::verb("Seems to be a Mifare Classic card (4 byte UID)");

                // Now we need to try to authenticate it for read/write access
                // Try with the factory default KeyA: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF
                logging::verb("Trying to authenticate block 4 with default KEYA value");
                uint8_t keya[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

                // Start with block 4 (the first block of sector 1) since sector 0
                // contains the manufacturer data and it's probably better just
                // to leave it alone unless you know what you're doing
                success = reader.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, keya);

                if (success)
                {
                    logging::verb("Sector 1 (Blocks 4..7) has been authenticated");
                    uint8_t data[16];

                    // If you want to write something to block 4 to test with, uncomment
                    // the following line and this text should be read back in a minute
                    // data = { 'a', 'd', 'a', 'f', 'r', 'u', 'i', 't', '.', 'c', 'o', 'm', 0, 0, 0, 0};
                    // success = reader.mifareclassic_WriteDataBlock (4, data);

                    // Try to read the contents of block 4
                    success = reader.mifareclassic_ReadDataBlock(4, data);

                    if (success)
                    {
                        // Data seems to have been read ... spit it out
                        logging::verb("Reading Block 4:");
                        logging::binary(data, 16);
                    }
                    else
                    {
                        logging::verb("Ooops ... unable to read the requested block.  Try another key?");
                    }
                }
                else
                {
                    logging::verb("Ooops ... authentication failed: Try another key?");
                }
            }

            if (uidLength == 7)
            {
            // We probably have a Mifare Ultralight card ...
                logging::verb("Seems to be a Mifare Ultralight tag (7 byte UID)");

                // Try to read the first general-purpose user page (#4)
                logging::verb("Reading page 4");
                uint8_t data[32];
                success = reader.mifareultralight_ReadPage(4, data);
                if (success)
                {
                // Data seems to have been read ... spit it out
                    logging::binary(data, 4);

                    // Wait a bit before reading the card again
                    delay(100);
                }
                else
                {
                    logging::verb("Ooops ... unable to read the requested page!?");
                }
            }
        }

        // Wait a bit before reading the card again
        delay(100);
    }
}
