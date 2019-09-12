#ifndef ACK_TRACKER_H_
#define ACK_TRACKER_H_

#include <cstdint>
#include "../../SdFat/src/SdFat.h"
#include "Particle.h"

//Inline header info
#define ACK_TRACKER_ID_SIZE 4
#define ACK_TRACKER_LENGTH_SIZE 2
#define ACK_TRACKER_ID_LOCATION 0
#define ACK_TRACKER_LENGTH_LOCATION (ACK_TRACKER_ID_LOCATION + ACK_TRACKER_ID_SIZE)

//File header info
#define ACK_TRACKER_FIRST_FILE_INDEX 0
#define ACK_TRACKER_TAIL_INIT 0
#define ACK_TRACKER_HEAD_INIT 0
#define ACK_TRACKER_ENTRY_SIZE_SIZE 2       //The size of the entry size variable in bytes
#define ACK_TRACKER_TAIL_SIZE 4             //The size of the tail in bytes
#define ACK_TRACKER_HEAD_SIZE 4             //The size of the head in bytes
#define ACK_TRACKER_ENTRY_SIZE_LOCATION 0   //The file position of the entry size
#define ACK_TRACKER_TAIL_LOCATION ACK_TRACKER_ENTRY_SIZE_SIZE   //The file position of the tail is after the entry size position
#define ACK_TRACKER_HEAD_LOCATION (ACK_TRACKER_TAIL_LOCATION + ACK_TRACKER_TAIL_SIZE)    //The file position of the head is after the tail position
#define ACK_TRACKER_FILE_HEADER_SIZE (ACK_TRACKER_ENTRY_SIZE_SIZE + ACK_TRACKER_TAIL_SIZE + ACK_TRACKER_HEAD_SIZE)
#define ACK_TRACKER_DATA_START_LOCATION ACK_TRACKER_FILE_HEADER_SIZE
#define ACK_TRACKER_MAX_ENTRIES 0xFFFFFFFE //The number of entries you can append before the Group File becomes full
#define ACK_TRACKER_MAX_FILE_SIZE 2000000000    //Files will be less than or equal to this size in bytes

#define ACK_TRACKER_ERROR32 0xFFFFFFFF
#define ACK_TRACKER_ERROR16 0xFFFF
/*
    * When data is added to the tracker, it is considered "unacknowledged".
    * Data, whether it is acknowledged or not, persists on an SD card.
    * Unconfirmed data then can be acknowledged/confirmed in FIFO order.
    * Data persists across a group of files with a max size of 2GB each
    * */
class AckTracker
{
    public:
        AckTracker(SdFat sd, int chipSelect, const char name[], uint16_t entrySize);
        bool begin();
        /*
            * Add the given array of data to the AckTracker. Data is deep-copied, so
            * scope is irrelevant. 
            * */
        bool add(uint32_t id, uint16_t length, uint8_t *data);
        bool getAbsolute(uint32_t index, uint32_t &id, uint16_t &length, uint8_t *data);
        /*
            * Data is retrieved by index. The oldest data in the AckTracker
            * has an index of 0, and the newest data has an index of
            * unconfirmedCount()-1.
            * Returns the id, length, and fills the provided data array with
            * a copy of the data. User is responsible for providing a big enough
            * array.
            *  */
        bool get(uint32_t index, uint32_t &id, uint16_t &length, uint8_t *data);
        // /*
        //     * Returns the length only of the data at the given index.
        //     * The oldest data in the AckTracker has an index of 0, and
        //     * the newest data has an index of unconfirmedCount()-1.
        //     * If the provided index reaches beyond the size of the AckTracker,
        //     * behavior is undefined.
        //     *  */
        uint16_t getLength(uint32_t index);
        // /*
        //     * Returns the ID only of the data at the given index.
        //     * The oldest data in the AckTracker has an index of 0, and
        //     * the newest data has an index of unconfirmedCount()-1.
        //     *  */
        uint32_t getID(uint32_t index);
        /*
            * Returns the number of data packets that haven't been confirmed yet.
            *  */
        uint32_t unconfirmedCount();
        /*
            * Confirms the next n packets of data (starting with the oldest data at index=0),
            * where confirmCount = n.
            *  */
        bool confirmNext(uint32_t confirmCount);
        /*
            * Confirms the next packet of data (the oldest at index=0)
            *  */
        bool confirmNext();
        uint32_t getHead();
        uint32_t getTail();
    protected:
        bool isValidIndex(uint32_t index);
        bool createBlankFile(uint8_t fileNum);
        uint32_t readTail(uint8_t fileNum);
        uint32_t readHead(uint8_t fileNum);
        bool writeHead(uint8_t fileNum, uint32_t head);
        uint8_t getFileNumFromIndex(uint32_t index);
        uint32_t getFileOffsetFromIndex(uint32_t index);
        uint32_t getIndex(uint8_t fileNum, uint32_t offset);
        uint8_t getNumFiles();
        uint32_t getMaxEntriesPerFile();
        bool rename();
    private:
        bool needsSDSetup = true;
        Logger log;
        SdFat sd;
        const int chipSelect;
        std::string name;
        uint16_t entrySize;
        uint32_t head = ACK_TRACKER_HEAD_INIT;
        uint32_t tail = ACK_TRACKER_TAIL_INIT;
        uint32_t unconfirmed = 0;
        uint8_t numFiles = 0;
        uint32_t maxEntriesPerFile = 0;
};

#endif