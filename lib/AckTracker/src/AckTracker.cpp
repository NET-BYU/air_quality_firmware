#include "AckTracker.h"

//#define UINT8_MAX 255
#define FILENAME_LENGTH_EXTENSION 30

#define FILEIO_LSB_FILTER 0x000000FF   //Can be used to zero-out every byte except for the LSB
#define FILEIO_NUM_BYTES_IN_U32 4      //Used to loop through each byte when converting 32-bit integer to array of chars
#define FILEIO_NUM_BYTES_IN_U16 2
#define FILEIO_NUM_BITS_IN_BYTE 8      //Used to calculate how much to shift when converting 32-bit integer to array of chars
#define FILEIO_LSB_IN_U32LEU8ARRAY 0   //Least Significant Byte in a Little-Endian Byte Array
#define FILEIO_MSB_IN_U32LEU8ARRAY (FILEIO_NUM_BYTES_IN_U32-1)  //Most Significant Byte in a 32-bit Little-Endian Byte Array
#define FILEIO_MSB_IN_U16LEU8ARRAY (FILEIO_NUM_BYTES_IN_U16-1)  //Most Significant Byte in a 16-bit Little-Endian Byte Array

/*
    * Converts a uint32_t variable into a Little-Endian array of bytes
    * Arg1 n is your 32-bit variable. Arg2 should be an array with at
    * least 4 bytes available to be written.
    * */
void convertU32ToLEU8Array(uint32_t n, uint8_t *b)
{
    for (uint8_t i = 0; i < FILEIO_NUM_BYTES_IN_U32; i++)  //Loop for each byte in the 32-bit integer
    {
        b[i] = (uint8_t) ((n >> (i * FILEIO_NUM_BITS_IN_BYTE)) & FILEIO_LSB_FILTER);   //Right shift to get rid of bytes we already handled, then zero-out everything except for the Least Significant Byte (LSB)
    }
}

/*
    * Converts a 4-byte array of chars into an unsigned 32-bit integer
    * */
uint32_t convertLEU8ArrayToU32(uint8_t *b)
{
    uint32_t n = 0;
    for (uint8_t i = 0; i < FILEIO_NUM_BYTES_IN_U32; i++)    //Loop from the MSB to the LSB
    {
        n = n << FILEIO_NUM_BITS_IN_BYTE;   //Shift by one byte to allow adding the next most significant byte for the next loop
        n += b[FILEIO_MSB_IN_U32LEU8ARRAY - i];  //Get what the next most significant byte is in the array, and add it to our number we are reconstructing
    }
    return n;   //Return the reconstructed number
}

void convertU16ToLEU8Array(uint16_t n, uint8_t *b)
{
    for (uint8_t i = 0; i < FILEIO_NUM_BYTES_IN_U16; i++)  //Loop for each byte in the 16-bit integer
    {
        b[i] = (uint8_t) ((n >> (i * FILEIO_NUM_BITS_IN_BYTE)) & FILEIO_LSB_FILTER);   //Right shift to get rid of bytes we already handled, then zero-out everything except for the Least Significant Byte (LSB)
    }
}

/*
    * Converts a 2-byte array of chars into an unsigned 16-bit integer
    * */
uint16_t convertLEU8ArrayToU16(uint8_t *b)
{
    uint16_t n = 0;
    for (uint8_t i = 0; i < FILEIO_NUM_BYTES_IN_U16; i++)    //Loop from the MSB to the LSB
    {
        n = n << FILEIO_NUM_BITS_IN_BYTE;   //Shift by one byte to allow adding the next most significant byte for the next loop
        n += b[FILEIO_MSB_IN_U16LEU8ARRAY - i];  //Get what the next most significant byte is in the array, and add it to our number we are reconstructing
    }
    return n;   //Return the reconstructed number
}

AckTracker::AckTracker(SdFat sd, const int chipSelect, const char name[], uint16_t entrySize) : log("AckTracker"), sd(sd), chipSelect(chipSelect), name(name), entrySize(entrySize)
{
    maxEntriesPerFile = ACK_TRACKER_MAX_FILE_SIZE / (entrySize + ACK_TRACKER_LENGTH_SIZE + ACK_TRACKER_ID_SIZE);
}

/*
    * Begins SD card, attempts to enumerate files, and reconstruct numFiles, tail, and head.
    * If no files belonging to the AckTracker exist, then one will be created.
    * 
    * Most calculation and time intense things are done here, then cached.
    * */
bool AckTracker::begin()
{
    log.info("Attempting to begin SD Card");
    bool sdStarted = sd.begin(chipSelect);
    if (sdStarted)
    {
        log.info("SD card started successfully");
        log.info("Attempting to enumerate files");
        for (uint8_t i = ACK_TRACKER_FIRST_FILE_INDEX; i < UINT8_MAX; i++)
        {
            char fName[name.length() + FILENAME_LENGTH_EXTENSION];
            sprintf(fName, "%s%d.acktrack", name.c_str(), i);
            log.info("Checking if the file %s exists", fName);
            if (!sd.exists(fName))          //If we found a file that doesn't exist yet
            {
                if (i == ACK_TRACKER_FIRST_FILE_INDEX)  //If the file that doesn't exist is the first file
                {
                    this->numFiles = 0;     //Indicate that there are no files yet
                    log.info("No files were found. Attempting to create blank file.");
                    bool createSuccess = createBlankFile(0); //Create the very first file, file 0
                    if (createSuccess)      //If we were able to successfully create the file
                    {
                        this->numFiles = 1; //Then we can say there is 1 file now
                        this->tail = ACK_TRACKER_TAIL_INIT;
                        this->head = ACK_TRACKER_HEAD_INIT;
                        this->unconfirmed = 0;
                        log.info("File successfully created.");
                        needsSDSetup = false;
                        return true;        //Return that the operation was successful
                    }
                    else    //Otherwise, if we weren't able to create a file
                    {
                        log.warn("Warning - Unable to create file");
                        return false;           //Return that we weren't able to create a file and thus begin() failed
                    }
                }
                else
                {
                    this->numFiles = i;
                    this->tail = readTail(i-1);
                    this->head = readHead(i-1);
                    if (this->tail == ACK_TRACKER_ERROR32 || this->head == ACK_TRACKER_ERROR32)
                    {
                        log.error("Could not begin. Unable to read tail or head.");
                        return false;
                    }
                    if (!isValidIndex(this->tail) || !isValidIndex(this->head))
                    {
                        log.error("File was corrupted. Attempting to change AckTracker name.");
                        this->rename();
                        return this->begin();
                    }
                    this->unconfirmed = this->tail - this->head;
                    log.info("The number of files found was %d", this->numFiles);
                }
                needsSDSetup = false;
                return true;
            }
        }
        this->numFiles=UINT8_MAX;
        this->tail = readTail(UINT8_MAX-1);
        this->head = readHead(UINT8_MAX-1);
        if (this->tail == ACK_TRACKER_ERROR32 || this->head == ACK_TRACKER_ERROR32)
        {
            log.error("Could not begin. Unable to read tail or head.");
            return false;
        }
        this->unconfirmed = this->tail - this->head;
        log.info("The number of files found was %d", this->numFiles);
        needsSDSetup = false;
        return true;
    }
    else
    {
        log.warn("Warning - Failed attempt to start SD card");
        return false;
    }
}

bool AckTracker::isValidIndex(uint32_t index)
{
    return true;//return ((index - ACK_TRACKER_FILE_HEADER_SIZE) % this->entrySize) == 0; //If it doesn't align, it's not a valid index
}

/*
    * Add the given array of data to the AckTracker. Data is deep-copied, so
    * scope is irrelevant. 
    * */
bool AckTracker::add(uint32_t id, uint16_t length, uint8_t *data)
{
    if (this->needsSDSetup)
    {
        if(!this->begin()) 
        {
            log.error("Could not start SD card.");
            return false;
        }
    }
    if (this->tail == ACK_TRACKER_MAX_ENTRIES)
    {
        log.error("Error - Could not add more data. AckTracker is full!");
        return false;
    }
    uint8_t tailFile = getNumFiles() - 1;
    uint32_t offset = getFileOffsetFromIndex(this->tail);
    log.info("Adding data to file %u at offset %lu (tail = %lu)", tailFile, offset, this->tail);
    char fName[name.length() + FILENAME_LENGTH_EXTENSION];
    sprintf(fName, "%s%d.acktrack", name.c_str(), tailFile);
    log.info("Attempting to add data with id(%lu) and length=%u to file %s", id, length, fName);
    if (sd.exists(fName))
    {
        log.info("File %s exists. Now attempting to add data.", fName);
        File f = sd.open(fName, O_RDWR | O_BINARY);
        if (f.available())
        {
            uint32_t fSize = f.fileSize();
            if (offset > fSize)
            {
                log.error("Error - writing offset %lu is greater than fileSize %lu", offset, fSize);
                f.close();
                this->needsSDSetup = true;
                return false;
            }
            else
            {
                log.info("Writing offset = %lu, fileSize = %lu", offset, fSize);
            }
            
            log.info("File %s available for read/writing.", fName);
            f.seek(offset);
            uint8_t id_B[ACK_TRACKER_ID_SIZE];
            convertU32ToLEU8Array(id, id_B);
            uint8_t length_B[ACK_TRACKER_LENGTH_SIZE];
            convertU16ToLEU8Array(length, length_B);
            if (f.write(id_B, ACK_TRACKER_ID_SIZE) < 0 ||
                f.write(length_B, ACK_TRACKER_LENGTH_SIZE) < 0 ||
                f.write(data, this->entrySize) < 0)
            {
                this->needsSDSetup = true;
                log.error("Error when writing to %s", fName);
                return false;
            }
            log.info("Successfully wrote data to file %s", fName);
            uint32_t tail = this->tail+1;
            uint8_t nextFileNum = getFileNumFromIndex(tail);
            if (nextFileNum > tailFile)
            {
                f.close();
                log.info("Max entries reached. Attempting to create blank file.");
                bool createSuccess = createBlankFile(nextFileNum);
                if (createSuccess)
                {
                    char fName[name.length() + FILENAME_LENGTH_EXTENSION];
                    sprintf(fName, "%s%d.acktrack", name.c_str(), nextFileNum);
                    log.info("Successfully created new file %s", fName);
                    (this->tail)++;
                    (this->numFiles)++;
                    (this->unconfirmed)++;
                    log.info("Attempting to write incremented tail %lu", this->tail);
                    f.seek(ACK_TRACKER_TAIL_LOCATION);
                    uint8_t tail_B[FILEIO_NUM_BYTES_IN_U32];
                    convertU32ToLEU8Array(this->tail, tail_B);
                    f.write(tail_B, ACK_TRACKER_TAIL_SIZE);
                    f.close();
                    return true;
                }
                else
                {
                    log.error("Error - Max entries reached, but couldn't create new file.");
                    this->needsSDSetup = true;
                    return false;
                }
            }
            else
            {
                log.info("Attempting to write incremented tail value");
                f.seek(ACK_TRACKER_TAIL_LOCATION);
                uint8_t tail_B[FILEIO_NUM_BYTES_IN_U32];
                (this->tail)++;
                (this->unconfirmed)++;
                convertU32ToLEU8Array(this->tail, tail_B);
                f.write(tail_B, ACK_TRACKER_TAIL_SIZE);
                f.close();
                return true;
            }
        }
        else
        {
            f.close();
            log.error("Error - could not write to file %s. File not available.", fName);
            this->needsSDSetup = true;
            return false;
        }
    }
    else
    {
        log.error("Error - could not add data to file %s. File does not exist.", fName);
        this->needsSDSetup = true;
        return false;
    }
    
}

/*
    * Data is retrieved by index. The oldest data in the AckTracker
    * has an index of 0, and the newest data has an index of
    * unconfirmedCount()-1.
    * Returns the id, length, and fills the provided data array with
    * a copy of the data. User is responsible for providing a big enough
    * array.
    *  */
bool AckTracker::getAbsolute(uint32_t index, uint32_t &id, uint16_t &length, uint8_t *data)
{
    if (index > this->tail)
    {
        id = ACK_TRACKER_ERROR32;
        length = ACK_TRACKER_ERROR16;
        log.error("Warning - cannot get data past tail");
        return false;
    }
    uint8_t fileNum = getFileNumFromIndex(index);
    char fName[name.length() + FILENAME_LENGTH_EXTENSION];
    sprintf(fName, "%s%d.acktrack", name.c_str(), fileNum);
    if (sd.exists(fName))
    {
        uint32_t offset = getFileOffsetFromIndex(index);
        File f = sd.open(fName, O_READ | O_BINARY);
        if (f.available())
        {
            f.seek(offset);
            uint8_t id_B[ACK_TRACKER_ID_SIZE];
            uint8_t length_B[ACK_TRACKER_LENGTH_SIZE];
            f.read(id_B, ACK_TRACKER_ID_SIZE);
            f.read(length_B, ACK_TRACKER_LENGTH_SIZE);
            id = convertLEU8ArrayToU32(id_B);
            length = convertLEU8ArrayToU16(length_B);
            f.read(data, length);
            f.close();
            return true;
        }
        else
        {
            log.error("Could not get data. File exists, but not available.");
            return false;
        }
        
    }
    else
    {
        log.error("Could not get data. File does not exist.");
        return false;
    }
}

bool AckTracker::get(uint32_t index, uint32_t &id, uint16_t &length, uint8_t *data)
{
    return getAbsolute(index + getHead(), id, length, data);
}

// /*
//     * Returns the length only of the data at the given index.
//     * The oldest data in the AckTracker has an index of 0, and
//     * the newest data has an index of unconfirmedCount()-1.
//     * If the provided index reaches beyond the size of the AckTracker,
//     * behavior is undefined.
//     *  */
uint16_t AckTracker::getLength(uint32_t index)
{
    index += getHead();

    if (index > this->tail)
    {
        log.error("getLength() failed. Index too large.");
        return ACK_TRACKER_ERROR16;
    }
    uint16_t length = ACK_TRACKER_ERROR16;
    uint8_t fileNum = getFileNumFromIndex(index);
    char fName[name.length() + FILENAME_LENGTH_EXTENSION];
    sprintf(fName, "%s%d.acktrack", name.c_str(), fileNum);
    if (sd.exists(fName))
    {
        uint32_t offset = getFileOffsetFromIndex(index);
        File f = sd.open(fName, O_READ | O_BINARY);
        if (f.available())
        {
            f.seek(offset + ACK_TRACKER_LENGTH_LOCATION);
            uint8_t length_B[ACK_TRACKER_LENGTH_SIZE];
            f.read(length_B, ACK_TRACKER_LENGTH_SIZE);
            length = convertLEU8ArrayToU16(length_B);
            f.close();
            log.info("Successfully retrieved length");
        }
        else
        {
            log.warn("Warning - could not retrieve length. File %s not available.", fName);
        }
    }
    else
    {
        log.warn("Warning - could not retrieve length. File %s doesn't exist", fName);
    }
    return length;
}

// /*
//     * Returns the ID only of the data at the given index.
//     * The oldest data in the AckTracker has an index of 0, and
//     * the newest data has an index of unconfirmedCount()-1.
//     *  */
uint32_t AckTracker::getID(uint32_t index)
{
    index += getHead();

    if (index > this->tail)
    {
        return ACK_TRACKER_ERROR32;
    }
    uint32_t id = ACK_TRACKER_ERROR32;
    uint8_t fileNum = getFileNumFromIndex(index);
    char fName[name.length() + FILENAME_LENGTH_EXTENSION];
    sprintf(fName, "%s%d.acktrack", name.c_str(), fileNum);
    if (sd.exists(fName))
    {
        uint32_t offset = getFileOffsetFromIndex(index);
        File f = sd.open(fName, O_READ | O_BINARY);
        if (f.available())
        {
            f.seek(offset);
            uint8_t id_B[ACK_TRACKER_ID_SIZE];
            f.read(id_B, ACK_TRACKER_ID_SIZE);
            id = convertLEU8ArrayToU32(id_B);
            f.close();
            log.info("Successfully retrieved ID %lu", id);
        }
        else
        {
            log.error("Warning - could not retrieve ID, file %s not available for reading", fName);
        }
        
    }
    else
    {
        log.error("Warning - could not retrieve ID, file %s doesn't exist.", fName);
    }
    return id;
}

/*
    * Returns the number of data packets that haven't been confirmed yet.
    *  */
uint32_t AckTracker::unconfirmedCount()
{
    return unconfirmed;
}

/*
    * Confirms the next n packets of data (starting with the oldest data at index=0),
    * where confirmCount = n.
    *  */
bool AckTracker::confirmNext(uint32_t confirmCount)
{
    log.info("Attempting to confirm %lu items", confirmCount);
    uint32_t head = this->head+confirmCount;
    if (head > tail)
    {
        log.error("Warning - Can't confirm more items than in buffer.");
        confirmCount -= (head - tail);
        head = this->head+confirmCount;
        if (confirmCount == 0)
        {
            return false;
        }
    }
    else if (head < this->head) //If there was an overflow
    {
        log.error("Error - Overflow, could not confirm %lu items.", confirmCount);
        return false;
    }
    writeHead(this->numFiles-1, this->head);
    (this->head)=head;
    this->unconfirmed -= confirmCount;
    return true;
}

/*
    * Confirms the next packet of data (the oldest at index=0)
    *  */
bool AckTracker::confirmNext()
{
    return confirmNext(1);
}

uint32_t AckTracker::getHead()
{
    return this->head;
}

uint32_t AckTracker::getTail()
{
    return this->tail;
}

bool AckTracker::createBlankFile(uint8_t fileNum)
{
    char fName[name.length() + FILENAME_LENGTH_EXTENSION];
    sprintf(fName, "%s%d.acktrack", name.c_str(), fileNum);
    if (!sd.exists(fName))
    {
        uint8_t header[ACK_TRACKER_FILE_HEADER_SIZE];
        convertU16ToLEU8Array(this->entrySize, header + ACK_TRACKER_ENTRY_SIZE_LOCATION);
        convertU32ToLEU8Array(ACK_TRACKER_TAIL_INIT, header + ACK_TRACKER_TAIL_LOCATION);
        convertU32ToLEU8Array(ACK_TRACKER_HEAD_INIT, header + ACK_TRACKER_HEAD_LOCATION);
        log.info("Attempting to create file %s", fName);
        File f = sd.open(fName, O_RDWR | O_CREAT | O_BINARY);
        f.write(header, ACK_TRACKER_FILE_HEADER_SIZE);
        f.close();
        log.info("Successfully created file %s", fName);
        return true;
    }
    log.warn("Warning - Unable to create blank file");
    return false;
}

uint32_t AckTracker::readTail(uint8_t fileNum)
{
    uint32_t tail = ACK_TRACKER_ERROR32;
    char fName[name.length() + FILENAME_LENGTH_EXTENSION];
    sprintf(fName, "%s%d.acktrack", name.c_str(), fileNum);
    log.info("Attempting to read tail from file %s", fName);
    if (sd.exists(fName))
    {
        log.info("File %s exists, now attempting to read", fName);
        uint8_t tail_B[FILEIO_NUM_BYTES_IN_U32];
        File f = sd.open(fName, O_READ | O_BINARY);
        if (f.available())
        {
            log.info("File %s is available for reading", fName);
            f.seek(ACK_TRACKER_TAIL_LOCATION);
            f.read(tail_B, ACK_TRACKER_TAIL_SIZE);
            tail = convertLEU8ArrayToU32(tail_B);
            log.info("Retreived tail value of %lu", tail);
        }
        else
        {
            log.warn("Warning - Could not retrieve tail, file %s not available for reading", fName);
        }
        f.close();
    }
    else
    {
        log.warn("Could not read tail from file %s, file does not exist", fName);
    }
    return tail;
}

uint32_t AckTracker::readHead(uint8_t fileNum)
{
    uint32_t head = ACK_TRACKER_ERROR32;
    char fName[name.length() + FILENAME_LENGTH_EXTENSION];
    sprintf(fName, "%s%d.acktrack", name.c_str(), fileNum);
    log.info("Attempting to read head from file %s", fName);
    if (sd.exists(fName))
    {
        log.info("File %s exists, now attempting to read", fName);
        uint8_t head_B[FILEIO_NUM_BYTES_IN_U32];
        File f = sd.open(fName, O_READ | O_BINARY);
        if (f.available())
        {
            log.info("File %s is available for reading", fName);
            f.seek(ACK_TRACKER_HEAD_LOCATION);
            f.read(head_B, ACK_TRACKER_HEAD_SIZE);
            head = convertLEU8ArrayToU32(head_B);
            log.info("Retreived head value of %lu", head);
        }
        else
        {
            log.error("Warning - Could not retrieve head, file %s not available for reading", fName);
        }
        f.close();
    }
    else
    {
        log.error("Could not read head from file %s, file does not exist", fName);
    }
    return head;
}

bool AckTracker::writeHead(uint8_t fileNum, uint32_t head)
{
    char fName[name.length() + FILENAME_LENGTH_EXTENSION];
    sprintf(fName, "%s%d.acktrack", name.c_str(), fileNum);
    log.info("Attempting to write head of file %s", fName);
    if (sd.exists(fName))
    {
        log.info("File %s exists, now attempting to write head.", fName);
        File f = sd.open(fName, O_RDWR | O_BINARY);
        if (f.available())
        {
            log.info("File %s available for writing. Writing head.", fName);
            f.seek(ACK_TRACKER_HEAD_LOCATION);
            uint8_t head_B[ACK_TRACKER_HEAD_SIZE];
            convertU32ToLEU8Array(head, head_B);
            f.write(head_B, ACK_TRACKER_HEAD_SIZE);
            f.close();
            return true;
        }
        else
        {
            log.warn("Warning - file %s exists, but not available for writing. Could not write head.", fName);
            f.close();
            return false;
        }
    }
    else
    {
        log.warn("Warning - file %s does not exist. Could not write head", fName);
        return false;
    }
}

uint8_t AckTracker::getFileNumFromIndex(uint32_t index)
{
    return index / getMaxEntriesPerFile();
}

uint32_t AckTracker::getFileOffsetFromIndex(uint32_t index)
{
    return ((index % getMaxEntriesPerFile()) * (this->entrySize + ACK_TRACKER_LENGTH_SIZE + ACK_TRACKER_ID_SIZE)) + ACK_TRACKER_FILE_HEADER_SIZE;
}

uint32_t AckTracker::getIndex(uint8_t fileNum, uint32_t offset)
{
    return (fileNum * getMaxEntriesPerFile()) + ((offset - ACK_TRACKER_FILE_HEADER_SIZE) / (this->entrySize + ACK_TRACKER_LENGTH_SIZE + ACK_TRACKER_ID_SIZE));
}

uint8_t AckTracker::getNumFiles()
{
    return this->numFiles;
}

uint32_t AckTracker::getMaxEntriesPerFile()
{
    return this->maxEntriesPerFile;
}

bool AckTracker::rename()
{
    bool success = false;
    for (uint32_t i = 0; i < UINT32_MAX; i++)
    {
        char suf[10];
        sprintf(suf, "_%lu_", i);
        std::string suf_s(suf);
        std::size_t pos = this->name.find(suf);
        if (pos != std::string::npos)
        {
            if (i == 0)
            {
                this->name = this->name + suf_s;
            }
            else
            {
                this->name.replace(pos, pos+suf_s.length(), suf_s);
            }
            
        }
    }
    return success;
}