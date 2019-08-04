#ifndef SIMPLE_ACK_TRACKER_H_
#define SIMPLE_ACK_TRACKER_H_

#include "AckTracker.h"
#include <list>

/*
    * This class implements the AckTracker with a simple list.
    * Note that this implementation is NOT persistent, and because
    * it is wholly contained in memory, capacity is greatly limited.
    * */
class SimpleAckTracker: public AckTracker
{
    public:
        /*
            * Add the given array of data to the AckTracker. Data is deep-copied, so
            * scope is irrelevant. 
            * */
        void add(std::uint32_t id, std::uint8_t length, std::uint8_t *data);
        /*
            * Data is retrieved by index. The oldest data in the AckTracker
            * has an index of 0, and the newest data has an index of
            * unconfirmedCount()-1.
            * Returns the id, length, and a copy of data from the given index.
            *  */
        void get(std::uint32_t index, std::uint32_t &id, std::uint8_t &length, std::uint8_t *data);
        /*
            * Returns the length only of the data at the given index.
            * The oldest data in the AckTracker has an index of 0, and
            * the newest data has an index of unconfirmedCount()-1.
            * If the provided index reaches beyond the size of the AckTracker,
            * behavior is undefined.
            *  */
        std::uint8_t getLengthOf(std::uint32_t index);
        /*
            * Returns the ID only of the data at the given index.
            * The oldest data in the AckTracker has an index of 0, and
            * the newest data has an index of unconfirmedCount()-1.
            *  */
        std::uint32_t getIDOf(std::uint32_t index);
        /*
            * Returns the number of data packets that haven't been confirmed yet.
            *  */
        std::uint32_t unconfirmedCount();
        /*
            * Confirms the next n packets of data (starting with the oldest data at index=0),
            * where confirmCount = n.
            *  */
        void confirmNext(std::uint32_t confirmCount);
        /*
            * Confirms the next packet of data (the oldest at index=0)
            *  */
        void confirmNext();
        
    private:
        /*
            * A support class that contains added data
            *  */
        class Packet
        {
            public:
                std::uint32_t id;   //The id of the data packet
                std::uint8_t length;//How many bytes of data are being saved
                std::uint8_t *data; //An array of bytes which stores a copy of provided data
                Packet(std::uint32_t id, std::uint8_t length, std::uint8_t *data)
                {
                    this->id = id;          //Store a copy of id, length and data upon initialization
                    this->length = length;
                    std::uint8_t *m_data = new std::uint8_t[length];    //We allocate a copy of the array on the heap, the confirm() functions will delete the data
                    this->data = m_data;
                    for (std::uint32_t i = 0; i < length; i++)
                    {
                        this->data[i] = data[i];
                    }
                }
        };
        std::list<Packet *> packets;
};

#endif
