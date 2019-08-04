#ifndef ACK_TRACKER_H_
#define ACK_TRACKER_H_

#include <cstdint>

/*
    * An abstract class that acts as the interface for Acknowledgement Trackers.
    * When data is added to the tracker, it is considered "unconfirmed".
    * Unconfirmed data then can be acknowledged/confirmed in FIFO order.
    * */
class AckTracker
{
    public:
        /*
            * Add the given array of data to the AckTracker. Data is deep-copied, so
            * scope is irrelevant. 
            * */
        virtual void add(std::uint32_t id, std::uint8_t length, std::uint8_t *data) = 0;
        /*
            * Data is retrieved by index. The oldest data in the AckTracker
            * has an index of 0, and the newest data has an index of
            * unconfirmedCount()-1.
            * Returns the id, length, and fills the provided data array with
            * a copy of the data. User is responsible for providing a big enough
            * array.
            *  */
        virtual void get(std::uint32_t index, std::uint32_t &id, std::uint8_t &length, std::uint8_t *data) = 0;
        /*
            * Returns the length only of the data at the given index.
            * The oldest data in the AckTracker has an index of 0, and
            * the newest data has an index of unconfirmedCount()-1.
            * If the provided index reaches beyond the size of the AckTracker,
            * behavior is undefined.
            *  */
        virtual std::uint8_t getLengthOf(std::uint32_t index) = 0;
        /*
            * Returns the ID only of the data at the given index.
            * The oldest data in the AckTracker has an index of 0, and
            * the newest data has an index of unconfirmedCount()-1.
            *  */
        virtual std::uint32_t getIDOf(std::uint32_t index) = 0;
        /*
            * Returns the number of data packets that haven't been confirmed yet.
            *  */
        virtual std::uint32_t unconfirmedCount() = 0;
        /*
            * Confirms the next n packets of data (starting with the oldest data at index=0),
            * where confirmCount = n.
            *  */
        virtual void confirmNext(std::uint32_t confirmCount) = 0;
        /*
            * Confirms the next packet of data (the oldest at index=0)
            *  */
        virtual void confirmNext() = 0;
};

#endif