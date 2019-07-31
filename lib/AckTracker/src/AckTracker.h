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
        typedef struct
        {
            std::uint32_t id;
            std::uint32_t size;
            std::uint8_t *data;
        } Packet;
        virtual void add(Packet packet) = 0;
        virtual void confirmNext() = 0;
        virtual std::uint32_t amount() = 0;
        virtual Packet next() = 0;
        Packet pack(std::uint32_t id, std::uint32_t size, std::uint8_t *data)
        {
            Packet p = {id, size, data};
            return p;
        }
};

#endif