#ifndef SIMPLE_ACK_TRACKER_H_
#define SIMPLE_ACK_TRACKER_H_

#include "AckTracker.h"
#include <queue>

/*
    * This class implements the AckTracker with a simple queue.
    * Note that this implementation is NOT persistent.
    * */
class SimpleAckTracker: public AckTracker
{
    public:
        void add(Packet packet) {dataBuff.push(packet);}
        void confirmNext() {dataBuff.pop();}
        std::uint32_t amount() {return dataBuff.size();}
        Packet next() {return dataBuff.front();}
    private:
        std::queue<Packet> dataBuff;
};

#endif