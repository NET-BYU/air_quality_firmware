#ifndef SQL_ACK_TRACKER_H
#define SQL_ACK_TRACKER_H

#include "PersistentAckTracker.h"

/*
    * A Persistent implementation of the Ack Tracker which
    * uses SQLite as a means to persist added data.
    * */
class SQLAckTracker: public PersistentAckTracker
{
    public:
        void add(Packet packet);
        void confirmNext();
        std::uint32_t amount();
        Packet next();
        void unconfirm(std::uint32_t start_id, std::uint32_t end_id);
};

#endif