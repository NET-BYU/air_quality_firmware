#ifndef PERSISTENT_ACK_TRACKER_H_
#define PERSISTENT_ACK_TRACKER_H_

#include "AckTracker.h"

/*
    * An abstract class/interface for persistent acknowledgement trackers.
    * 
    * Any data pushed to this tracker will persist, even after it is
    * confirmed/acknowledged.
    * 
    * Data that was previously confirmed/acknowledged and removed from
    * the tracker can be "unconfirmed"/"unacknowledged" and re-added to the tracker.
    * */
class PersistentAckTracker: public AckTracker
{
    public:
        virtual void unconfirm(std::uint32_t start_id, std::uint32_t end_id) = 0;
};

#endif