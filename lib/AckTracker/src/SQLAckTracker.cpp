#include "SQLAckTracker.h"

void SQLAckTracker::add(Packet packet)
{
    //
}

void SQLAckTracker::confirmNext()
{
    //
}

std::uint32_t SQLAckTracker::amount()
{
    return 0;
}

SQLAckTracker::Packet SQLAckTracker::next()
{
    return {0, 0, nullptr};
}

void SQLAckTracker::unconfirm(std::uint32_t start_id, std::uint32_t end_id)
{
    //
}
