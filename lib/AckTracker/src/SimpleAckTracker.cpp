#include "SimpleAckTracker.h"

using std::uint32_t;
using std::uint8_t;

/*
    * Add the given array of data to the AckTracker. Data is deep-copied, so
    * scope is irrelevant. 
    * */
void SimpleAckTracker::add(uint32_t id, uint8_t length, uint8_t *data)
{
    Packet *p = new Packet(id, length, data);   //Create a packet on the heap
    packets.push_back(p);                       //Push a pointer to the packet onto our AckTracker
}

/*
    * Data is retrieved by index. The oldest data in the AckTracker
    * has an index of 0, and the newest data has an index of
    * unconfirmedCount()-1.
    * Sets the id, length, and fills the provided data array with
    * a copy of the data. User is responsible for providing a big enough
    * array.
    *  */
void SimpleAckTracker::get(uint32_t index, uint32_t &id, uint8_t &length, uint8_t *data)
{
    std::list<Packet *>::iterator it = packets.begin(); //Set the iterator to the beginning of the packet list
    for (uint32_t i = 0; (i < index) && (it != packets.end()); i++) //Increase the iterator until we have reached the correct index
    {
        it++;
    }
    Packet *p = *it;    //Get the pointer to the packet at the given index
    id = p->id;         //"Return" the id and length by setting the variables passed in by reference
    length = p->length;
    for (uint8_t i = 0; i < p->length; i++) //Loop through the given array and fill it with data from our packet
    {
        data[i] = p->data[i];
    }
}

/*
    * Returns the length only of the data at the given index.
    * The oldest data in the AckTracker has an index of 0, and
    * the newest data has an index of unconfirmedCount()-1.
    * If the provided index reaches beyond the size of the AckTracker,
    * behavior is undefined.
    *  */
uint8_t SimpleAckTracker::getLengthOf(uint32_t index)
{
    std::list<Packet *>::iterator it = packets.begin(); //Set the iterator to the beginning of the packet list
    for (uint32_t i = 0; (i < index) && (it != packets.end()); i++) //Increase the iterator until we have reached the correct index
    {
        it++;
    }
    Packet *p = *it;    //Get the pointer to the packet at the given index
    return p->length;
}

/*
    * Returns the ID only of the data at the given index.
    * The oldest data in the AckTracker has an index of 0, and
    * the newest data has an index of unconfirmedCount()-1.
    *  */
uint32_t SimpleAckTracker::getIDOf(uint32_t index)
{
    std::list<Packet *>::iterator it = packets.begin(); //Set the iterator to the beginning of the packet list
    for (uint32_t i = 0; (i < index) && (it != packets.end()); i++) //Increase the iterator until we have reached the correct index
    {
        it++;
    }
    Packet *p = *it;    //Get the pointer to the packet at the given index
    return p->id;
}

/*
    * Returns the number of data packets that haven't been confirmed yet.
    *  */
uint32_t SimpleAckTracker::unconfirmedCount()
{
    return packets.size();
}

/*
    * Confirms the next n packets of data (starting with the oldest data at index=0),
    * where confirmCount = n.
    *  */
void SimpleAckTracker::confirmNext(uint32_t confirmCount)
{
    std::list<Packet *>::iterator it;
    for (uint32_t i = 0; (i < confirmCount) && (packets.size() > 0); i++) //Increase the iterator until we have confirmed the correct amount of packets, or until there are no more packets to confirm
    {
        it = packets.begin();   //Set the iterator to the beginning of the packet list
        Packet *p = *it;        //Get the pointer to the packet at the beginning
        delete [] p->data;      //Remove the packet's data from memory
        delete p;               //Remove the packet from memory
        packets.pop_front();    //Pop away the packet we just deleted from memory
    }
}

/*
    * Confirms the next packet of data (the oldest at index=0)
    *  */
void SimpleAckTracker::confirmNext()
{
    std::list<Packet *>::iterator it = packets.begin(); //Set the iterator to the beginning of the packet list
    Packet *p = *it;    //Get the pointer to the packet at the beginning
    delete [] p->data;  //Remove the packet's data from memory
    delete p;           //Remove the packet from memory
    packets.pop_front();//Pop away the packet we just deleted from memory
}

