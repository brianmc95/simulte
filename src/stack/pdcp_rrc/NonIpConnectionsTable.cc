// 
//                           SimuLTE
// 
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself, 
// and cannot be removed from it.
// 

#include "stack/pdcp_rrc/NonIpConnectionsTable.h"

NonIpConnectionsTable::NonIpConnectionsTable()
{
    // Table is resetted by putting all fields equal to 0xFF
    memset(NonIpHt_, 0xFF, sizeof(struct entry_) * TABLE_SIZE);
}

unsigned int NonIpConnectionsTable::hash_func(long srcAddr, long dstAddr)
{
    return (srcAddr | dstAddr) % TABLE_SIZE;
}

unsigned int NonIpConnectionsTable::hash_func(long srcAddr, long dstAddr, uint16_t dir)
{
    return (srcAddr | dstAddr | dir) % TABLE_SIZE;
}

LogicalCid NonIpConnectionsTable::find_entry(long srcAddr, long dstAddr)
{
    int hashIndex = hash_func(srcAddr, dstAddr);
    while (1)
    {
        if (NonIpHt_[hashIndex].lcid_ == 0xFFFF)            // Entry not found
            return 0xFFFF;
        if (NonIpHt_[hashIndex].srcAddr_ == srcAddr &&
            NonIpHt_[hashIndex].dstAddr_ == dstAddr)
            return NonIpHt_[hashIndex].lcid_;                // Entry found
        hashIndex = (hashIndex + 1) % TABLE_SIZE;    // Linear scanning of the hash table
    }
}

LogicalCid NonIpConnectionsTable::find_entry(long srcAddr, long dstAddr, uint16_t dir)
{
    int hashIndex = hash_func(srcAddr, dstAddr, dir);
    while (1)
    {
        if (NonIpHt_[hashIndex].lcid_ == 0xFFFF)            // Entry not found
            return 0xFFFF;
        if (NonIpHt_[hashIndex].srcAddr_ == srcAddr &&
            NonIpHt_[hashIndex].dstAddr_ == dstAddr &&
            NonIpHt_[hashIndex].dir_ == dir)
            return NonIpHt_[hashIndex].lcid_;                // Entry found
        hashIndex = (hashIndex + 1) % TABLE_SIZE;    // Linear scanning of the hash table
    }
}

void NonIpConnectionsTable::create_entry(long srcAddr, long dstAddr, LogicalCid lcid)
{
    int hashIndex = hash_func(srcAddr, dstAddr);
    while (NonIpHt_[hashIndex].lcid_ != 0xFFFF)
        hashIndex = (hashIndex + 1) % TABLE_SIZE;    // Linear scanning of the hash table
    NonIpHt_[hashIndex].srcAddr_ = srcAddr;
    NonIpHt_[hashIndex].dstAddr_ = dstAddr;
    NonIpHt_[hashIndex].lcid_ = lcid;
    return;
}

void NonIpConnectionsTable::create_entry(long srcAddr, long dstAddr, uint16_t dir, LogicalCid lcid)
{
    int hashIndex = hash_func(srcAddr, dstAddr, dir);
    while (NonIpHt_[hashIndex].lcid_ != 0xFFFF)
        hashIndex = (hashIndex + 1) % TABLE_SIZE;    // Linear scanning of the hash table
    NonIpHt_[hashIndex].srcAddr_ = srcAddr;
    NonIpHt_[hashIndex].dstAddr_ = dstAddr;
    NonIpHt_[hashIndex].dir_ = dir;
    NonIpHt_[hashIndex].lcid_ = lcid;
    return;
}

NonIpConnectionsTable::~NonIpConnectionsTable()
{
    memset(NonIpHt_, 0xFF, sizeof(struct entry_) * TABLE_SIZE);
}
