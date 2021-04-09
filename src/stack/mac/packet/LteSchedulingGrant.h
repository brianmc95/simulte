//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//

#ifndef SCHEDULING_GRANT_H_
#define SCHEDULING_GRANT_H_

#include "stack/mac/packet/LteSchedulingGrant_m.h"
#include "common/LteCommon.h"
#include "stack/mac/amc/UserTxParams.h"

class UserTxParams;

class LteSchedulingGrant : public LteSchedulingGrant_Base
{
  protected:

    const UserTxParams* userTxParams;
    RbMap grantedBlocks;
    std::vector<unsigned int> grantedCwBytes;
    Direction direction_;

  public:

    LteSchedulingGrant(const char *name = NULL, int kind = 0) :
        LteSchedulingGrant_Base(name, kind)
    {
        userTxParams = NULL;
        grantedCwBytes.resize(MAX_CODEWORDS);
    }

    ~LteSchedulingGrant()
    {
        if (userTxParams != NULL)
        {
            delete userTxParams;
            userTxParams = NULL;
        }
    }

    LteSchedulingGrant(const LteSchedulingGrant& other) :
        LteSchedulingGrant_Base(other.getName())
    {
        operator=(other);
    }

    LteSchedulingGrant& operator=(const LteSchedulingGrant& other)
    {
        if (other.userTxParams != NULL)
        {
            const UserTxParams* txParams = check_and_cast<const UserTxParams*>(other.userTxParams);
            userTxParams = txParams->dup();
        }
        else
        {
            userTxParams = 0;
        }
        grantedBlocks = other.grantedBlocks;
        grantedCwBytes = other.grantedCwBytes;
        direction_ = other.direction_;
        LteSchedulingGrant_Base::operator=(other);
        return *this;
    }

    virtual LteSchedulingGrant *dup() const
    {
        return new LteSchedulingGrant(*this);
    }

    void setUserTxParams(const UserTxParams* arg)
    {
        if(userTxParams){
            delete userTxParams;
        }
        userTxParams = arg;
    }

    const UserTxParams* getUserTxParams() const
    {
        return userTxParams;
    }

    const unsigned int getBlocks(Remote antenna, Band b) const
        {
        return grantedBlocks.at(antenna).at(b);
    }

    void setBlocks(Remote antenna, Band b, const unsigned int blocks)
    {
        grantedBlocks[antenna][b] = blocks;
    }

    const RbMap& getGrantedBlocks() const
    {
        return grantedBlocks;
    }

    void setGrantedBlocks(const RbMap& rbMap)
    {
        grantedBlocks = rbMap;
    }

    virtual void setGrantedCwBytesArraySize(unsigned int size)
    {
        grantedCwBytes.resize(size);
    }
    virtual unsigned int getGrantedCwBytesArraySize() const
    {
        return grantedCwBytes.size();
    }
    virtual unsigned int getGrantedCwBytes(unsigned int k) const
    {
        return grantedCwBytes.at(k);
    }
    virtual void setGrantedCwBytes(unsigned int k, unsigned int grantedCwBytes_var)
    {
        grantedCwBytes[k] = grantedCwBytes_var;
    }
    void setDirection(Direction dir)
    {
        direction_ = dir;
    }
    Direction getDirection() const
    {
        return direction_;
    }
};

class LteMode4SchedulingGrant : public LteSchedulingGrant
{
protected:
    simtime_t startTime;
    std::vector<double> possibleRRIs;
    bool retransmission;
    bool firstTransmission;
    unsigned int timeGapTransRetrans;
    unsigned int spsPriority;
    unsigned int numSubchannels;
    unsigned int maximumLatency;
    unsigned int startingSubchannel;
    unsigned int mcs;
    unsigned int retransSubchannel; // It is possible the retransmission has different resources assigned to it.
    unsigned int resourceReselectionCounter;

public:

    LteMode4SchedulingGrant(const char *name = NULL, int kind = 0) :
        LteSchedulingGrant(name, kind)
    {
        numSubchannels = 0;
        spsPriority = 0;
        maximumLatency = 0;
        timeGapTransRetrans = 0;
        startingSubchannel = 0;
        mcs = 0;
        retransSubchannel = 0;
        resourceReselectionCounter = 0;
        firstTransmission = true;
        startTime = simTime();
    }


    ~LteMode4SchedulingGrant()
    {
    }

    LteMode4SchedulingGrant(const LteMode4SchedulingGrant& other) :
        LteSchedulingGrant(other.getName())
    {
        operator=(other);
    }

    LteMode4SchedulingGrant& operator=(const LteMode4SchedulingGrant& other)
    {
        numSubchannels = other.numSubchannels;
        spsPriority = other.spsPriority;
        startTime = other.startTime;
        maximumLatency = other.maximumLatency;
        timeGapTransRetrans = other.timeGapTransRetrans;
        startingSubchannel = other.startingSubchannel;
        mcs = other.mcs;
        retransSubchannel = other.retransSubchannel;
        resourceReselectionCounter = other.resourceReselectionCounter;
        possibleRRIs = other.possibleRRIs;
        LteSchedulingGrant::operator=(other);
        return *this;
    }

    virtual LteMode4SchedulingGrant *dup() const
    {
        return new LteMode4SchedulingGrant(*this);
    }

    void setStartTime(simtime_t start)
    {
        startTime = start;
    }
    simtime_t getStartTime() const
    {
        return startTime;
    }
    void setSpsPriority(unsigned int priority)
    {
        spsPriority = priority;
    }
    unsigned int getSpsPriority() const
    {
        return spsPriority;
    }
    void setNumberSubchannels(unsigned int subchannels)
    {
        numSubchannels = subchannels;
    }
    unsigned int getNumSubchannels() const
    {
        return numSubchannels;
    }
    void setMaximumLatency(unsigned int maxLatency)
    {
        maximumLatency = maxLatency;
    }
    unsigned int getMaximumLatency() const
    {
        return maximumLatency;
    }
    void setTimeGapTransRetrans(unsigned int timeGapTransRetrans)
    {
        this->timeGapTransRetrans = timeGapTransRetrans;
    }
    unsigned int getTimeGapTransRetrans() const
    {
        return timeGapTransRetrans;
    }
    void setStartingSubchannel(unsigned int subchannelIndex)
    {
        this->startingSubchannel = subchannelIndex;
    }
    unsigned int getStartingSubchannel() const
    {
        return startingSubchannel;
    }
    void setMcs(unsigned int mcs)
    {
        this->mcs = mcs;
    }
    unsigned int getMcs() const
    {
        return mcs;
    }
    void setRetransSubchannel(unsigned int retransSubchannel)
    {
        this->retransSubchannel = retransSubchannel;
    }
    unsigned int getRetransSubchannel() const
    {
        return retransSubchannel;
    }
    void setResourceReselectionCounter(unsigned int resourceReselectionCounter)
    {
        this->resourceReselectionCounter = resourceReselectionCounter;
    }
    unsigned int getResourceReselectionCounter() const
    {
        return resourceReselectionCounter;
    }
    void setRetransmission(bool retransmission)
    {
        this->retransmission = retransmission;
    }
    bool getRetransmission() const
    {
        return retransmission;
    }
    std::vector<double> getPossibleRRIs()
    {
        return possibleRRIs;
    }
    void setPossibleRRIs(std::vector<double> RRIs)
    {
        this->possibleRRIs = RRIs;
    }
    bool getFirstTransmission() const
    {
        return firstTransmission;
    }
    void setFirstTransmission(bool firstTransmission)
    {
        this->firstTransmission = firstTransmission;
    }
};

#endif
