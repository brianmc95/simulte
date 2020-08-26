//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//
// This file is an extension of SimuLTE
// Author: Brian McCarthy
// email : b.mccarthy@cs.ucc.ie

#ifndef _LTE_LTEMACVUEMODE4COUNTER_H_
#define _LTE_LTEMACVUEMODE4COUNTER_H_

#include "stack/mac/layer/LteMacVUeMode4.h"
#include "corenetwork/deployer/LteDeployer.h"
#include <unordered_map>

//class LteMode4SchedulingGrant;

class LteMacVUeMode4Counter: public LteMacVUeMode4 {

protected:

    /**
     * Generate a scheduling grant
     */
    virtual void macGenerateSchedulingGrant(double maximumLatency, int priority);


    /**
     * Reads MAC parameters for ue and performs initialization.
     */
    virtual void initialize(int stage);

    /**
     * Analyze gate of incoming packet
     * and call proper handler
     */
    virtual void handleMessage(cMessage *msg);

    /**
     * Main loop
     */
    virtual void handleSelfMessage();

    /**
     * Purges PDUs from the HARQ buffers for sending to the PHY layer.
     */
    void flushHarqBuffers();

    void finish();

public:
    LteMacVUeMode4Counter();
    virtual ~LteMacVUeMode4Counter();

    virtual bool isD2DCapable()
    {
        return true;
    }
};

#endif /* _LTE_LTEMACVUEMODE4COUNTER_H_ */
