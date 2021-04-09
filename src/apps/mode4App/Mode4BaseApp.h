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

/**
 * Mode4BaseApp is a new application developed to be used with Mode 4 based simulation
 * Author: Brian McCarthy
 * Email: b.mccarthy@cs.ucc.ie
 */

#ifndef _LTE_MODE4BASEAPP_H_
#define _LTE_MODE4BASEAPP_H_

#include <omnetpp.h>

#include "common/LteCommon.h"

class Mode4BaseApp : public cSimpleModule {
public:


protected:
    /**
     * @brief Length of the ApplPkt header
     **/
    int headerLength;

public:

protected:

    int lowerGateIn_;
    int lowerGateOut_;

    /**
     * @brief Handle messages from lower layer
     *
     * Redefine this function if you want to process messages from lower
     * layers.
     *
     * The basic application layer just silently deletes all messages it
     * receives.
     */

    virtual ~Mode4BaseApp();

    virtual int numInitStages() const { return inet::NUM_INIT_STAGES; }

    /**
     * Grabs NED parameters, initializes gates
     * and the TTI self message
     */
    virtual void initialize(int stage);

    /**
     * Analyze gate of incoming packet
     * and call proper handler
     */
    virtual void handleMessage(cMessage *msg);

    virtual void handleLowerMessage(cMessage *msg);


    /**
     * Statistics recording
     */
    virtual void finish();

    /**
     * Main loop of the Mac level, calls the scheduler
     * and every other function every TTI : must be reimplemented
     * by derivate classes
     */
    virtual void handleSelfMessage(cMessage *msg) = 0;

    /**
     * sendLowerPackets() is used
     * to send packets to lower layer
     *
     * @param pkt Packet to send
     */
    void sendLowerPackets(cPacket* pkt);

};

#endif
