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

#include "apps/mode4App/Mode4BaseApp.h"
#include "common/LteCommon.h"

/**
 * First we have to initialize the module from which we derived ours,
 * in this case BaseLayer.
 *
 * Then we have to intialize the gates and - if necessary - some own
 * variables.
 **/

void Mode4BaseApp::initialize(int stage)
{
    cSimpleModule::initialize(stage);
    if (stage == inet::INITSTAGE_LOCAL) {
        lowerGateIn_ = findGate("lowerGateIn");
        lowerGateOut_ = findGate("lowerGateOut");
    }
}

void Mode4BaseApp::handleMessage(cMessage* msg)
{
    if (msg->isSelfMessage()) {
        handleSelfMessage(msg);
    }
    else if (msg->getArrivalGateId() == lowerGateIn_) {
        handleLowerMessage(msg);
    }
    else if (msg->getArrivalGateId() == -1) {
        /* Classes extending this class may not use all the gates, f.e.
         * BaseApplLayer has no upper gates. In this case all upper gate-
         * handles are initialized to -1. When getArrivalGateId() equals -1,
         * it would be wrong to forward the message to one of these gates,
         * as they actually don't exist, so raise an error instead.
         */
        throw cRuntimeError("No self message and no gateID?? Check configuration.");
    }
    else {
        /* msg->getArrivalGateId() should be valid, but it isn't recognized
         * here. This could signal the case that this class is extended
         * with extra gates, but handleMessage() isn't overridden to
         * check for the new gate(s).
         */
        throw cRuntimeError("Unknown gateID?? Check configuration or override handleMessage().");
    }
}

void Mode4BaseApp::handleLowerMessage(cMessage* msg){
}

void Mode4BaseApp::handleSelfMessage(cMessage* msg){
}

void Mode4BaseApp::sendLowerPackets(cPacket* pkt)
{
    send(pkt, lowerGateOut_);
}

void Mode4BaseApp::finish()
{
}

Mode4BaseApp::~Mode4BaseApp()
{
}
