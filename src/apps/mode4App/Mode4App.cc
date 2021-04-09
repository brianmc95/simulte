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
 * Mode4App is a new application developed to be used with Mode 4 based simulation
 * Author: Brian McCarthy
 * Email: b.mccarthy@cs.ucc.ie
 */

#include "apps/mode4App/Mode4App.h"
#include "common/LteControlInfo.h"
#include "stack/phy/packet/cbr_m.h"

Define_Module(Mode4App);

void Mode4App::initialize(int stage)
{
    Mode4BaseApp::initialize(stage);
    if (stage==inet::INITSTAGE_LOCAL){
        // Register the node with the binder
        // Issue primarily is how do we set the link layer address

        // Get the binder
        binder_ = getBinder();

        // Get our UE
        cModule *ue = getParentModule();

        //Register with the binder
        nodeId_ = binder_->registerNode(ue, UE, 0);

        // Register the nodeId_ with the binder.
        binder_->setMacNodeId(nodeId_, nodeId_);
    } else if (stage==inet::INITSTAGE_APPLICATION_LAYER) {
        selfSender_ = NULL;
        nextSno_ = 0;

        selfSender_ = new cMessage("selfSender");

        size_ = par("packetSize");
        period_ = par("period");
        priority_ = par("priority");
        duration_ = par("duration");

        sentMsg_ = registerSignal("sentMsg");
        delay_ = registerSignal("delay");
        rcvdMsg_ = registerSignal("rcvdMsg");
        cbr_ = registerSignal("cbr");

<<<<<<< HEAD
//        starting the app at the same time for all veh
//        double delay = 0.001 ;

        double delay = 0.001 * intuniform(0, 1000, 0);

=======
        double delay = 0.001 * intuniform(0, 1000, 0);
>>>>>>> 51d6febab625bfdb30015c417012510f02f21c46
        scheduleAt((simTime() + delay).trunc(SIMTIME_MS), selfSender_);
    }
}

void Mode4App::handleLowerMessage(cMessage* msg)
{
    if (msg->isName("CBR")) {
        Cbr* cbrPkt = check_and_cast<Cbr*>(msg);
        double channel_load = cbrPkt->getCbr();
        emit(cbr_, channel_load);
        delete cbrPkt;
    } else {
        AlertPacket* pkt = check_and_cast<AlertPacket*>(msg);

        if (pkt == 0)
            throw cRuntimeError("Mode4App::handleMessage - FATAL! Error when casting to AlertPacket");

        // emit statistics
        simtime_t delay = simTime() - pkt->getTimestamp();
        emit(delay_, delay);
        emit(rcvdMsg_, (long)1);

        EV << "Mode4App::handleMessage - Packet received: SeqNo[" << pkt->getSno() << "] Delay[" << delay << "]" << endl;

        delete msg;
    }
}

void Mode4App::handleSelfMessage(cMessage* msg)
{
    if (!strcmp(msg->getName(), "selfSender")){
        // Replace method
        AlertPacket* packet = new AlertPacket("Alert");
        packet->setTimestamp(simTime());
        packet->setByteLength(size_);
        packet->setSno(nextSno_);

        nextSno_++;

        auto lteControlInfo = new FlowControlInfoNonIp();

        lteControlInfo->setSrcAddr(nodeId_);
        lteControlInfo->setDirection(D2D_MULTI);
        lteControlInfo->setPriority(priority_);
        lteControlInfo->setDuration(duration_);
        lteControlInfo->setCreationTime(simTime());

        packet->setControlInfo(lteControlInfo);

        Mode4BaseApp::sendLowerPackets(packet);
        emit(sentMsg_, (long)1);

        scheduleAt(simTime() + period_, selfSender_);
    }
    else
        throw cRuntimeError("Mode4App::handleMessage - Unrecognized self message");
}

void Mode4App::finish()
{
    cancelAndDelete(selfSender_);
}

Mode4App::~Mode4App()
{
    binder_->unregisterNode(nodeId_);
}
