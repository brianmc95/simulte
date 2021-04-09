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

#ifndef _LTE_MODE4APP_H_
#define _LTE_MODE4APP_H_

#include "apps/mode4App/Mode4BaseApp.h"
#include "apps/alert/AlertPacket_m.h"
#include "corenetwork/binder/LteBinder.h"


class Mode4App : public Mode4BaseApp {

public:
    ~Mode4App() override;

protected:
    //sender
    int size_;
    int nextSno_;
    int priority_;
    int duration_;
    simtime_t period_;

    simsignal_t sentMsg_;
    simsignal_t delay_;
    simsignal_t rcvdMsg_;
    simsignal_t cbr_;

    cMessage *selfSender_;

    LteBinder* binder_;
    MacNodeId nodeId_;

   int numInitStages() const { return inet::NUM_INIT_STAGES; }

   /**
    * Grabs NED parameters, initializes gates
    * and the TTI self message
    */
   void initialize(int stage);

   void handleLowerMessage(cMessage* msg);


   /**
    * Statistics recording
    */
   void finish();

   /**
    * Main loop of the Mac level, calls the scheduler
    * and every other function every TTI : must be reimplemented
    * by derivate classes
    */
   void handleSelfMessage(cMessage* msg);

   /**
    * sendLowerPackets() is used
    * to send packets to lower layer
    *
    * @param pkt Packet to send
    */
   void sendLowerPackets(cPacket* pkt);

};

#endif
