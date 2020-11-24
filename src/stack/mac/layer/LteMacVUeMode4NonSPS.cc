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
 * LteMacVUeMode4NonSPS is a new model which implements the functionality of LTE Mode 4 as per 3GPP release 14
 * Author: Brian McCarthy
 * Email: b.mccarthy@cs.ucc.ie
 */

#include "stack/mac/buffer/harq/LteHarqBufferRx.h"
#include "stack/mac/buffer/LteMacQueue.h"
#include "stack/mac/buffer/harq_d2d/LteHarqBufferRxD2DMirror.h"
#include "stack/mac/layer/LteMacVUeMode4NonSPS.h"
#include "stack/mac/scheduler/LteSchedulerUeUl.h"
#include "stack/phy/packet/SpsCandidateResources.h"
#include "stack/phy/packet/cbr_m.h"
#include "stack/phy/layer/Subchannel.h"
#include "stack/mac/amc/AmcPilotD2D.h"
#include "common/LteCommon.h"
#include "stack/phy/layer/LtePhyBase.h"
#include "inet/networklayer/common/InterfaceEntry.h"
#include "inet/common/ModuleAccess.h"
#include "inet/networklayer/ipv4/IPv4InterfaceData.h"
#include "stack/mac/amc/LteMcs.h"
#include <map>

Define_Module(LteMacVUeMode4NonSPS);

LteMacVUeMode4NonSPS::LteMacVUeMode4NonSPS() :
    LteMacVUeMode4()
{
}

LteMacVUeMode4NonSPS::~LteMacVUeMode4NonSPS()
{
}

void LteMacVUeMode4NonSPS::initialize(int stage)
{
    LteMacVUeMode4::initialize(stage);

    if (stage == inet::INITSTAGE_LOCAL)
    {
        // Do all the local stuff if we even need any in reality
    }
}

void LteMacVUeMode4NonSPS::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage())
    {
        LteMacVUeMode4::handleMessage(msg);
        return;
    }

    cPacket* pkt = check_and_cast<cPacket *>(msg);
    cGate* incoming = pkt->getArrivalGate();

    if (incoming == down_[IN])
    {
        if (strcmp(pkt->getName(), "CSRs") == 0)
        {
            // This should not happen in this case
            throw cRuntimeError("LteMacVUeMode4NonSPS::HandleMessage - Should not receive CSRs. Aborting");
        }
        else if (strcmp(pkt->getName(), "CBR") == 0)
        {
            LteMacVUeMode4::handleMessage(msg);
            return;
        }
    }
    else if (incoming == up_[IN])
    {
        if (strcmp(pkt->getName(), "newDataPkt")== 0)
        {
            FlowControlInfoNonIp* lteInfo = check_and_cast<FlowControlInfoNonIp*>(pkt->removeControlInfo());
            receivedTime_ = NOW;
            simtime_t elapsedTime = receivedTime_ - lteInfo->getCreationTime();
            simtime_t duration = SimTime(lteInfo->getDuration(), SIMTIME_MS);
            duration = duration - elapsedTime;
            double dur = duration.dbl();
            remainingTime_ = lteInfo->getDuration() - dur;

            if (schedulingGrant_ == NULL)
            {
                macGenerateSchedulingGrant(remainingTime_, lteInfo->getPriority());
            }
            else if ((schedulingGrant_ != NULL && periodCounter_ > remainingTime_) || alwaysReschedule_)
            {
                emit(grantBreakTiming, 1);
                delete schedulingGrant_;
                schedulingGrant_ = NULL;
                macGenerateSchedulingGrant(remainingTime_, lteInfo->getPriority());
            }
            else
            {
                LteMode4SchedulingGrant* mode4Grant = check_and_cast<LteMode4SchedulingGrant*>(schedulingGrant_);
                mode4Grant->setSpsPriority(lteInfo->getPriority());
                // Need to get the creation time for this
                mode4Grant->setMaximumLatency(remainingTime_);
            }
            // Need to set the size of our grant to the correct size we need to ask rlc for, i.e. for the sdu size.
            schedulingGrant_->setGrantedCwBytes((MAX_CODEWORDS - currentCw_), pkt->getBitLength());

            pkt->setControlInfo(lteInfo);
        }
    }

    LteMacUeRealisticD2D::handleMessage(msg);
}



void LteMacVUeMode4NonSPS::handleSelfMessage() {
    EV << "----- UE MAIN LOOP -----" << endl;

    // extract pdus from all harqrxbuffers and pass them to unmaker
    HarqRxBuffers::iterator hit = harqRxBuffers_.begin();
    HarqRxBuffers::iterator het = harqRxBuffers_.end();
    LteMacPdu *pdu = NULL;
    std::list < LteMacPdu * > pduList;

    for (; hit != het; ++hit) {
        pduList = hit->second->extractCorrectPdus();
        while (!pduList.empty()) {
            pdu = pduList.front();
            pduList.pop_front();
            macPduUnmake(pdu);
        }
    }

    EV << NOW << "LteMacVUeMode4NonSPS::handleSelfMessage " << nodeId_ << " - HARQ process "
       << (unsigned int) currentHarq_ << endl;
    // updating current HARQ process for next TTI

    //unsigned char currentHarq = currentHarq_;

    // no grant available - if user has backlogged data, it will trigger scheduling request
    // no harq counter is updated since no transmission is sent.

    LteMode4SchedulingGrant *mode4Grant = dynamic_cast<LteMode4SchedulingGrant *>(schedulingGrant_);

    if (mode4Grant == NULL) {
        EV << NOW << " LteMacVUeMode4NonSPS::handleSelfMessage " << nodeId_ << " NO configured grant" << endl;

        // No configured Grant simply continue
    } else if (mode4Grant->getPeriodic() && mode4Grant->getStartTime() <= NOW) {
        // Periodic checks
        if (--expirationCounter_ == mode4Grant->getPeriod()) {
            // Gotten to the point of the final tranmission must determine if we reselect or not.
            double randomReReserve = dblrand(1);
            if (randomReReserve < probResourceKeep_) {
                int expiration = 0;
                if (resourceReservationInterval_ == 0.5) {
                    expiration = intuniform(10, 30, 3);
                } else if (resourceReservationInterval_ == 0.2) {
                    expiration = intuniform(25, 75, 3);
                } else {
                    expiration = intuniform(5, 15, 3);
                }
                mode4Grant->setResourceReselectionCounter(expiration);
                mode4Grant->setFirstTransmission(true);
                expirationCounter_ = expiration * mode4Grant->getPeriod();
                emit(rrcSelected, expiration);
                emit(retainGrant, 1);
            }
        }
        if (--periodCounter_ > 0 && !mode4Grant->getFirstTransmission()) {
            return;
        } else if (expirationCounter_ > 0) {
            // resetting grant period
            periodCounter_ = mode4Grant->getPeriod();
            // this is periodic grant TTI - continue with frame sending
        } else if (expirationCounter_ <= 0) {
            emit(grantBreak, 1);
            mode4Grant->setExpiration(0);
            expiredGrant_ = true;
        }
    }
    bool requestSdu = false;
    if (mode4Grant != NULL && mode4Grant->getStartTime() <= NOW) // if a grant is configured
    {
        if (mode4Grant->getFirstTransmission()) {
            mode4Grant->setFirstTransmission(false);
            // Need to ensure the grant is correctly broken at this point
            emit(grantBreak, 1);
            mode4Grant->setExpiration(0);
            expiredGrant_ = true;

        }
        if (!firstTx) {
            EV << "\t currentHarq_ counter initialized " << endl;
            firstTx = true;
            // the eNb will receive the first pdu in 2 TTI, thus initializing acid to 0
//            currentHarq_ = harqRxBuffers_.begin()->second->getProcesses() - 2;
            currentHarq_ = UE_TX_HARQ_PROCESSES - 2;
        }
        EV << "\t " << schedulingGrant_ << endl;

        EV << NOW << " LteMacVUeMode4NonSPS::handleSelfMessage " << nodeId_ << " entered scheduling" << endl;

        bool retx = false;
        bool availablePdu = false;

        HarqTxBuffers::iterator it2;
        LteHarqBufferTx *currHarq;
        for (it2 = harqTxBuffers_.begin(); it2 != harqTxBuffers_.end(); it2++) {
            EV << "\t Looking for retx in acid " << (unsigned int) currentHarq_ << endl;
            currHarq = it2->second;

            // check if the current process has unit ready for retx
            retx = currHarq->getProcess(currentHarq_)->hasReadyUnits();
            CwList cwListRetx = currHarq->getProcess(currentHarq_)->readyUnitsIds();

            if (it2->second->isSelected()) {
                LteHarqProcessTx *selectedProcess = it2->second->getSelectedProcess();
                // Ensure that a pdu is not already on the HARQ buffer awaiting sending.
                if (selectedProcess != NULL) {
                    for (int cw = 0; cw < MAX_CODEWORDS; cw++) {
                        if (selectedProcess->getPduLength(cw) != 0) {
                            availablePdu = true;
                        }
                    }
                }
            }

            EV << "\t [process=" << (unsigned int) currentHarq_ << "] , [retx=" << ((retx) ? "true" : "false")
               << "] , [n=" << cwListRetx.size() << "]" << endl;

            // if a retransmission is needed
            if (retx) {
                UnitList signal;
                signal.first = currentHarq_;
                signal.second = cwListRetx;
                currHarq->markSelected(signal, schedulingGrant_->getUserTxParams()->getLayers().size());
            }
        }
        // if no retx is needed, proceed with normal scheduling
        // TODO: This may yet be changed to appear after MCS selection, issue is that if you pick max then you might get more sdus then you want
        // Basing it on the previous mcs value is at least more realistic as to the size of the pdu you will get.
        if (!retx && !availablePdu) {
            scheduleList_ = lcgScheduler_->schedule();
            bool sent = macSduRequest();

            if (!sent) {
                macPduMake();
            }

            requestSdu = sent;
        }
        // Message that triggers flushing of Tx H-ARQ buffers for all users
        // This way, flushing is performed after the (possible) reception of new MAC PDUs
        cMessage *flushHarqMsg = new cMessage("flushHarqMsg");
        flushHarqMsg->setSchedulingPriority(1);        // after other messages
        scheduleAt(NOW, flushHarqMsg);
    }
    //============================ DEBUG ==========================
    HarqTxBuffers::iterator it;

    EV << "\n htxbuf.size " << harqTxBuffers_.size() << endl;

    int cntOuter = 0;
    int cntInner = 0;
    for (it = harqTxBuffers_.begin(); it != harqTxBuffers_.end(); it++) {
        LteHarqBufferTx *currHarq = it->second;
        BufferStatus harqStatus = currHarq->getBufferStatus();
        BufferStatus::iterator jt = harqStatus.begin(), jet = harqStatus.end();

        EV_DEBUG << "\t cicloOuter " << cntOuter << " - bufferStatus.size=" << harqStatus.size() << endl;
        for (; jt != jet; ++jt) {
            EV_DEBUG << "\t\t cicloInner " << cntInner << " - jt->size=" << jt->size()
                     << " - statusCw(0/1)=" << jt->at(0).second << "/" << jt->at(1).second << endl;
        }
    }
    //======================== END DEBUG ==========================

    unsigned int purged = 0;
    // purge from corrupted PDUs all Rx H-HARQ buffers
    for (hit = harqRxBuffers_.begin(); hit != het; ++hit) {
        // purge corrupted PDUs only if this buffer is for a DL transmission. Otherwise, if you
        // purge PDUs for D2D communication, also "mirror" buffers will be purged
        if (hit->first == cellId_)
            purged += hit->second->purgeCorruptedPdus();
    }
    EV << NOW << " LteMacVUeMode4NonSPS::handleSelfMessage Purged " << purged << " PDUS" << endl;

    if (!requestSdu) {
        // update current harq process id
        currentHarq_ = (currentHarq_ + 1) % harqProcesses_;
    }

    EV << "--- END UE MAIN LOOP ---" << endl;
}

void LteMacVUeMode4NonSPS::macGenerateSchedulingGrant(double maximumLatency, int priority)
{
    /**
     * 1. Packet priority
     * 2. Resource reservation interval
     * 3. Maximum latency
     * 4. Number of subchannels
     * 6. Send message to PHY layer looking for CSRs
     */

    LteMode4SchedulingGrant* mode4Grant = new LteMode4SchedulingGrant("LteMode4Grant");

    // Priority is the most difficult part to figure out, for the moment I will assign it as a fixed value
    mode4Grant -> setSpsPriority(priority);
    mode4Grant -> setPeriod(resourceReservationInterval_ * 100);
    mode4Grant -> setMaximumLatency(maximumLatency);
    mode4Grant -> setPossibleRRIs(validResourceReservationIntervals_);

    int minSubchannelNumberPSSCH = minSubchannelNumberPSSCH_;
    int maxSubchannelNumberPSSCH = maxSubchannelNumberPSSCH_;
    double resourceReservationInterval = resourceReservationInterval_;

    // Selecting the number of subchannel at random as there is no explanation as to the logic behind selecting the resources in the range unlike when selecting MCS.
    int numSubchannels = intuniform(minSubchannelNumberPSSCH, maxSubchannelNumberPSSCH, 2);

    mode4Grant -> setNumberSubchannels(numSubchannels);
    mode4Grant -> setResourceReselectionCounter(0);
    mode4Grant -> setExpiration(0);
    emit(rrcSelected, 0);

    simtime_t selectedStartTime = (simTime() + SimTime(1, SIMTIME_MS) - TTI).trunc(SIMTIME_MS);

    emit(grantStartTime, selectedStartTime);

    // Need to randomly select this or do I allow the PHY layer to do this as well, probably PHY
    int initialSubchannel = 0;
    int finalSubchannel = initialSubchannel + mode4Grant->getNumSubchannels(); // Is this actually one additional subchannel?

    // Emit statistic about the use of resources, i.e. the initial subchannel and it's length.
    emit(selectedSubchannelIndex, initialSubchannel);
    emit(selectedNumSubchannels, mode4Grant->getNumSubchannels());

    int totalGrantedBlocks = 0;
    if (adjacencyPSCCHPSSCH_){
        totalGrantedBlocks = (numSubchannels * subchannelSize_) - 2; // Account for SCI
    } else {
        totalGrantedBlocks = numSubchannels * subchannelSize_;
    }


    mode4Grant->setStartTime(selectedStartTime);
    mode4Grant->setPeriodic(false);
    mode4Grant->setTotalGrantedBlocks(totalGrantedBlocks); // account for the 2 RBs used for the sci message
    mode4Grant->setDirection(D2D_MULTI);
    mode4Grant->setCodewords(1);
    mode4Grant->setStartingSubchannel(initialSubchannel);
    mode4Grant->setMcs(maxMCSPSSCH_);

    LteMod mod = _QPSK;
    if (maxMCSPSSCH_ > 9 && maxMCSPSSCH_ < 17)
    {
        mod = _16QAM;
    }
    else if (maxMCSPSSCH_ > 16 && maxMCSPSSCH_ < 29 )
    {
        mod = _64QAM;
    }

    unsigned int i = (mod == _QPSK ? 0 : (mod == _16QAM ? 9 : (mod == _64QAM ? 15 : 0)));

    const unsigned int* tbsVect = itbs2tbs(mod, SINGLE_ANTENNA_PORT0, 1, maxMCSPSSCH_ - i);
    maximumCapacity_ = tbsVect[totalGrantedBlocks-1];
    mode4Grant->setGrantedCwBytes(currentCw_, maximumCapacity_);

    periodCounter_= mode4Grant->getPeriod();
    expirationCounter_= (mode4Grant->getResourceReselectionCounter() * periodCounter_) + 1;

    schedulingGrant_ = mode4Grant;
}

void LteMacVUeMode4NonSPS::flushHarqBuffers()
{
    // send the selected units to lower layers
    // First make sure packets are sent down
    // HARQ retrans needs to be taken into account
    // Maintain unit list maybe and that causes retrans?
    // But purge them once all messages sent.

    LteMode4SchedulingGrant* mode4Grant = dynamic_cast<LteMode4SchedulingGrant*>(schedulingGrant_);

    int period = 0;
    if (schedulingGrant_ != NULL){
        period = schedulingGrant_->getPeriod();
    }

    // Ensure CR updated.
    channelOccupancyRatio_ = calculateChannelOccupancyRatio(period);

    HarqTxBuffers::iterator it2;
    for(it2 = harqTxBuffers_.begin(); it2 != harqTxBuffers_.end(); it2++)
    {
        std::unordered_map<std::string,double> cbrMap = cbrPSSCHTxConfigList_.at(currentCbrIndex_);
        std::unordered_map<std::string,double>::const_iterator got;

        if (packetDropping_) {
            double crLimit;
            got = cbrMap.find("cr-Limit");
            if (got == cbrMap.end())
                crLimit = 1;
            else
                crLimit = got->second;

            if (channelOccupancyRatio_ > crLimit) {
                // Need to drop the unit currently selected
                UnitList ul = it2->second->firstAvailable();
                it2->second->forceDropProcess(ul.first);
                emit(packetDropDCC, 1);
            }
        }

        if (it2->second->isSelected())
        {
            LteHarqProcessTx* selectedProcess = it2->second->getSelectedProcess();
            for (int cw=0; cw<MAX_CODEWORDS; cw++)
            {
                int pduLength = selectedProcess->getPduLength(cw) * 8;
                int minMCS = minMCSPSSCH_;
                int maxMCS = maxMCSPSSCH_;
                if (pduLength > 0)
                {
                    if (useCBR_){
                        int cbrMinMCS;
                        int cbrMaxMCS;

                        got = cbrMap.find("minMCS-PSSCH");
                        if ( got == cbrMap.end() )
                            cbrMinMCS = minMCSPSSCH_;
                        else
                            cbrMinMCS = (int)got->second;

                        got = cbrMap.find("maxMCS-PSSCH");
                        if ( got == cbrMap.end() )
                            cbrMaxMCS = maxMCSPSSCH_;
                        else
                            cbrMaxMCS = (int)got->second;

                        int rri = mode4Grant->getPeriod();
                        if (rriLookup_) {
                            // RRI Adaptation based on lookup table similar to DCC
                            got = cbrMap.find("allowedRRI");
                            if (got != cbrMap.end()) {
                                rri = (int) got->second * 100;
                            }
                        }
                        else if (crLimit_) {
                            got = cbrMap.find("cr-Limit");
                            // Calculate an RRI which ensures the channelOccupancyRatio reduces to the point that
                            // it remains within the cr-limit
                            int i = 0;
                            double newOccupancyRatio = channelOccupancyRatio_;
                            rri = (int) validResourceReservationIntervals_.at(i) * 100;
                            while (newOccupancyRatio > got->second && i < validResourceReservationIntervals_.size()){
                                rri = (int) validResourceReservationIntervals_.at(i) * 100;
                                newOccupancyRatio = calculateChannelOccupancyRatio(rri);
                                i++;
                            }
                        }

                        if (rri != mode4Grant->getPeriod()) {
                            periodCounter_ = rri;

                            if (periodCounter_ > expirationCounter_) {
                                // Gotten to the point of the final transmission must determine if we reselect or not.
                                double randomReReserve = dblrand(1);
                                if (randomReReserve > probResourceKeep_) {
                                    int expiration = 0;
                                    if (resourceReservationInterval_ == 0.5){
                                        expiration = intuniform(10, 30, 3);
                                    } else if (resourceReservationInterval_ == 0.2){
                                        expiration = intuniform(25, 75, 3);
                                    } else {
                                        if (rri / 100 > 5){
                                            // This ensures that in the case that our rri is higher than the minimum 5
                                            // that we ensure we send at least one more transmission
                                            expiration = intuniform(rri/100, 15, 3);
                                        } else {
                                            expiration = intuniform(5, 15, 3);
                                        }
                                    }

                                    mode4Grant->setResourceReselectionCounter(expiration);
                                    emit(rrcSelected, expiration);
                                    emit(retainGrant, 1);
                                    // This remains at the default RRI this ensures that grants don't live overly long if they return to lower RRIs
                                    expirationCounter_ = expiration * resourceReservationInterval_ * 100;
                                } else {
                                    emit(grantBreak, 1);
                                    mode4Grant->setExpiration(0);
                                    expiredGrant_ = true;
                                }
                            }
                        }

                        if (maxMCSPSSCH_ < cbrMinMCS || cbrMaxMCS < minMCSPSSCH_)
                        {
                            // No overlap therefore I will use the cbr values (this is left to the UE).
                            minMCS = cbrMinMCS;
                            maxMCS = cbrMaxMCS;
                        }
                        else
                        {
                            minMCS = max(minMCSPSSCH_, cbrMinMCS);
                            maxMCS = min(maxMCSPSSCH_, cbrMaxMCS);
                        }
                    }

                    bool foundValidMCS = false;
                    int totalGrantedBlocks = mode4Grant->getTotalGrantedBlocks();

                    int mcsCapacity = 0;
                    for (int mcs=minMCS; mcs <= maxMCS; mcs++)
                    {
                        LteMod mod = _QPSK;
                        if (maxMCSPSSCH_ > 9 && maxMCSPSSCH_ < 17)
                        {
                            mod = _16QAM;
                        }
                        else if (maxMCSPSSCH_ > 16 && maxMCSPSSCH_ < 29 )
                        {
                            mod = _64QAM;
                        }

                        unsigned int i = (mod == _QPSK ? 0 : (mod == _16QAM ? 9 : (mod == _64QAM ? 15 : 0)));

                        const unsigned int* tbsVect = itbs2tbs(mod, SINGLE_ANTENNA_PORT0, 1, mcs - i);
                        mcsCapacity = tbsVect[totalGrantedBlocks-1];

                        if (mcsCapacity > pduLength)
                        {
                            foundValidMCS = true;
                            mode4Grant->setMcs(mcs);
                            mode4Grant->setGrantedCwBytes(cw, mcsCapacity);

                            LteMode4SchedulingGrant* phyGrant = mode4Grant->dup();

                            UserControlInfo* uinfo = new UserControlInfo();
                            uinfo->setSourceId(getMacNodeId());
                            uinfo->setDestId(getMacNodeId());
                            uinfo->setFrameType(GRANTPKT);
                            uinfo->setTxNumber(1);
                            uinfo->setDirection(D2D_MULTI);
                            uinfo->setSubchannelNumber(mode4Grant->getStartingSubchannel());
                            uinfo->setSubchannelLength(mode4Grant->getNumSubchannels());
                            uinfo->setGrantStartTime(mode4Grant->getStartTime());

                            phyGrant->setControlInfo(uinfo);

                            // Send Grant to PHY layer for sci creation
                            sendLowerPackets(phyGrant);
                            // Send pdu to PHY layer for sending.
                            it2->second->sendSelectedDown();

                            // Log transmission to A calculation log
                            previousTransmissions_[NOW.dbl()] = mode4Grant->getNumSubchannels();

                            missedTransmissions_ = 0;

                            emit(selectedMCS, mcs);

                            break;
                        }
                    }
                    if (!foundValidMCS)
                    {
                        // Never found an MCS to satisfy the requirements of the message must regenerate grant
                        LteMode4SchedulingGrant* mode4Grant = check_and_cast<LteMode4SchedulingGrant*>(schedulingGrant_);
                        int priority = mode4Grant->getSpsPriority();
                        int latency = mode4Grant->getMaximumLatency();
                        simtime_t elapsedTime = NOW - receivedTime_;
                        remainingTime_ -= elapsedTime.dbl();

                        emit(grantBreakSize, pduLength);
                        emit(maximumCapacity, mcsCapacity);

                        if (remainingTime_ <= 0)
                        {
                            //emit(droppedTimeout, 1);
                            selectedProcess->forceDropProcess();
                            delete schedulingGrant_;
                            schedulingGrant_ = NULL;
                        }
                        else
                        {
                            delete schedulingGrant_;
                            schedulingGrant_ = NULL;
                            macGenerateSchedulingGrant(remainingTime_, priority);
                        }
                    }
                }
                break;
            }
        }
        else
        {
            // if no transmission check if we need to break the grant.
            ++missedTransmissions_;
            emit(missedTransmission, 1);

            LteMode4SchedulingGrant* phyGrant = mode4Grant->dup();

            UserControlInfo* uinfo = new UserControlInfo();
            uinfo->setSourceId(getMacNodeId());
            uinfo->setDestId(getMacNodeId());
            uinfo->setFrameType(GRANTPKT);
            uinfo->setTxNumber(1);
            uinfo->setDirection(D2D_MULTI);

            phyGrant->setControlInfo(uinfo);

            if (missedTransmissions_ >= reselectAfter_)
            {
                phyGrant->setPeriod(0);
                phyGrant->setExpiration(0);

                delete schedulingGrant_;
                schedulingGrant_ = NULL;
                missedTransmissions_ = 0;

                emit(grantBreakMissedTrans, 1);
            }

            // Send Grant to PHY layer for sci creation
            sendLowerPackets(phyGrant);
        }
    }
    if (expiredGrant_) {
        // Grant has expired, only generate new grant on receiving next message to be sent.
        delete schedulingGrant_;
        schedulingGrant_ = NULL;
        expiredGrant_ = false;
    }
}

void LteMacVUeMode4NonSPS::finish()
{
    binder_->removeUeInfo(ueInfo_);

    delete preconfiguredTxParams_;
    delete ueInfo_;
}


