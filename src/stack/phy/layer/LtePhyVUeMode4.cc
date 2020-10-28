//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//
// This file is an extension of SimuLTE
// Author: Brian McCarthy
// email : b.mccarthy@cs.ucc.ie


#include <math.h>
#include <assert.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include "stack/phy/layer/LtePhyVUeMode4.h"
#include "stack/phy/packet/LteFeedbackPkt.h"
#include "stack/d2dModeSelection/D2DModeSelectionBase.h"
#include "stack/phy/packet/SpsCandidateResources.h"
#include "stack/phy/packet/cbr_m.h"

Define_Module(LtePhyVUeMode4);

LtePhyVUeMode4::LtePhyVUeMode4()
{
    handoverStarter_ = NULL;
    handoverTrigger_ = NULL;
}

LtePhyVUeMode4::~LtePhyVUeMode4()
{
}

void LtePhyVUeMode4::initialize(int stage)
{
    if (stage != inet::INITSTAGE_NETWORK_LAYER_2)
        LtePhyUeD2D::initialize(stage);

    if (stage == inet::INITSTAGE_LOCAL)
    {
        adjacencyPSCCHPSSCH_             = par("adjacencyPSCCHPSSCH");
        randomScheduling_                = par("randomScheduling");
        sensingWindowSizeOverride_       = par("sensingWindowSizeOverride");
        pStep_                           = par("pStep");
        selectionWindowStartingSubframe_ = par("selectionWindowStartingSubframe");
        numSubchannels_                  = par("numSubchannels");
        subchannelSize_                  = par("subchannelSize");
        oneShotMechanism_                = par("oneShotMechanism");
        oneShotT2_                       = par("oneShotT2");
        oneShotT3_                       = par("oneShotT3");
        oneShotSensing_                  = false;
        d2dDecodingTimer_                = NULL;
        transmitting_                    = false;
        rssiFiltering_                   = par("rssiFiltering");
        rsrpFiltering_                   = par("rsrpFiltering");

        int thresholdRSSI                = par("thresholdRSSI");

        thresholdRSSI_ = (-112 + 2 * thresholdRSSI);

        d2dTxPower_ = par("d2dTxPower");
        if (d2dTxPower_ <= 0){
            d2dTxPower_ = txPower_;
        }

        // The threshold has a size of 64, and allowable values of 0 - 66
        // Deciding on this for now as it makes the most sense (low priority for both then more likely to take it)
        // High priority for both then less likely to take it.
        for (int i = 1; i < 66; i++)
        {
            ThresPSSCHRSRPvector_.push_back(i);
        }

        // SCI stats
        sciSent                     = registerSignal("sciSent");

        sciReceived                 = registerSignal("sciReceived");
        sciDecoded                  = registerSignal("sciDecoded");
        oneShot                     = registerSignal("oneShot");

        sciFailedDueToProp          = registerSignal("sciFailedDueToProp");
        sciFailedDueToInterference  = registerSignal("sciFailedDueToInterference");
        sciUnsensed                 = registerSignal("sciUnsensed");
        sciFailedHalfDuplex         = registerSignal("sciFailedHalfDuplex");

        txRxDistanceSCI             = registerSignal("txRxDistanceSCI");

        sciReceived_ = 0;
        sciDecoded_ = 0;
        oneShot_ = 0;

        sciFailedHalfDuplex_ = 0;
        sciFailedDueToProp_ = 0;
        sciFailedDueToInterference_ = 0;
        sciUnsensed_ = 0;


        // TB Stats
        tbSent                             = registerSignal("tbSent");

        tbReceived                         = registerSignal("tbReceived");
        tbDecoded                          = registerSignal("tbDecoded");

        periodic                           = registerSignal("periodic");

        tbFailedDueToNoSCI                 = registerSignal("tbFailedDueToNoSCI");
        tbFailedButSCIReceived             = registerSignal("tbFailedButSCIReceived");
        tbAndSCINotReceived                = registerSignal("tbAndSCINotReceived");

        tbFailedHalfDuplex                 = registerSignal("tbFailedHalfDuplex");
        tbFailedDueToProp                  = registerSignal("tbFailedDueToProp");
        tbFailedDueToInterference          = registerSignal("tbFailedDueToInterference");

        tbFailedDueToPropIgnoreSCI         = registerSignal("tbFailedDueToPropIgnoreSCI");
        tbFailedDueToInterferenceIgnoreSCI = registerSignal("tbFailedDueToInterferenceIgnoreSCI");
        tbDecodedIgnoreSCI                 = registerSignal("tbDecodedIgnoreSCI");

        txRxDistanceTB                     = registerSignal("txRxDistanceTB");

        tbReceived_ = 0;
        tbDecoded_ = 0;

        tbFailedDueToNoSCI_ = 0;
        tbFailedButSCIReceived_ = 0;
        tbAndSCINotReceived_ = 0;
        tbFailedHalfDuplex_ = 0;
        tbFailedDueToProp_ = 0;
        tbFailedDueToInterference_ = 0;
        tbFailedDueToPropIgnoreSCI_ = 0;
        tbFailedDueToInterferenceIgnoreSCI_ = 0;
        tbDecodedIgnoreSCI_ = 0;

        // General stats

        cbr                         = registerSignal("cbr");
        cbrPscch                    = registerSignal("cbrPscch");
        threshold                   = registerSignal("threshold");

        subchannelReceived          = registerSignal("subchannelReceived");
        subchannelsUsed             = registerSignal("subchannelsUsed");
        senderID                    = registerSignal("senderID");
        subchannelSent              = registerSignal("subchannelSent");
        subchannelsUsedToSend       = registerSignal("subchannelsUsedToSend");
        interPacketDelay            = registerSignal("interPacketDelay");
        posX                        = registerSignal("posX");
        posY                        = registerSignal("posY");

        subchannelReceived_ = 0;
        subchannelsUsed_ = 0;

        sensingWindowFront_ = 0; // Will ensure when we first update the sensing window we don't skip over the first element

        cbrCountDown_ = intuniform(0, 1000);
    }
    else if (stage == INITSTAGE_NETWORK_LAYER_2)
    {
        // Need to start initialising the sensingWindow
        deployer_ = getDeployer();
        int index = intuniform(0, binder_->phyPisaData.maxChannel() - 1);
        deployer_->lambdaInit(nodeId_, index);
        deployer_->channelUpdate(nodeId_, intuniform(1, binder_->phyPisaData.maxChannel2()));

        nodeId_ = getAncestorPar("macNodeId");

        initialiseSensingWindow();
    }
}

void LtePhyVUeMode4::handleSelfMessage(cMessage *msg)
{
    if (msg->isName("d2dDecodingTimer"))
    {
        std::sort(begin(sciInfo_), end(sciInfo_), [](const std::tuple<LteAirFrame*, std::vector<double>, std::vector<double>, std::vector<double>, double, double> &t1,
        const std::tuple<LteAirFrame*, std::vector<double>, std::vector<double>, std::vector<double>, double, double> &t2) {
            return get<5>(t1) > get<5>(t2); // or use a custom compare function
        });

        std::sort(begin(tbInfo_), end(tbInfo_), [](const std::tuple<LteAirFrame*, std::vector<double>, std::vector<double>, std::vector<double>, double, double> &t1,
        const std::tuple<LteAirFrame*, std::vector<double>, std::vector<double>, std::vector<double>, double, double> &t2) {
            return get<5>(t1) > get<5>(t2); // or use a custom compare function
        });

        std::vector<int> missingTbs;
        for (int i=0; i<sciInfo_.size(); i++){
            bool foundTB=false;
            LteAirFrame* sciFrame = get<0>(sciInfo_[i]);
            UserControlInfo* sciInfo = check_and_cast<UserControlInfo*>(sciFrame->removeControlInfo());
            for (int j=0; j<tbInfo_.size();j++){
                LteAirFrame* tbFrame = get<0>(tbInfo_[j]);
                UserControlInfo* tbInfo = check_and_cast<UserControlInfo*>(tbFrame->removeControlInfo());
                if (sciInfo->getSourceId() == tbInfo->getSourceId()){
                    foundTB = true;
                    tbFrame->setControlInfo(tbInfo);
                    break;
                }
                tbFrame->setControlInfo(tbInfo);
            }
            if (!foundTB){
                missingTbs.push_back(i);
            }
            sciFrame->setControlInfo(sciInfo);
        }

        for (int i=0; i<sciInfo_.size(); i++){
            // Get received SCI and it's corresponding RsrpVector
            LteAirFrame* frame = get<0>(sciInfo_[i]);
            std::vector<double> rsrpVector = get<1>(sciInfo_[i]);
            std::vector<double> rssiVector = get<2>(sciInfo_[i]);
            std::vector<double> sinrVector = get<3>(sciInfo_[i]);
            double attenuation = get<4>(sciInfo_[i]);

            UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(frame->removeControlInfo());

            // decode the selected frame
            decodeAirFrame(frame, lteInfo, rsrpVector, rssiVector, sinrVector, attenuation);

            emit(sciReceived, sciReceived_);
            emit(sciUnsensed, sciUnsensed_);
            emit(sciDecoded, sciDecoded_);
            emit(sciFailedDueToProp, sciFailedDueToProp_);
            emit(sciFailedDueToInterference, sciFailedDueToInterference_);
            emit(sciFailedHalfDuplex, sciFailedHalfDuplex_);
            emit(subchannelReceived, subchannelReceived_);
            emit(subchannelsUsed, subchannelsUsed_);
            emit(oneShot, oneShot_);

            sciReceived_ = 0;
            sciDecoded_ = 0;
            sciFailedHalfDuplex_ = 0;
            sciFailedDueToInterference_ = 0;
            sciFailedDueToProp_ = 0;
            subchannelReceived_ = 0;
            subchannelsUsed_ = 0;
            sciUnsensed_ = 0;
            oneShot_ = 0;

        }
        int countTbs = 0;
        if (tbInfo_.empty()){
            for(countTbs; countTbs<missingTbs.size(); countTbs++){
                emit(txRxDistanceTB, -1);
                emit(tbReceived, -1);
                emit(tbDecoded, -1);
                emit(tbFailedDueToNoSCI, -1);
                emit(tbFailedDueToProp, -1);
                emit(tbFailedDueToInterference, -1);
                emit(tbFailedButSCIReceived, -1);
                emit(tbFailedHalfDuplex, -1);
                emit(periodic, -1);

                emit(tbFailedDueToPropIgnoreSCI ,-1);
                emit(tbFailedDueToInterferenceIgnoreSCI ,-1);
                emit(tbDecodedIgnoreSCI ,-1);
            }
        }
        for (int i=0; i<tbInfo_.size(); i++)
        {
            if(std::find(missingTbs.begin(), missingTbs.end(), i) != missingTbs.end()) {
                // This corresponds to where we are missing a TB, record results as being negative to identify this.
                emit(txRxDistanceTB, -1);
                emit(tbReceived, -1);
                emit(tbDecoded, -1);
                emit(tbFailedDueToNoSCI, -1);
                emit(tbFailedDueToProp, -1);
                emit(tbFailedDueToInterference, -1);
                emit(tbFailedButSCIReceived, -1);
                emit(tbFailedHalfDuplex, -1);
                emit(periodic, -1);

                emit(tbFailedDueToPropIgnoreSCI ,-1);
                emit(tbFailedDueToInterferenceIgnoreSCI ,-1);
                emit(tbDecodedIgnoreSCI ,-1);
            } else {
                LteAirFrame* frame = get<0>(tbInfo_[i]);
                std::vector<double> rsrpVector = get<1>(tbInfo_[i]);
                std::vector<double> rssiVector = get<2>(tbInfo_[i]);
                std::vector<double> sinrVector = get<3>(tbInfo_[i]);
                double attenuation = get<4>(tbInfo_[i]);

                UserControlInfo *lteInfo = check_and_cast<UserControlInfo *>(frame->removeControlInfo());

                // decode the selected frame
                decodeAirFrame(frame, lteInfo, rsrpVector, rssiVector, sinrVector, attenuation);

                emit(tbReceived, tbReceived_);
                emit(tbDecoded, tbDecoded_);
                emit(tbFailedDueToNoSCI, tbFailedDueToNoSCI_);
                emit(tbFailedDueToProp, tbFailedDueToProp_);
                emit(tbFailedDueToInterference, tbFailedDueToInterference_);
                emit(tbFailedButSCIReceived, tbFailedButSCIReceived_);
                emit(tbFailedHalfDuplex, tbFailedHalfDuplex_);
                emit(periodic, int(lteInfo->getPeriodic()));

                emit(tbFailedDueToPropIgnoreSCI ,tbFailedDueToPropIgnoreSCI_);
                emit(tbFailedDueToInterferenceIgnoreSCI ,tbFailedDueToInterferenceIgnoreSCI_);
                emit(tbDecodedIgnoreSCI ,tbDecodedIgnoreSCI_);

                tbReceived_ = 0;
                tbDecoded_ = 0;
                tbFailedDueToNoSCI_ = 0;
                tbFailedButSCIReceived_ = 0;
                tbFailedHalfDuplex_ = 0;
                tbFailedDueToProp_ = 0;
                tbFailedDueToInterference_ = 0;

                tbFailedDueToPropIgnoreSCI_ = 0;
                tbFailedDueToInterferenceIgnoreSCI_ = 0;
                tbDecodedIgnoreSCI_ = 0;
            }
        }
        std::vector<cPacket*>::iterator it;
        for(it=scis_.begin();it!=scis_.end();it++)
        {
            delete(*it);
        }
        sciInfo_.clear();
        tbInfo_.clear();
        scis_.clear();
        delete msg;
        d2dDecodingTimer_ = NULL;
    }
    else if (msg->isName("updateSubframe"))
    {
        transmitting_ = false;
        updateSubframe();
        if (cbrCountDown_ == 0) {
            // Ensures we update CBR every 100ms
            updateCBR();
            cbrCountDown_ = 99;
        } else {
            cbrCountDown_ --;
        }
        delete msg;
    }
    else if (msg->isName("oneShotTimer"))
    {
        // This is the sort of stuff required for sending the SCI before it is sent
        // This information has to be made available to the SCI before sending it down.

        // Most likely at this point we will select the resource and generate MOST of the below

        // Then when transmission of the signal goes you select one of the frees send it up
        // But you translate it first.

        oneShotSensing_ = false;

        int finalSubchannel = oneShotSubchannel_ + 1;

        // Determine the RBs on which we will send our message
        RbMap grantedBlocks;
        int totalGrantedBlocks = 0;
        if (adjacencyPSCCHPSSCH_){
            // Adjacent mode just provide the allocated bands

            for (int i=oneShotSubchannel_;i<finalSubchannel;i++)
            {
                int initialBand = i * subchannelSize_;
                for (Band b = initialBand; b < initialBand + subchannelSize_ ; b++)
                {
                    grantedBlocks[MACRO][b] = 1;
                    ++totalGrantedBlocks;
                    if (totalGrantedBlocks == 2)
                        break;
                }
            }
        } else {
            // Start at subchannel numsubchannels.
            // Remove 1 subchannel from each grant.
            // Account for the two blocks taken for the SCI
            int initialBand;
            if (oneShotSubchannel_ == 0){
                // If first subchannel to use need to make sure to add the number of subchannels for the SCI messages
                initialBand = numSubchannels_ * 2;
            } else {
                initialBand = (numSubchannels_ * 2) + (oneShotSubchannel_ * (subchannelSize_ - 2));
            }
            for (Band b = initialBand; b < initialBand + (subchannelSize_ * 1) ; b++) {
                grantedBlocks[MACRO][b] = 1;
                ++totalGrantedBlocks;
            }
        }

        UserControlInfo* uinfo = new UserControlInfo();
        uinfo->setSourceId(nodeId_);
        uinfo->setDestId(nodeId_);
        uinfo->setFrameType(RESPKT);
        uinfo->setTxNumber(1);
        uinfo->setGrantedBlocks(grantedBlocks);
        uinfo->setTotalGrantedBlocks(totalGrantedBlocks);
        uinfo->setDirection(D2D_MULTI);

        // Subframe Index has to be translated correctly the subchannel is less of an issue.
        int breakOut = 100;
        int selectedSubframe = intuniform(1000 + oneShotT2_, 1100, 1);
        while (oneShotCSRs_.find(selectedSubframe) == oneShotCSRs_.end() & breakOut > 0){
            selectedSubframe = intuniform(1000 + oneShotT2_ + 1, 1100, 1);
            breakOut--;
        }

        auto it = std::begin(oneShotCSRs_[selectedSubframe]);
        // 'advance' the iterator n times
        int n = intuniform(0, oneShotCSRs_[selectedSubframe].size() - 1, 1);
        std::advance(it,n);
        int selectedSubchannel = *it;

        int sensingWindowLength = pStep_ * 10;
        if (sensingWindowSizeOverride_ > 0){
            sensingWindowLength = sensingWindowSizeOverride_;
        }

        simtime_t elapsed_time = NOW.trunc(SIMTIME_MS) - oneShotStartTime_.trunc(SIMTIME_MS);

        int elapsed_ms = elapsed_time.dbl() * 1000;

        selectedSubframe = selectedSubframe - elapsed_ms - sensingWindowLength;

        sendOneShotMessage(uinfo, selectedSubframe - 1, selectedSubchannel); // -1 ensures the correct subframe logged i.e. account for trans time.

        transmitting_ = true;

        for (int i=0; i<numSubchannels_; i++)
        {
            // Mark all the subchannels as not sensed
            sensingWindow_[sensingWindowFront_][i]->setSensed(false);
        }

        std::vector<std::tuple<double, int, int>> orderedCSRs;

        orderedCSRs.push_back(std::make_tuple(-std::numeric_limits<double>::infinity(), selectedSubframe, selectedSubchannel));

        // Send the packet up to the MAC layer where it will choose the CSR and the retransmission if that is specified
        // Need to generate the message that is to be sent to the upper layers.
        SpsCandidateResources* candidateResourcesMessage = new SpsCandidateResources("CSRs");
        candidateResourcesMessage->setCSRs(orderedCSRs);
        send(candidateResourcesMessage, upperGateOut_);

        delete msg;
    }
    else
        LtePhyUe::handleSelfMessage(msg);
}

// TODO: ***reorganize*** method
void LtePhyVUeMode4::handleAirFrame(cMessage* msg)
{
    UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(msg->removeControlInfo());

    connectedNodeId_ = masterId_;
    LteAirFrame* frame = check_and_cast<LteAirFrame*>(msg);
    EV << "LtePhyVUeMode4: received new LteAirFrame with ID " << frame->getId() << " from channel" << endl;
    //Update coordinates of this user
    if (lteInfo->getFrameType() == HANDOVERPKT)
    {
        // check if handover is already in process
        if (handoverTrigger_ != NULL && handoverTrigger_->isScheduled())
        {
            delete lteInfo;
            delete frame;
            return;
        }

        handoverHandler(frame, lteInfo);
        return;
    }

    // HACK: if this is a multicast connection, change the destId of the airframe so that upper layers can handle it
    // All packets in mode 4 are multicast
    lteInfo->setDestId(nodeId_);

    // send H-ARQ feedback up
    if (lteInfo->getFrameType() == HARQPKT || lteInfo->getFrameType() == GRANTPKT || lteInfo->getFrameType() == RACPKT || lteInfo->getFrameType() == D2DMODESWITCHPKT)
    {
        handleControlMsg(frame, lteInfo);
        return;
    }

    // this is a DATA packet

    // if not already started, auto-send a message to signal the presence of data to be decoded
    if (d2dDecodingTimer_ == NULL)
    {
        d2dDecodingTimer_ = new cMessage("d2dDecodingTimer");
        d2dDecodingTimer_->setSchedulingPriority(10);          // last thing to be performed in this TTI
        scheduleAt(NOW, d2dDecodingTimer_);
    }

    Coord myCoord = getCoord();
    // Only store frames which are within 1500m over this the interference caused is negligible.
    if (myCoord.distance(lteInfo->getCoord()) < 1500) {
        // store frame, together with related control info
        frame->setControlInfo(lteInfo);

        // Capture the Airframe for decoding later
        storeAirFrame(frame);
    } else {
        delete lteInfo;
        delete frame;
    }
}

void LtePhyVUeMode4::handleUpperMessage(cMessage* msg)
{

    UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(msg->removeControlInfo());

    LteAirFrame* frame;

    if (lteInfo->getFrameType() == GRANTPKT)
    {
        // Generate CSRs or save the grant use when generating SCI information
        LteMode4SchedulingGrant* grant = check_and_cast<LteMode4SchedulingGrant*>(msg);
        if (grant->getTotalGrantedBlocks() == 0){
            // Generate a vector of CSRs and send it to the MAC layer
            if (randomScheduling_){
                computeRandomCSRs(grant);
            } else {
                computeCSRs(grant);
            }
            delete lteInfo;
            if (!oneShotMechanism_){
                delete grant;
            }
        }
        else
        {
            sciGrant_ = grant;
            lteInfo->setUserTxParams(sciGrant_->getUserTxParams()->dup());
            lteInfo->setGrantedBlocks(sciGrant_->getGrantedBlocks());
            lteInfo->setTotalGrantedBlocks(sciGrant_->getTotalGrantedBlocks());
            lteInfo->setDirection(D2D_MULTI);
            if (oneShotMechanism_){
                lteInfo->setPeriodic(false);
            } else {
                lteInfo->setPeriodic(true);
            }
            availableRBs_ = sendSciMessage(msg, lteInfo);
            for (int i=0; i<numSubchannels_; i++)
            {
                // Mark all the subchannels as not sensed
                sensingWindow_[sensingWindowFront_][i]->setSensed(false);
            }
        }
        return;
    }
    else if (lteInfo->getFrameType() == HARQPKT)
    {
        frame = new LteAirFrame("harqFeedback-grant");
    }

    // if this is a multicast/broadcast connection, send the frame to all neighbors in the hearing range
    // otherwise, send unicast to the destination

    EV << "LtePhyVUeMode4::handleUpperMessage - " << nodeTypeToA(nodeType_) << " with id " << nodeId_
       << " sending message to the air channel. Dest=" << lteInfo->getDestId() << endl;

    // Mark that we are in the process of transmitting a packet therefore when we go to decode messages we can mark as failure due to half duplex
    transmitting_ = true;

    lteInfo->setGrantedBlocks(availableRBs_);

    if (oneShotMechanism_){
        lteInfo->setPeriodic(false);
    }

    frame = prepareAirFrame(msg, lteInfo);

    emit(tbSent, 1);

    if (lteInfo->getDirection() == D2D_MULTI)
        sendBroadcast(frame);
    else
        sendUnicast(frame);
}

void LtePhyVUeMode4::sendOneShotMessage(UserControlInfo* lteInfo, int subframe, int subchannelIndex)
{
    RbMap rbMap = lteInfo->getGrantedBlocks();
    UserControlInfo* SCIInfo = lteInfo->dup();
    LteAirFrame* frame = NULL;

    lteInfo->setPeriodic(false);

    // Store the RBs used for transmission. For interference computation
    UsedRBs info;
    info.time_ = NOW;
    info.rbMap_ = rbMap; // We are only using these in this situation

    usedRbs_.push_back(info);

    std::vector<UsedRBs>::iterator it = usedRbs_.begin();
    while (it != usedRbs_.end())  // purge old allocations
    {
        if (it->time_ < NOW)
            usedRbs_.erase(it++);
        else
            ++it;
    }
    lastActive_ = NOW;

    SCIInfo->setFrameType(RESPKT);
    SCIInfo->setGrantedBlocks(rbMap);
    SCIInfo->setTotalGrantedBlocks(lteInfo->getTotalGrantedBlocks());

    /*
     * Need to prepare the airframe were sending
     * Ensure that it will fit into it's grant
     * if not don't send anything except a break reservation message
     */
    EV << NOW << " LtePhyVUeMode4::handleUpperMessage - message from stack" << endl;

    // create LteAirFrame and encapsulate the received packet
    SidelinkControlInformation* SCI = createOneShotSCIMessage(subframe, subchannelIndex);

    LteAirFrame* sciFrame = prepareAirFrame(SCI, SCIInfo);

    emit(sciSent, 1);
    emit(subchannelSent, subchannelIndex);
    emit(subchannelsUsedToSend, 1);
    sendBroadcast(sciFrame);

    delete lteInfo;
    delete sciGrant_;
}

RbMap LtePhyVUeMode4::sendSciMessage(cMessage* msg, UserControlInfo* lteInfo)
{
    RbMap rbMap = lteInfo->getGrantedBlocks();
    UserControlInfo* SCIInfo = lteInfo->dup();
    LteAirFrame* frame = NULL;

    RbMap sciRbs;
    RbMap allRbs = rbMap;
    if (adjacencyPSCCHPSSCH_)
    {
        // Adjacent mode

        // Setup so SCI gets 2 RBs from the grantedBlocks.
        RbMap::iterator it;
        std::map<Band, unsigned int>::iterator jt;
        //for each Remote unit used to transmit the packet
        int allocatedBlocks = 0;
        for (it = rbMap.begin(); it != rbMap.end(); ++it)
        {
            if (allocatedBlocks == 2)
            {
                break;
            }
            //for each logical band used to transmit the packet
            for (jt = it->second.begin(); jt != it->second.end(); ++jt)
            {
                Band band = jt->first;

                if (allocatedBlocks == 2)
                {
                    // Have all the blocks allocated to the SCI so can move on.
                    break;
                }

                if (jt->second == 0) // this Rb is not allocated
                    continue;
                else
                {
                    // RB is allocated to grant, now give it to the SCI.
                    if (jt->second == 1){
                        jt->second = 0;
                        sciRbs[it->first][band] = 1;
                        ++allocatedBlocks;
                    }
                }
            }
        }
    }
    else
    {
        // Non-Adjacent mode

        // Take 2 rbs from the inital RBs available to nodes.
        int startingRB = sciGrant_->getStartingSubchannel() * 2;

        for (Band b = startingRB; b <= startingRB + 1 ; b++)
        {
            sciRbs[MACRO][b] = 1;
            allRbs[MACRO][b] = 1;
        }
    }

    // Store the RBs used for transmission. For interference computation
    UsedRBs info;
    info.time_ = NOW;
    info.rbMap_ = allRbs;

    usedRbs_.push_back(info);

    std::vector<UsedRBs>::iterator it = usedRbs_.begin();
    while (it != usedRbs_.end())  // purge old allocations
    {
        if (it->time_ < NOW)
            usedRbs_.erase(it++);
        else
            ++it;
    }
    lastActive_ = NOW;

    SCIInfo->setFrameType(SCIPKT);
    SCIInfo->setGrantedBlocks(sciRbs);
    SCIInfo->setTotalGrantedBlocks(sciGrant_->getTotalGrantedBlocks());
    SCIInfo->setGrantStartTime(sciGrant_->getStartTime());

    /*
     * Need to prepare the airframe were sending
     * Ensure that it will fit into it's grant
     * if not don't send anything except a break reservation message
     */
    EV << NOW << " LtePhyVUeMode4::handleUpperMessage - message from stack" << endl;

    // create LteAirFrame and encapsulate the received packet
    SidelinkControlInformation* SCI = createSCIMessage();
    LteAirFrame* sciFrame = prepareAirFrame(SCI, SCIInfo);

    emit(sciSent, 1);
    emit(subchannelSent, sciGrant_->getStartingSubchannel());
    emit(subchannelsUsedToSend, sciGrant_->getNumSubchannels());
    sendBroadcast(sciFrame);

    delete sciGrant_;
    delete lteInfo;

    return (rbMap);
}

void LtePhyVUeMode4::computeRandomCSRs(LteMode4SchedulingGrant* &grant) {
    // Function determines all possible CSRs which fit the packet and returns this to the MAC layer
    // Determine the total number of possible CSRs
    if (grant->getMaximumLatency() >= 100) {
        if (grant->getPeriod() == 50){
            grant->setMaximumLatency(50);
        } else if (grant->getPeriod() == 20){
            grant->setMaximumLatency(20);
        } else {
            grant->setMaximumLatency(100);
        }
    }

    int pRsvpTx = grant->getPeriod();
    int grantLength = grant->getNumSubchannels();
    int cResel = grant->getResourceReselectionCounter();
    int maxLatency = grant->getMaximumLatency();
    std::vector<double> allowedRRIs = grant->getPossibleRRIs();

    int sensingWindowLength = pStep_ * 10;
    if (sensingWindowSizeOverride_ > 0){
        sensingWindowLength = sensingWindowSizeOverride_;
    }

    // Start and end of Selection Window.
    int minSelectionIndex = sensingWindowLength + selectionWindowStartingSubframe_;
    int maxSelectionIndex = sensingWindowLength + maxLatency;

    int totalPossibleCSRs = ((maxSelectionIndex - minSelectionIndex) * numSubchannels_) / grantLength;

    // Create a set of all the possible CSRs
    // Each SubchannelIndex being the starting index of a CSR.
    // Subframe -> {SubchannelIndex, SubchannelIndex}
    std::unordered_map<int, std::set<int>> possibleCSRs;
    for (int i = minSelectionIndex; i <= maxSelectionIndex; i++) {
        std::set<int> subframe;
        for (int j = 0; j <= (numSubchannels_ - grantLength); j += grantLength) {
            subframe.insert(j);
        }
        possibleCSRs[i] = subframe;
    }

    // Simply convert the possible CSRs to the correct format and shuffle them and return 20% of them as normal.
    std::vector<std::tuple<double, int, int>> orderedCSRs;
    std::unordered_map<int, std::set<int>>::iterator it;

    for (it=possibleCSRs.begin(); it!=possibleCSRs.end(); it++)
    {
        int subframe = it->first;
        std::set<int>::iterator jt;
        for (jt=it->second.begin(); jt!=it->second.end(); jt++)
        {
            int sensingSubframeIndex = subframe;
            int initialSubchannelIndex = *jt;
            int finalSubchannelIndex = *jt + grantLength;

            // Subchannel has never been reserved and thus has negative infinite RSSI.
            int transIndex = subframe - sensingWindowLength;
            orderedCSRs.push_back(std::make_tuple(-std::numeric_limits<double>::infinity(), transIndex, initialSubchannelIndex));
        }
    }

    // Send the packet up to the MAC layer where it will choose the CSR and the retransmission if that is specified
    // Need to generate the message that is to be sent to the upper layers.
    SpsCandidateResources* candidateResourcesMessage = new SpsCandidateResources("CSRs");
    candidateResourcesMessage->setCSRs(orderedCSRs);
    send(candidateResourcesMessage, upperGateOut_);
}

void LtePhyVUeMode4::computeCSRs(LteMode4SchedulingGrant* &grant) {
    EV << NOW << " LtePhyVUeMode4::computeCSRs - going through sensing window to compute CSRS..." << endl;
    // Determine the total number of possible CSRs
    if (grant->getMaximumLatency() >= 100) {
        if (grant->getPeriod() == 50){
            grant->setMaximumLatency(50);
        } else if (grant->getPeriod() == 20){
            grant->setMaximumLatency(20);
        } else {
            grant->setMaximumLatency(100);
        }
    }

    EV << NOW
       << " LtePhyVUeMode4::computeCSRs - eliminating CSRS which were not sensed in sensing window and those above the threshold ..."
       << endl;
    int pRsvpTx = grant->getPeriod();
    int grantLength = grant->getNumSubchannels();
    int cResel = grant->getResourceReselectionCounter();
    int maxLatency = grant->getMaximumLatency();
    std::vector<double> allowedRRIs = grant->getPossibleRRIs();

    int sensingWindowLength = pStep_ * 10;
    if (sensingWindowSizeOverride_ > 0){
        sensingWindowLength = sensingWindowSizeOverride_;
    }

    // Start and end of Selection Window.
    int minSelectionIndex = sensingWindowLength + selectionWindowStartingSubframe_;
    int maxSelectionIndex = sensingWindowLength + maxLatency;

    int totalPossibleCSRs = ((maxSelectionIndex - minSelectionIndex) * numSubchannels_) / grantLength;

    // Create a set of all the possible CSRs
    // Each SubchannelIndex being the starting index of a CSR.
    // Subframe -> {SubchannelIndex, SubchannelIndex}
    std::unordered_map<int, std::set<int>> possibleCSRs;
    for (int i = minSelectionIndex; i <= maxSelectionIndex; i++) {
        std::set<int> subframe;
        for (int j = 0; j <= (numSubchannels_ - grantLength); j += grantLength) {
            subframe.insert(j);
        }
        possibleCSRs[i] = subframe;
    }

    // subframes disallowed
    std::vector<int> notSensedSubframes;

    // subchannels disallowed
    std::map < int, std::unordered_map < int, std::vector < int >> > aboveThresholdDisallowedIndices;

    int disallowedCSRs = 0;

    // Number of time the threshold needs to be increased by 3dB to allow for 20% of CSRs to be selected
    int minThresholdIncreasesRequired = 0;

    // If we don't have RRIs greater than 100ms then any subframe which is older than 100ms is not relevant for this part
    // of the selection process, thus we can skip a majority of the sensing window saving time.
    // Only in the case of RRIs of 1000ms will the whole sensing window need to be searched.
    double maxRRI = *std::max_element(allowedRRIs.begin(), allowedRRIs.end());
    int fallBack = 100 * maxRRI;

    if (fallBack >= sensingWindowLength)
    {
        fallBack = sensingWindowLength;
    }

    int minSubCh = sensingWindowLength - fallBack;

    int z = minSubCh;
    while (z <= sensingWindow_.size()) {
        // The use of z is to correspond with the notation in the standard see 3GPP TS 36.213 14.1.1.6

        int pRsvpTxPrime = pStep_ * pRsvpTx / 100;
        int Q = 1;

        // Check if frame is sensed or not.

        int translatedZ = translateIndex(sensingWindowLength - z);

        if (!sensingWindow_[translatedZ][0]->getSensed()) {
            /**
             *  Not sensed calculation
             *
             *  y + j * P'rsvpTx = z + Pstep * k * q
             *
             *  y = subframe of possible CSR
             *  j = {0, 1, ... Cresel-1}
             *  Pstep is the length of frames we have e.g. 100ms long frames
             *  PrsvpTx is the resource reservation interval of transmission e.g. 100
             *  P'rsvpTx = Pstep * PrsvpTx / 100
             *
             *  z = sensing window subframe index
             *  k is all possible RRIs {0.2, 0.5, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10}
             *  q = {1, 2 ... Q}
             *  n' is the current subframe index i.e. 1000.
             *  Q = 1/k if k < 1 AND n' - z <= Pstep * k.
             *
             *  Translated calc (to get all possible disallowed indices)
             *
             *  y = (z + Pstep * k * q) - (j * P'rsvpTx)
             */

            std::vector<double>::iterator k;
            for (k = allowedRRIs.begin(); k != allowedRRIs.end(); k++) {
                // This applies to all allowed RRIs as well.
                // 10 * pStep_ = n'

                if ((*k) < 1 && sensingWindowLength - z < pStep_ * (*k)) {
                    Q = 1 / *k;
                }

                for (int q = 1; q <= Q; q++) {

                    for (int j = 1; j <= cResel; j++) {
                        int disallowedSubframe = (z + (j * pRsvpTxPrime)) - (pStep_ * q * (*k));
                        // Only mark as disallowed if it corresponds with a frame in the selection window
                        if (disallowedSubframe >= minSelectionIndex && disallowedSubframe <= maxSelectionIndex) {
                            notSensedSubframes.push_back(disallowedSubframe);
                            disallowedCSRs += numSubchannels_ / grantLength;
                        }
                    }
                }
            }
        } else {
            int j = 0;
            while (j < numSubchannels_) {
                /**
                 *  RSRP based calculation
                 *
                 *  y + j * P'rsvpRx = tSLm + q * Pstep * PrsvpRx
                 *
                 *  y = subframe of possible CSR
                 *  j = {0, 1, ... Cresel-1} (will use c in the below code for ease of coding)
                 *  Pstep is the length of frames we have e.g. 100ms long frames
                 *  PrsvRx is the resource reservation interval of the received SCI e.g. 100
                 *  P'rsvpTx = Pstep * PrsvpTx / 100
                 *
                 *  tSLm = sensing window subframe index (equivalent of z in the above calc, will use z here for ease of coding)
                 *  q = {1, 2 ... Q}
                 *  n' is the current subframe index i.e. 1000.
                 *  Q = 1/PrsvpTx if PrsvpTx < 1 AND n' - z <= Pstep * PrsvpTx.
                 *
                 *  Translated calc (to get all possible disallowed indices)
                 *
                 *  y = (z + q * pStep_ * PrsvpRx) - (j * P'rsvpTx);
                 */

                // It's possible that a grant might span multiple subchannels, if this is the case then check all for
                // An SCI and record the information for each independently for the later calculation
                std::vector<double> averageRSRPs;
                std::vector<int> priorities;
                std::vector<double> rris;

                bool subchannelReserved = false;
                bool subchannelUsed = false;
                if (j + grantLength > numSubchannels_) {
                    // We cannot fill this grant in this subframe and should move to the next one
                    break;
                }

                int k = j;
                while (k < j + grantLength) {
                    if (sensingWindow_[translatedZ][k]->getReserved()) {
                        // If RRI = 0 then we know the next resource is not reserved.
                        int rri = sensingWindow_[translatedZ][k]->getResourceReservationInterval();
                        if (rri > 0) {
                            subchannelReserved = true;

                            priorities.push_back(sensingWindow_[translatedZ][k]->getPriority());
                            if (rri == 11) {
                                rris.push_back(0.5);
                            } else if (rri == 12) {
                                rris.push_back(0.2);
                            } else {
                                rris.push_back(rri);
                            }
                            double totalRSRPLinear = 0;
                            // Specifically the average should be for the part of the subchannel we will end up using
                            for (int l = j; l < j + grantLength; l++) {
                                if (sensingWindow_[translatedZ][l]->getAverageRSRP() != -std::numeric_limits<double>::infinity()) {
                                    totalRSRPLinear += dBmToLinear(sensingWindow_[translatedZ][l]->getAverageRSRP());
                                }
                            }
                            if (totalRSRPLinear != 0) {
                                subchannelUsed = true;
                                averageRSRPs.push_back(linearToDBm(totalRSRPLinear/grantLength));
                            }
                            k += grantLength;
                        } else if (sensingWindow_[translatedZ][k]->getTimeGapRetrans() > 0){
                            int oneShotSubframe = sensingWindow_[translatedZ][k]->getTimeGapRetrans() + z;
                            if (oneShotSubframe > 1000){
                                // we have a future resource which is reserved and needs to be removed from the possible CSRs
                                int oneShotSubchannel = sensingWindow_[translatedZ][k]->getOneShotSubchannelIndex();
                                possibleCSRs[oneShotSubframe].erase(oneShotSubchannel);
                                if (possibleCSRs[oneShotSubframe].size() == 0){
                                    // If the subframe is now empty then erase it also.
                                    possibleCSRs.erase(oneShotSubframe);
                                }
                            }
                        }
                    }
                     // Increment K to the next subchannel
                     k++;
                }
                if (subchannelReserved && subchannelUsed) {
                    subchannelReserved = false;

                    int highestThreshold = 0;
                    bool thresholdBreach = false;
                    double pRsvpRx;

                    // Get the priorities of both messages
                    int messagePriority = grant->getSpsPriority();
                    for (int l = 0; l < averageRSRPs.size(); l++) {
                        double averageRSRP = averageRSRPs[l];
                        int receivedPriority = priorities[l];
                        double receivedRri = rris[l];

                        // Get the threshold for the corresponding priorities
                        int index = messagePriority * 8 + receivedPriority + 1;
                        int threshold = ThresPSSCHRSRPvector_[index];
                        int thresholdDbm = (-128 + (threshold - 1) * 2);

                        if (averageRSRP > thresholdDbm) {
                            // Must determine the number of increases required to make this a CSR.
                            int thresholdIncreaseFactor = 1;
                            thresholdBreach = true;
                            while (averageRSRP > thresholdDbm) {
                                thresholdDbm = thresholdDbm + (3 * thresholdIncreaseFactor);
                                ++thresholdIncreaseFactor;
                            }
                            if (thresholdIncreaseFactor > highestThreshold) {
                                highestThreshold = thresholdIncreaseFactor;
                                pRsvpRx = receivedRri;
                            }
                        }
                    }

                    if (thresholdBreach) {
                        // This series of subchannels is to be excluded
                        int Q = 1;
                        if (pRsvpRx < 1 && z <= sensingWindowLength - pStep_ * pRsvpRx) {
                            Q = 1 / pRsvpRx;
                        }

                        for (int q = 1; q <= Q; q++) {
                            // j replaced with c in this case as would disrupt above use of j
                            for (int c = 0; c < cResel; c++) {
                                // Based on above calc comment
                                int disallowedIndex = (z + q * pStep_ * pRsvpRx) - (c * pRsvpTxPrime);

                                // Only mark as disallowed if it corresponds with a frame in the selection window
                                if (disallowedIndex >= minSelectionIndex && disallowedIndex <= maxSelectionIndex) {
                                    aboveThresholdDisallowedIndices[highestThreshold][disallowedIndex].push_back(j);
                                    ++disallowedCSRs;
                                }
                            }
                        }
                    }
                }
                // Increase J to search from the next available subchannel
                j += grantLength;
            }
        }
        z++;
    }


    // If too many CSRs are reserved need to reclaim some
    if (disallowedCSRs > totalPossibleCSRs * .8)
    {
        std::map<int, std::unordered_map<int, std::vector<int>>>::const_iterator it;
        for (it = aboveThresholdDisallowedIndices.begin(); it != aboveThresholdDisallowedIndices.end(); it++) {
            int disallowedAtThisIncrease = 0;
            for (int j = minSelectionIndex; j < maxSelectionIndex; j++) {

                std::unordered_map<int, std::vector<int>>::const_iterator got = it->second.find(j);

                if (got != it->second.end()) {
                    disallowedAtThisIncrease += got->second.size();
                }
            }
            // Remove CSRs counted at this increase.
            disallowedCSRs -= disallowedAtThisIncrease;

            // If we go below the 80% disallowed CSRs then mark it, these will have to be added back into possibleCSRs
            if (disallowedCSRs < totalPossibleCSRs * .8) {
                // Found the minimum increases to have enough CSRs.
                minThresholdIncreasesRequired = it->first;
                break;
            }
        }
    }

    // Need to remove all the not sensed subframes first
    for (int i=0; i<notSensedSubframes.size(); i++)
    {
        int subframeIndex = notSensedSubframes[i];
        if ( possibleCSRs.find(subframeIndex) != possibleCSRs.end() ) {
            // Simply erase this element as an option.
            possibleCSRs.erase(subframeIndex);
        }
    }

    // Now need to go through all the threshold breaking CSRs and remove them
    std::map<int, std::unordered_map<int, std::vector<int>>>::const_iterator it;
    for (it = aboveThresholdDisallowedIndices.begin(); it != aboveThresholdDisallowedIndices.end(); it++) {

        // Ignore those that we have to keep due to increased thresholds
        if (it->first > minThresholdIncreasesRequired)
        {
            // Go through each subframe in this threshold
            std::unordered_map<int, std::vector<int>>::const_iterator jt;
            for (jt=it->second.begin(); jt!=it->second.end(); jt++) {

                // Go through each subchannel in this threshold
                std::vector<int>::const_iterator kt;
                for (kt=jt->second.begin(); kt!=jt->second.end(); kt++){
                    // Erase the subchannel
                    possibleCSRs[jt->first].erase(*kt);

                    if (possibleCSRs[jt->first].size() == 0){
                        // If the subframe is now empty then erase it also.
                        possibleCSRs.erase(jt->first);
                    }
                }
            }
        }
    }

    /*
     * Using RSSI pick subchannels with lowest RSSI (Across time) pick 20% lowest.
     * report this to MAC layer.
     */
    std::vector<std::tuple<double, int, int>> optimalCSRs;

    if (rssiFiltering_) {
        optimalCSRs = selectBestRSSIs(possibleCSRs, grant, totalPossibleCSRs);
    } else if (rsrpFiltering_) {
        optimalCSRs = selectBestRSRPs(possibleCSRs, grant, totalPossibleCSRs);
    } else if (oneShotMechanism_) {

        oneShotCSRs_ = possibleCSRs;

        int breakOut = 100;
        int selectedSubframe = intuniform(1002, 1000 + oneShotT2_, 1);
        while (oneShotCSRs_.find(selectedSubframe) == oneShotCSRs_.end() & breakOut > 0){
            selectedSubframe = intuniform(1002, 1000 + oneShotT2_, 1);
            breakOut--;
        }

        auto it = std::begin(oneShotCSRs_[selectedSubframe]);
        // 'advance' the iterator n times
        int n = intuniform(0, oneShotCSRs_[selectedSubframe].size() - 1, 1);
        std::advance(it,n);
        int selectedSubchannel = *it;

        simtime_t selectedStartTime = (simTime().trunc(SIMTIME_MS) + SimTime(selectedSubframe - sensingWindowLength -1, SIMTIME_MS)).trunc(SIMTIME_MS);

        oneShotSubchannel_ = selectedSubchannel;

        sciGrant_ = grant;

        sendOneShotSignal_ = new cMessage("oneShotTimer");
        sendOneShotSignal_->setSchedulingPriority(1);        // Generate the subframe at start of next TTI
        scheduleAt(selectedStartTime, sendOneShotSignal_);

        oneShotSignalTime_ = selectedStartTime;

        oneShotStartTime_ = NOW.trunc(SIMTIME_MS);

        oneShotSensing_ = true;

        return;
    } else {
        // Simply convert the possible CSRs to the correct format and shuffle them and return 20% of them as normal.
        std::vector<std::tuple<double, int, int>> orderedCSRs;
        std::unordered_map<int, std::set<int>>::iterator it;

        for (it=possibleCSRs.begin(); it!=possibleCSRs.end(); it++)
        {
            int subframe = it->first;
            std::set<int>::iterator jt;
            for (jt=it->second.begin(); jt!=it->second.end(); jt++)
            {
                int sensingSubframeIndex = subframe;
                int initialSubchannelIndex = *jt;
                int finalSubchannelIndex = *jt + grantLength;

                // Subchannel has never been reserved and thus has negative infinite RSSI.
                int transIndex = subframe - sensingWindowLength;
                orderedCSRs.push_back(std::make_tuple(-std::numeric_limits<double>::infinity(), transIndex, initialSubchannelIndex));
            }
        }

        optimalCSRs = orderedCSRs;
    }

    // Send the packet up to the MAC layer where it will choose the CSR and the retransmission if that is specified
    // Need to generate the message that is to be sent to the upper layers.
    SpsCandidateResources* candidateResourcesMessage = new SpsCandidateResources("CSRs");
    candidateResourcesMessage->setCSRs(optimalCSRs);
    send(candidateResourcesMessage, upperGateOut_);
}

std::vector<std::tuple<double, int, int>> LtePhyVUeMode4::selectBestRSSIs(std::unordered_map<int, std::set<int>> possibleCSRs, LteMode4SchedulingGrant* &grant, int totalPossibleCSRs)
{
    EV << NOW << " LtePhyVUeMode4::selectBestRSSIs - Selecting best CSRs from possible CSRs..." << endl;
    int decrease = pStep_;
    if (grant->getPeriod() < 100)
    {
        // Same as pPrimeRsvpTx from other parts of the function
        decrease = (pStep_ * grant->getPeriod())/100;
    }

    int maxLatency = grant->getMaximumLatency();

    int sensingWindowLength = pStep_ * 10;
    if (sensingWindowSizeOverride_ > 0){
        sensingWindowLength = sensingWindowSizeOverride_;
    }

    // Start and end of Selection Window.
    int minSelectionIndex = sensingWindowLength + selectionWindowStartingSubframe_;
    int maxSelectionIndex = sensingWindowLength + maxLatency;

    unsigned int grantLength = grant->getNumSubchannels();

    // This will be avgRSSI -> (subframeIndex, subchannelIndex)
    std::vector<std::tuple<double, int, int>> orderedCSRs;
    std::unordered_map<int, std::set<int>>::iterator it;

    for (it=possibleCSRs.begin(); it!=possibleCSRs.end(); it++)
    {
        int subframe = it->first;
        std::set<int>::iterator jt;
        for (jt=it->second.begin(); jt!=it->second.end(); jt++)
        {
            int sensingSubframeIndex = subframe;
            int initialSubchannelIndex = *jt;
            int finalSubchannelIndex = *jt + grantLength;

            while (sensingSubframeIndex > sensingWindowLength){
                // decrease the subframe index until we are within the sensing window.
                sensingSubframeIndex -= decrease;
            }

            double totalRSSI = 0;
            int numSubchannels = 0;
            while (sensingSubframeIndex > 0)
            {
                int translatedSubframeIndex = translateIndex(sensingWindowLength - sensingSubframeIndex);
                for (int subchannelCounter = initialSubchannelIndex; subchannelCounter < finalSubchannelIndex; subchannelCounter++)
                {
                    if (sensingWindow_[translatedSubframeIndex][subchannelCounter]->getSensed())
                    {
                        double averageRSSI = dBmToLinear(sensingWindow_[translatedSubframeIndex][subchannelCounter]->getAverageRSSI());
                        if (averageRSSI != -std::numeric_limits<double>::infinity()){
                            totalRSSI += averageRSSI;
                            ++numSubchannels;
                        }
                    }
                    else
                    {
                        break;
                    }
                }
                sensingSubframeIndex -= decrease;
            }
            double averageRSSI = 0;
            if (numSubchannels != 0)
            {
                // Can be the case when the sensing window is not full that we don't find the historic CSRs
                averageRSSI = totalRSSI / numSubchannels;
                int transIndex = subframe - sensingWindowLength;
                orderedCSRs.push_back(std::make_tuple(linearToDBm(averageRSSI), transIndex, initialSubchannelIndex));
            }
            else {
                // Subchannel has never been reserved and thus has negative infinite RSSI.
                int transIndex = subframe - sensingWindowLength;
                orderedCSRs.push_back(std::make_tuple(-std::numeric_limits<double>::infinity(), transIndex, initialSubchannelIndex));
            }
        }
    }

    // Shuffle ensures that the subframes and subchannels appear in a random order, making the selections more balanced
    // throughout the selection window.
    std::random_shuffle (orderedCSRs.begin(), orderedCSRs.end());

    std::sort(begin(orderedCSRs), end(orderedCSRs), [](const std::tuple<double, int, int> &t1, const std::tuple<double, int, int> &t2) {
        return get<0>(t1) < get<0>(t2); // or use a custom compare function
    });

    int minSize = std::round(totalPossibleCSRs * .2);
    orderedCSRs.resize(minSize);

    return orderedCSRs;
}

std::vector<std::tuple<double, int, int>> LtePhyVUeMode4::selectBestRSRPs(std::unordered_map<int, std::set<int>> possibleCSRs, LteMode4SchedulingGrant* &grant, int totalPossibleCSRs)
{
    EV << NOW << " LtePhyVUeMode4::selectBestRSSIs - Selecting best CSRs from possible CSRs..." << endl;
    int decrease = pStep_;
    if (grant->getPeriod() < 100)
    {
        // Same as pPrimeRsvpTx from other parts of the function
        decrease = (pStep_ * grant->getPeriod())/100;
    }

    int maxLatency = grant->getMaximumLatency();

    int sensingWindowLength = pStep_ * 10;
    if (sensingWindowSizeOverride_ > 0){
        sensingWindowLength = sensingWindowSizeOverride_;
    }

    // Start and end of Selection Window.
    int minSelectionIndex = sensingWindowLength + selectionWindowStartingSubframe_;
    int maxSelectionIndex = sensingWindowLength + maxLatency;

    unsigned int grantLength = grant->getNumSubchannels();

    // This will be avgRSSI -> (subframeIndex, subchannelIndex)
    std::vector<std::tuple<double, int, int>> orderedCSRs;
    std::unordered_map<int, std::set<int>>::iterator it;

    for (it=possibleCSRs.begin(); it!=possibleCSRs.end(); it++)
    {
        int subframe = it->first;
        std::set<int>::iterator jt;
        for (jt=it->second.begin(); jt!=it->second.end(); jt++)
        {
            int sensingSubframeIndex = subframe;
            int initialSubchannelIndex = *jt;
            int finalSubchannelIndex = *jt + grantLength;

            while (sensingSubframeIndex > sensingWindowLength){
                // decrease the subframe index until we are within the sensing window.
                sensingSubframeIndex -= decrease;
            }

            double totalRSRP = 0;
            int numSubchannels = 0;
            while (sensingSubframeIndex > 0)
            {
                int translatedSubframeIndex = translateIndex(sensingWindowLength - sensingSubframeIndex);
                for (int subchannelCounter = initialSubchannelIndex; subchannelCounter < finalSubchannelIndex; subchannelCounter++)
                {
                    if (sensingWindow_[translatedSubframeIndex][subchannelCounter]->getSensed())
                    {
                        double averageRSRP = dBmToLinear(sensingWindow_[translatedSubframeIndex][subchannelCounter]->getAverageRSRP());
                        if (averageRSRP != -std::numeric_limits<double>::infinity()){
                            totalRSRP += averageRSRP;
                            ++numSubchannels;
                        }
                    }
                    else
                    {
                        break;
                    }
                }
                sensingSubframeIndex -= decrease;
            }
            double averageRSRP = 0;
            if (numSubchannels != 0)
            {
                // Can be the case when the sensing window is not full that we don't find the historic CSRs
                averageRSRP = totalRSRP / numSubchannels;
                int transIndex = subframe - sensingWindowLength;
                orderedCSRs.push_back(std::make_tuple(linearToDBm(averageRSRP), transIndex, initialSubchannelIndex));
            }
            else {
                // Subchannel has never been reserved and thus has negative infinite RSRP.
                int transIndex = subframe - sensingWindowLength;
                orderedCSRs.push_back(std::make_tuple(-std::numeric_limits<double>::infinity(), transIndex, initialSubchannelIndex));
            }
        }
    }

    // Shuffle ensures that the subframes and subchannels appear in a random order, making the selections more balanced
    // throughout the selection window.
    std::random_shuffle (orderedCSRs.begin(), orderedCSRs.end());

    std::sort(begin(orderedCSRs), end(orderedCSRs), [](const std::tuple<double, int, int> &t1, const std::tuple<double, int, int> &t2) {
        return get<0>(t1) < get<0>(t2); // or use a custom compare function
    });

    int minSize = std::round(totalPossibleCSRs * .2);
    orderedCSRs.resize(minSize);

    return orderedCSRs;
}

SidelinkControlInformation* LtePhyVUeMode4::createSCIMessage()
{
    EV << NOW << " LtePhyVUeMode4::createSCIMessage - Start creating SCI..." << endl;

    SidelinkControlInformation* sci = new SidelinkControlInformation("SCI Message");

    /*
     * Priority (based on upper layer)
     * 0-7
     * Mapping unknown, so possibly just take the priority max and min and map to 0-7
     * This needs to be integrated from the application layer.
     * Will take this from the scheduling grant.
     */
    sci->setPriority(sciGrant_->getSpsPriority());

    /* Resource Interval
     *
     * 0 -> 16
     * 0 = not reserved
     * 1 = 100ms (1) RRI [Default]
     * 2 = 200ms (2) RRI
     * ...
     * 10 = 1000ms (10) RRI
     * 11 = 50ms (0.5) RRI
     * 12 = 20ms (0.2) RRI
     * 13 - 15 = Reserved
     *
     */
    if (sciGrant_->getExpiration() != 0)
    {
        if (sciGrant_->getPeriod() == 50){
            sci->setResourceReservationInterval(11);
        } else if (sciGrant_->getPeriod() == 20) {
            sci->setResourceReservationInterval(12);
        } else {
            sci->setResourceReservationInterval(sciGrant_->getPeriod() / 100);
        }
    }
    else
    {
        sci->setResourceReservationInterval(0);
    }
    /* frequency Resource Location
     * Based on another parameter RIV
     * but size is
     * Log2(Nsubchannel(Nsubchannel+1)/2) (rounded up)
     * 0 - 8 bits
     * 0 - 256 different values
     *
     * Based on TS36.213 14.1.1.4C
     *     if SubchannelLength -1 < numSubchannels/2
     *         RIV = numSubchannels(SubchannelLength-1) + subchannelIndex
     *     else
     *         RIV = numSubchannels(numSubchannels-SubchannelLength+1) + (numSubchannels-1-subchannelIndex)
     */
    unsigned int riv;
    //
    if (sciGrant_->getNumSubchannels() -1 <= (numSubchannels_/2))
    {
        // RIV calculation for less than half+1
        riv = ((numSubchannels_ * (sciGrant_->getNumSubchannels() - 1)) + sciGrant_->getStartingSubchannel());
    }
    else
    {
        // RIV calculation for more than half size
        riv = ((numSubchannels_ * (numSubchannels_ - sciGrant_->getNumSubchannels() + 1)) + (numSubchannels_ - 1 - sciGrant_->getStartingSubchannel()));
    }

    sci->setFrequencyResourceLocation(riv);

    /* TimeGapRetrans
     * 1 - 15
     * ms from init to retrans
     */
    sci->setTimeGapRetrans(sciGrant_->getTimeGapTransRetrans());


    /* mcs
     * 5 bits
     * 26 combos
     * Technically the MAC layer determines the MCS that it wants the message sent with and as such it will be in the packet
     */
    sci->setMcs(sciGrant_->getMcs());

    /* retransmissionIndex
     * 0 / 1
     * if 0 retrans is in future/this is the first transmission
     * if 1 retrans is in the past/this is the retrans
     */
    if (sciGrant_->getRetransmission())
    {
        sci->setRetransmissionIndex(1);
    }
    else
    {
        sci->setRetransmissionIndex(0);
    }

    /* Filler up to 32 bits
     * Can just set the length to 32 bits and ignore the issue of adding filler
     */
    sci->setBitLength(32);

    return sci;
}

SidelinkControlInformation* LtePhyVUeMode4::createOneShotSCIMessage(int subframe, int subchannelIndex)
{
    EV << NOW << " LtePhyVUeMode4::createOneShotSCIMessage - Start creating SCI..." << endl;

    SidelinkControlInformation* sci = new SidelinkControlInformation("SCI Message");

    /*
     * Priority (based on upper layer)
     * 0-7
     * Mapping unknown, so possibly just take the priority max and min and map to 0-7
     * This needs to be integrated from the application layer.
     * Will take this from the scheduling grant.
     */
    sci->setPriority(sciGrant_->getSpsPriority());

    /* Resource Interval
     *
     * 0 -> 16
     * 0 = not reserved
     * 1 = 100ms (1) RRI [Default]
     * 2 = 200ms (2) RRI
     * ...
     * 10 = 1000ms (10) RRI
     * 11 = 50ms (0.5) RRI
     * 12 = 20ms (0.2) RRI
     * 13 - 15 = Reserved
     *
     */

    // This is always 0 for one shot as they don't maintain the grant at all.
    sci->setResourceReservationInterval(0);

    /* frequency Resource Location
     * Based on another parameter RIV
     * but size is
     * Log2(Nsubchannel(Nsubchannel+1)/2) (rounded up)
     * 0 - 8 bits
     * 0 - 256 different values
     *
     * Based on TS36.213 14.1.1.4C
     *     if SubchannelLength -1 < numSubchannels/2
     *         RIV = numSubchannels(SubchannelLength-1) + subchannelIndex
     *     else
     *         RIV = numSubchannels(numSubchannels-SubchannelLength+1) + (numSubchannels-1-subchannelIndex)
     */
    unsigned int riv;
    //
    if (sciGrant_->getNumSubchannels() -1 <= (numSubchannels_/2))
    {
        // RIV calculation for less than half+1
        riv = ((numSubchannels_ * (sciGrant_->getNumSubchannels() - 1)) + oneShotSubchannel_);
    }
    else
    {
        // RIV calculation for more than half size
        riv = ((numSubchannels_ * (numSubchannels_ - sciGrant_->getNumSubchannels() + 1)) + (numSubchannels_ - 1 - sciGrant_->getStartingSubchannel()));
    }

    sci->setFrequencyResourceLocation(riv);

    /* TimeGapRetrans
     * 1 - 15
     * ms from init to retrans
     */

    // So this is where we need to add the proper time gap between now and the future transmission this is where most logic will be
    sci->setTimeGapRetrans(subframe);


    /* mcs
     * 5 bits
     * 26 combos
     * Technically the MAC layer determines the MCS that it wants the message sent with and as such it will be in the packet
     */
    sci->setMcs(sciGrant_->getMcs());

    /* retransmissionIndex
     * 0 / 1
     * if 0 retrans is in future/this is the first transmission
     * if 1 retrans is in the past/this is the retrans
     */
    sci->setRetransmissionIndex(0);

    // Put in the number of the selected subchannel to send on.
    sci->setOneShotLocation(subchannelIndex);

    /* Filler up to 32 bits
     * Can just set the length to 32 bits and ignore the issue of adding filler
     */
    sci->setBitLength(32);

    return sci;
}

LteAirFrame* LtePhyVUeMode4::prepareAirFrame(cMessage* msg, UserControlInfo* lteInfo){
    // Helper function to prepare airframe for sending.
    LteAirFrame* frame = new LteAirFrame("airframe");

    frame->encapsulate(check_and_cast<cPacket*>(msg));
    frame->setSchedulingPriority(airFramePriority_);
    frame->setDuration(TTI);

    lteInfo->setCoord(getRadioPosition());

    lteInfo->setTxPower(txPower_);
    lteInfo->setD2dTxPower(d2dTxPower_);
    frame->setControlInfo(lteInfo);

    return frame;
}

void LtePhyVUeMode4::storeAirFrame(LteAirFrame* newFrame)
{
    // implements the capture effect
    // store the frame received from the nearest transmitter
    UserControlInfo* newInfo = check_and_cast<UserControlInfo*>(newFrame->getControlInfo());
    Coord myCoord = getCoord();

    std::tuple<std::vector<double>, double> rsrpAttenuation = channelModel_->getRSRP_D2D(newFrame, newInfo, nodeId_, myCoord);
    std::vector<double> rsrpVector = get<0>(rsrpAttenuation);
    double attenuation = get<1>(rsrpAttenuation);

    // Seems we don't really actually need the enbId, I have set it to 0 as it is referenced but never used for calc
    std::tuple<std::vector<double>, std::vector<double>> rssiSinrVectors = channelModel_->getRSSI_SINR(newFrame, newInfo, nodeId_, myCoord, 0, rsrpVector);

    std::vector<double> rssiVector = get<0>(rssiSinrVectors);
    std::vector<double> sinrVector = get<1>(rssiSinrVectors);

    int countAssignedRbs = 0;
    double avgSinr = 0.0;
    RbMap grantedBlocks = newInfo->getGrantedBlocks();

    RbMap::iterator it;
    std::map<Band, unsigned int>::iterator jt;
    //for each Remote unit used to transmit the packet
    for (it = grantedBlocks.begin(); it != grantedBlocks.end(); ++it) {
        //for each logical band used to transmit the packet
        for (jt = it->second.begin(); jt != it->second.end(); ++jt) {
            //this Rb is not allocated
            if (jt->second == 0)
                continue;
            avgSinr += sinrVector[jt->first];
            countAssignedRbs++;
        }
    }

    avgSinr = avgSinr / countAssignedRbs;

    // Need to be able to figure out which subchannel is associated to the Rbs in this case
    if (newInfo->getFrameType() == SCIPKT || newInfo->getFrameType() == RESPKT){
        sciInfo_.push_back(std::make_tuple(newFrame, rsrpVector, rssiVector, sinrVector, attenuation, avgSinr));
    }  else{
        tbInfo_.push_back(std::make_tuple(newFrame, rsrpVector, rssiVector, sinrVector, attenuation, avgSinr));
    }
}

void LtePhyVUeMode4::decodeAirFrame(LteAirFrame* frame, UserControlInfo* lteInfo, std::vector<double> &rsrpVector, std::vector<double> &rssiVector, std::vector<double> &sinrVector, double &attenuation)
{
    EV << NOW << " LtePhyVUeMode4::decodeAirFrame - Start decoding..." << endl;

    // apply decider to received packet
    bool interference_result = false;
    bool prop_result = false;

    cPacket* pkt = frame->decapsulate();

    if(lteInfo->getFrameType() == RESPKT)
    {
        double pkt_dist = getCoord().distance(lteInfo->getCoord());
        emit(txRxDistanceSCI, pkt_dist);
        oneShot_ = 1;

        SidelinkControlInformation *sci = check_and_cast<SidelinkControlInformation *>(pkt);
        std::tuple<int, int> indexAndLength = decodeRivValue(sci, lteInfo);
        int subchannelIndex = std::get<0>(indexAndLength);
        int lengthInSubchannels = std::get<1>(indexAndLength);

        subchannelReceived_ = subchannelIndex;
        subchannelsUsed_ = lengthInSubchannels;
        emit(senderID, lteInfo->getSourceId());

        if (!transmitting_)
        {

            sciReceived_ += 1;

            bool notSensed = false;
            double erfParam = (lteInfo->getD2dTxPower() - attenuation - -90.5) / (3 * sqrt(2));
            double erfValue = erf(erfParam);
            double packetSensingRatio = 0.5 * (1 + erfValue);
            double er = dblrand(1);

            if (er >= packetSensingRatio){
                // Packet was not sensed so mark as such and delete it.
                lteInfo->setDeciderResult(false);
                sciUnsensed_ += 1;
            } else {
                std::tuple<bool, bool> res = channelModel_->error_Mode4(frame, lteInfo, rsrpVector, sinrVector, 0);
                prop_result = get<0>(res);
                interference_result = get<1>(res);

                RbMap::iterator mt;
                std::map<Band, unsigned int>::iterator nt;
                RbMap usedRbs = lteInfo->getGrantedBlocks();
                std::vector < Subchannel * > currentSubframe = sensingWindow_[sensingWindowFront_];
                Subchannel *currentSubchannel = currentSubframe[subchannelIndex];
                std::vector<Band>::iterator lt;
                std::vector <Band> allocatedBands = currentSubchannel->getOccupiedBands();
                for (lt = allocatedBands.begin(); lt != allocatedBands.end(); lt++) {
                    // Record RSRP and RSSI for this band depending if it was used or not
                    bool used = false;

                    //for each Remote unit used to transmit the packet
                    for (mt = usedRbs.begin(); mt != usedRbs.end(); ++mt) {
                        //for each logical band used to transmit the packet
                        for (nt = mt->second.begin(); nt != mt->second.end(); ++nt) {
                            if (nt->first == *lt) {
                                currentSubchannel->addRsrpValue(rsrpVector[(*lt)], (*lt));
                                currentSubchannel->addRssiValue(rssiVector[(*lt)], (*lt));
                                used = true;
                                break;
                            }
                        }
                    }
                }

                // Need to ensure that we haven't previously decoded a higher SINR packet.
                if (interference_result & !sensingWindow_[sensingWindowFront_][subchannelIndex]->getReserved()) {
                    for (int i = subchannelIndex; i < subchannelIndex + lengthInSubchannels; i++) {
                        Subchannel *currentSubchannel = currentSubframe[i];
                        // Record the SCI info in the subchannel.
                        currentSubchannel->setPriority(sci->getPriority());
                        currentSubchannel->setResourceReservationInterval(sci->getResourceReservationInterval());
                        currentSubchannel->setFrequencyResourceLocation(sci->getFrequencyResourceLocation());
                        currentSubchannel->setTimeGapRetrans(sci->getTimeGapRetrans());
                        currentSubchannel->setMcs(sci->getMcs());
                        currentSubchannel->setRetransmissionIndex(sci->getRetransmissionIndex());
                        currentSubchannel->setSciSubchannelIndex(subchannelIndex);
                        currentSubchannel->setSciLength(lengthInSubchannels);
                        currentSubchannel->setOneShotSubchannelIndex(sci->getOneShotLocation());
                        currentSubchannel->setReserved(true);

                        if (oneShotSensing_) {
                            if (sci->getTimeGapRetrans() > 0) {
                                // We need to get this and look at subchannels in T2->T3 which overlap
                                int subframe_in_future =  sci->getTimeGapRetrans();

                                double ttis_in_future = sci->getTimeGapRetrans();
                                ttis_in_future = (ttis_in_future-1) * TTI;
                                // setFrequencyResourceLocation will determine the subchannel which will be used for
                                // future transmissions.
                                int subchannel_in_future = sci->getOneShotLocation();

                                simtime_t reservationTime = (NOW + ttis_in_future).trunc(SIMTIME_MS);

                                if (reservationTime == oneShotSignalTime_ &&
                                    subchannel_in_future == oneShotSubchannel_) {
                                    // Need to defer or decide on having the collision if this is the last subchannel and subframe in t1t2 then send
                                    // otherwise defer
                                    int breakOut = 100;

                                    simtime_t elapsed_time = NOW.trunc(SIMTIME_MS) - oneShotStartTime_.trunc(SIMTIME_MS);

                                    int elapsed_ms = elapsed_time.dbl() * 1000;

                                    elapsed_ms = elapsed_ms + 1000;

                                    if (elapsed_ms < 1000 + oneShotT2_) {
                                        // Can defer this transmission

                                        int selectedSubframe = intuniform(elapsed_ms + 2, 1000 + oneShotT2_, 1);
                                        while (oneShotCSRs_.find(selectedSubframe) == oneShotCSRs_.end() &
                                               breakOut > 0) {
                                            selectedSubframe = intuniform(elapsed_ms + 2, 1000 + oneShotT2_, 1);
                                            breakOut--;
                                        }

                                        auto it = std::begin(oneShotCSRs_[selectedSubframe]);
                                        // 'advance' the iterator n times
                                        int n = intuniform(0, oneShotCSRs_[selectedSubframe].size() - 1, 1);
                                        std::advance(it, n);
                                        int selectedSubchannel = *it;

                                        int sensingWindowLength = pStep_ * 10;
                                        if (sensingWindowSizeOverride_ > 0) {
                                            sensingWindowLength = sensingWindowSizeOverride_;
                                        }

                                        simtime_t selectedStartTime = (oneShotStartTime_.trunc(SIMTIME_MS) + SimTime(selectedSubframe - sensingWindowLength - 1, SIMTIME_MS)).trunc(SIMTIME_MS);

                                        oneShotSubchannel_ = selectedSubchannel;

                                        cancelEvent(sendOneShotSignal_); // Cancel the previous signal

                                        sendOneShotSignal_ = new cMessage("oneShotTimer");
                                        sendOneShotSignal_->setSchedulingPriority(1);        // Generate the subframe at start of next TTI
                                        scheduleAt(selectedStartTime, sendOneShotSignal_);

                                        oneShotSignalTime_ = selectedStartTime;

                                        oneShotSensing_ = true;
                                    }
                                } else {

                                    simtime_t elapsed_time = NOW.trunc(SIMTIME_MS) - oneShotStartTime_.trunc(SIMTIME_MS);

                                    int elapsed_ms = elapsed_time.dbl() * 1000;

                                    int subframe = elapsed_ms + subframe_in_future + 1000;

                                    if (subframe < 1100) {
                                        // Erase the subchannel
                                        oneShotCSRs_[subframe].erase(subchannel_in_future);

                                        if (oneShotCSRs_[subframe].size() == 0) {
                                            // If the subframe is now empty then erase it also.
                                            oneShotCSRs_.erase(subframe);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    lteInfo->setDeciderResult(true);
                    sciDecoded_ += 1;
                } else if (!prop_result) {
                    lteInfo->setDeciderResult(false);
                    sciFailedDueToProp_ += 1;
                } else {
                    lteInfo->setDeciderResult(false);
                    interference_result = false;
                    sciFailedDueToInterference_ += 1;
                }
            }
            // No longer need the packet can delete it.
            delete lteInfo;
            delete pkt;
        }
        else
        {
            sciFailedHalfDuplex_ += 1;
            delete lteInfo;
            delete pkt;
        }
        delete frame;
    }
    else if(lteInfo->getFrameType() == SCIPKT)
    {
        double pkt_dist = getCoord().distance(lteInfo->getCoord());
        emit(txRxDistanceSCI, pkt_dist);

        SidelinkControlInformation *sci = check_and_cast<SidelinkControlInformation *>(pkt);
        std::tuple<int, int> indexAndLength = decodeRivValue(sci, lteInfo);
        int subchannelIndex = std::get<0>(indexAndLength);
        int lengthInSubchannels = std::get<1>(indexAndLength);

        subchannelReceived_ = subchannelIndex;
        subchannelsUsed_ = lengthInSubchannels;
        emit(senderID, lteInfo->getSourceId());

        if (!transmitting_)
        {

            sciReceived_ += 1;
            oneShot_ = 0;

            bool notSensed = false;
            double erfParam = (lteInfo->getD2dTxPower() - attenuation - -90.5) / (3 * sqrt(2));
            double erfValue = erf(erfParam);
            double packetSensingRatio = 0.5 * (1 + erfValue);
            double er = dblrand(1);

            if (er >= packetSensingRatio){
                // Packet was not sensed so mark as such and delete it.
                lteInfo->setDeciderResult(false);
                sciUnsensed_ += 1;
            } else {
                std::tuple<bool, bool> res = channelModel_->error_Mode4(frame, lteInfo, rsrpVector, sinrVector, 0);
                prop_result = get<0>(res);
                interference_result = get<1>(res);

                subchannelReceived_ = subchannelIndex;
                subchannelsUsed_ = lengthInSubchannels;
                emit(senderID, lteInfo->getSourceId());

                RbMap::iterator mt;
                std::map<Band, unsigned int>::iterator nt;
                RbMap usedRbs = lteInfo->getGrantedBlocks();
                std::vector < Subchannel * > currentSubframe = sensingWindow_[sensingWindowFront_];
                Subchannel *currentSubchannel = currentSubframe[subchannelIndex];
                std::vector<Band>::iterator lt;
                std::vector <Band> allocatedBands = currentSubchannel->getOccupiedBands();
                for (lt = allocatedBands.begin(); lt != allocatedBands.end(); lt++) {
                    // Record RSRP and RSSI for this band depending if it was used or not
                    bool used = false;

                    //for each Remote unit used to transmit the packet
                    for (mt = usedRbs.begin(); mt != usedRbs.end(); ++mt) {
                        //for each logical band used to transmit the packet
                        for (nt = mt->second.begin(); nt != mt->second.end(); ++nt) {
                            if (nt->first == *lt) {
                                currentSubchannel->addRsrpValue(rsrpVector[(*lt)], (*lt));
                                currentSubchannel->addRssiValue(rssiVector[(*lt)], (*lt));
                                used = true;
                                break;
                            }
                        }
                    }
                }

                // Need to ensure that we haven't previously decoded a higher SINR packet.
                if (interference_result & !sensingWindow_[sensingWindowFront_][subchannelIndex]->getReserved()) {
                    for (int i = subchannelIndex; i < subchannelIndex + lengthInSubchannels; i++) {
                        Subchannel *currentSubchannel = currentSubframe[i];
                        // Record the SCI info in the subchannel.
                        currentSubchannel->setPriority(sci->getPriority());
                        currentSubchannel->setResourceReservationInterval(sci->getResourceReservationInterval());
                        currentSubchannel->setFrequencyResourceLocation(sci->getFrequencyResourceLocation());
                        currentSubchannel->setTimeGapRetrans(sci->getTimeGapRetrans());
                        currentSubchannel->setMcs(sci->getMcs());
                        currentSubchannel->setRetransmissionIndex(sci->getRetransmissionIndex());
                        currentSubchannel->setSciSubchannelIndex(subchannelIndex);
                        currentSubchannel->setSciLength(lengthInSubchannels);
                        currentSubchannel->setReserved(true);
                    }
                    lteInfo->setDeciderResult(true);
                    sciDecoded_ += 1;
                } else if (!prop_result) {
                    lteInfo->setDeciderResult(false);
                    sciFailedDueToProp_ += 1;
                } else {
                    lteInfo->setDeciderResult(false);
                    // Just ensure the result is recorded as false
                    interference_result = false;
                    sciFailedDueToInterference_ += 1;
                }
            }
            pkt->setControlInfo(lteInfo);
            scis_.push_back(pkt);
        }
        else
        {
            sciFailedHalfDuplex_ += 1;
            delete lteInfo;
            delete pkt;
        }
        delete frame;
    }
    else
    {
        double pkt_dist = getCoord().distance(lteInfo->getCoord());
        emit(txRxDistanceTB, pkt_dist);

        if(!transmitting_){

            tbReceived_ += 1;

            // Have a TB want to make sure we have the SCI for it.
            bool foundCorrespondingSci = false;
            bool sciDecodedSuccessfully = false;
            SidelinkControlInformation *correspondingSCI;
            UserControlInfo *sciInfo;
            std::vector<cPacket *>::iterator it;
            for (it = scis_.begin(); it != scis_.end(); it++) {
                sciInfo = check_and_cast<UserControlInfo*>((*it)->removeControlInfo());
                // if the SCI and TB have same source then we have the right SCI
                if (sciInfo->getSourceId() == lteInfo->getSourceId()) {
                    //Successfully received the SCI
                    foundCorrespondingSci = true;

                    correspondingSCI = check_and_cast<SidelinkControlInformation*>(*it);

                    if (sciInfo->getDeciderResult()) {
                        sciDecodedSuccessfully = true;
                    }

                    if (lteInfo->getDirection() == D2D_MULTI) {
                        std::tuple<bool, bool> res = channelModel_->error_Mode4(frame, lteInfo, rsrpVector, sinrVector, correspondingSCI->getMcs());
                        prop_result = get<0>(res);
                        interference_result = get<1>(res);
                    }

                    // Remove the SCI
                    scis_.erase(it);
                    break;
                } else {
                    (*it)->setControlInfo(sciInfo);
                }
            }
            if (!foundCorrespondingSci || !sciDecodedSuccessfully) {
                tbFailedDueToNoSCI_ += 1;
                if (!prop_result) {
                    tbFailedDueToPropIgnoreSCI_ += 1;
                } else if (!interference_result) {
                    tbFailedDueToInterferenceIgnoreSCI_ += 1;
                } else {
                    tbDecodedIgnoreSCI_ += 1;
                }

            } else if (!prop_result) {
                tbFailedButSCIReceived_ += 1;
                tbFailedDueToProp_ += 1;
                tbFailedDueToPropIgnoreSCI_ += 1;
            } else if (!interference_result) {
                tbFailedButSCIReceived_ += 1;
                tbFailedDueToInterference_ += 1;
                tbFailedDueToInterferenceIgnoreSCI_ += 1;
            } else {
                tbDecoded_ += 1;
                tbDecodedIgnoreSCI_ += 1;
                std::map<MacNodeId, simtime_t>::iterator jt = previousTransmissionTimes_.find(lteInfo->getSourceId());
                if ( jt != previousTransmissionTimes_.end() ) {
                    simtime_t elapsed_time = NOW - jt->second;
                    emit(interPacketDelay, elapsed_time);
                }
                previousTransmissionTimes_[lteInfo->getSourceId()] = NOW;
            }
            if (foundCorrespondingSci) {
                // Need to get the map only for the RBs used for transmission
                RbMap::iterator mt;
                std::map<Band, unsigned int>::iterator nt;
                RbMap usedRbs = lteInfo->getGrantedBlocks();

                // Now need to find the associated Subchannels, record the RSRP and RSSI for the message and go from there.
                // Need to again do the RIV steps
                std::tuple<int, int> indexAndLength = decodeRivValue(correspondingSCI, sciInfo);
                int subchannelIndex = std::get<0>(indexAndLength);
                int lengthInSubchannels = std::get<1>(indexAndLength);

                std::vector <Subchannel *> currentSubframe = sensingWindow_[sensingWindowFront_];
                for (int i = subchannelIndex; i < subchannelIndex + lengthInSubchannels; i++) {
                    Subchannel *currentSubchannel = currentSubframe[i];
                    std::vector<Band>::iterator lt;
                    std::vector <Band> allocatedBands = currentSubchannel->getOccupiedBands();
                    for (lt = allocatedBands.begin(); lt != allocatedBands.end(); lt++) {
                        // Record RSRP and RSSI for this band depending if it was used or not
                        bool used = false;

                        //for each Remote unit used to transmit the packet
                        for (mt = usedRbs.begin(); mt != usedRbs.end(); ++mt) {
                            //for each logical band used to transmit the packet
                            for (nt = mt->second.begin(); nt != mt->second.end(); ++nt) {
                                if (nt->first == *lt) {
                                    currentSubchannel->addRsrpValue(rsrpVector[(*lt)], (*lt));
                                    currentSubchannel->addRssiValue(rssiVector[(*lt)], (*lt));
                                    used = true;
                                    break;
                                }
                            }
                        }
                    }
                }
                // Need to delete the message now
                delete correspondingSCI;
                delete sciInfo;
            }
        }
        else{
            tbFailedHalfDuplex_ += 1;
        }

        delete frame;

        // send decapsulated message along with result control info to upperGateOut_
        lteInfo->setDeciderResult(interference_result);
        pkt->setControlInfo(lteInfo);
        send(pkt, upperGateOut_);

        if (getEnvir()->isGUI())
            updateDisplayString();
    }

    // update statistics
    if (interference_result)
    {
        numAirFrameReceived_++;
    }
    else
    {
        numAirFrameNotReceived_++;
    }

    EV << "Handled LteAirframe with ID " << frame->getId() << " with result "
       << (interference_result ? "RECEIVED" : "NOT RECEIVED") << endl;
}

std::tuple<int,int> LtePhyVUeMode4::decodeRivValue(SidelinkControlInformation* sci, UserControlInfo* sciInfo)
{
    EV << NOW << " LtePhyVUeMode4::decodeRivValue - Decoding RIV value of SCI allows correct placement in sensing window..." << endl;
    //UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(pkt->removeControlInfo());
    RbMap rbMap = sciInfo->getGrantedBlocks();
    RbMap::iterator it;
    std::map<Band, unsigned int>::iterator jt;
    Band startingBand;
    bool bandNotFound = true;

    it = rbMap.begin();

    while (it != rbMap.end() && bandNotFound )
    {
        for (jt = it->second.begin(); jt != it->second.end(); ++jt)
        {
            Band band = jt->first;
            if (jt->second == 1) // this Rb is not allocated
            {
                startingBand = band;
                bandNotFound= false;
                break;
            }
        }
    }

    // Get RIV first as this is common
    unsigned int RIV = sci->getFrequencyResourceLocation();

    // Get the subchannel Index (allows us later to calculate the length of the message
    int subchannelIndex;
    if (adjacencyPSCCHPSSCH_)
    {
        // If adjacent: Subchannel Index = band//subchannelSize
        subchannelIndex = startingBand/subchannelSize_;
    }
    else
    {
        // If non-adjacent: Subchannel Index = band/2
        subchannelIndex = startingBand/2;
    }

    // Based on TS36.213 14.1.1.4C
    // if SubchannelLength -1 < numSubchannels/2
    // RIV = numSubchannels(SubchannelLength-1) + subchannelIndex
    // else
    // RIV = numSubchannels(numSubchannels-SubchannelLength+1) + (numSubchannels-1-subchannelIndex)
    // RIV limit
    // log2(numSubchannels(numSubchannels+1)/2)
    double subchannelLOverHalf = (numSubchannels_ + 2 + (( -1 - subchannelIndex - RIV )/numSubchannels_));
    double subchannelLUnderHalf = (RIV + numSubchannels_ - subchannelIndex)/numSubchannels_;
    int lengthInSubchannels;

    // First the number has to be whole in both cases, it's length + subchannelIndex must be less than the number of subchannels
    if (floor(subchannelLOverHalf) == subchannelLOverHalf && subchannelLOverHalf <= numSubchannels_ && subchannelLOverHalf + subchannelIndex <= numSubchannels_)
    {
        lengthInSubchannels = subchannelLOverHalf;
    }
    // Same as above but also the length must be less than half + 1
    else if (floor(subchannelLUnderHalf) == subchannelLUnderHalf && subchannelLUnderHalf + subchannelIndex <= numSubchannels_ && subchannelLUnderHalf <= numSubchannels_ /2 + 1)
    {
        lengthInSubchannels = subchannelLUnderHalf;
    }
    return std::make_tuple(subchannelIndex, lengthInSubchannels);
}

void LtePhyVUeMode4::updateCBR()
{
    double cbrValue = 0.0;
    double cbrPscchValue = 0.0;

    int cbrIndex = sensingWindowFront_ - 1;
    int cbrCount = 0;
    int totalSubchannels = 0;

    if (sensingWindow_.size() > 99){
        cbrCount = 99;
    } else{
        cbrCount = sensingWindow_.size();
    }

    while (cbrCount > 0){
        if (cbrIndex == -1){
            cbrIndex = sensingWindow_.size() - 1;
        }
        std::vector<Subchannel *>::iterator it;
        std::vector <Subchannel *> currentSubframe = sensingWindow_[cbrIndex];
        for (it = currentSubframe.begin(); it != currentSubframe.end(); it++) {
            if ((*it)->getSensed()) {
                totalSubchannels++;
                if ((*it)->getAverageRSSI() > thresholdRSSI_) {
                    cbrValue++;
                }
                if ((*it)->getAverageRSSIPscch() > thresholdRSSI_) {
                    cbrPscchValue++;
                }
            }
        }
        cbrIndex --;
        cbrCount --;
    }

    cbrValue = cbrValue / totalSubchannels;
    cbrPscchValue = cbrPscchValue / totalSubchannels;

    emit(cbr, cbrValue);
    emit(cbrPscch, cbrPscchValue);

    Cbr* cbrPkt = new Cbr("CBR");
    cbrPkt->setCbr(cbrValue);
    send(cbrPkt, upperGateOut_);
}

void LtePhyVUeMode4::updateSubframe()
{
    // Ensure for every 1ms we record the position of the vehicle
    emit(posX, getCoord().x);
    emit(posY, getCoord().y);

    int sensingWindowLength = pStep_ * 10;
    if (sensingWindowSizeOverride_ > 0){
        sensingWindowLength = sensingWindowSizeOverride_;
    }

    // Increment the pointer to the next element in the sensingWindow
    if (sensingWindowFront_ < sensingWindowLength - 1) {
        ++sensingWindowFront_;
    }
    else{
        // Front has gone over the end of the sensing window reset it.
        sensingWindowFront_ = 0;
    }


    // First find the subframe that we want to look at i.e. the front one I imagine
    // If the front isn't occupied then skip on
    // If it is occupied, pop it off, update it and push it back
    // All good then.

    std::vector<Subchannel*> subframe = sensingWindow_[sensingWindowFront_];

    if (subframe.at(0)->getSubframeTime() <= NOW - SimTime(sensingWindowLength, SIMTIME_MS) - TTI)
    {
        std::vector<Subchannel*>::iterator it;
        for (it=subframe.begin(); it!=subframe.end(); it++)
        {
            (*it)->reset(NOW - TTI);
        }
    }

    cMessage* updateSubframe = new cMessage("updateSubframe");
    updateSubframe->setSchedulingPriority(0);        // Generate the subframe at start of next TTI
    scheduleAt(NOW + TTI, updateSubframe);
}

void LtePhyVUeMode4::initialiseSensingWindow()
{
    EV << NOW << " LtePhyVUeMode4::initialiseSensingWindow - creating subframes to be added to sensingWindow..." << endl;

    simtime_t subframeTime = NOW - TTI;

    int sensingWindowLength = pStep_ * 10;
    if (sensingWindowSizeOverride_ > 0){
        sensingWindowLength = sensingWindowSizeOverride_;
    }

    // Reserve the full size of the sensing window (might improve efficiency).
    sensingWindow_.reserve(sensingWindowLength);

    while(sensingWindow_.size() < sensingWindowLength)
    {
        std::vector<Subchannel*> subframe;
        subframe.reserve(numSubchannels_);
        Band band = 0;

        if (!adjacencyPSCCHPSSCH_)
        {
            // This assumes the bands only every have 1 Rb (which is fine as that appears to be the case)
            band = numSubchannels_*2;
        }
        for (int i = 0; i < numSubchannels_; i++) {
            Subchannel *currentSubchannel = new Subchannel(subchannelSize_, subframeTime);
            // Need to determine the RSRP and RSSI that corresponds to background noise
            // Best off implementing this in the channel model as a method.

            std::vector <Band> occupiedBands;

            int overallCapacity = 0;
            // Ensure the subchannel is allocated the correct number of RBs
            while (overallCapacity < subchannelSize_ && band < getBinder()->getNumBands()) {
                // This acts like there are multiple RBs per band which is not allowed.
                occupiedBands.push_back(band);
                ++overallCapacity;
                ++band;
            }
            currentSubchannel->setOccupiedBands(occupiedBands);
            subframe.push_back(currentSubchannel);
        }
        sensingWindow_.push_back(subframe);
        subframeTime += TTI;
    }
    // Send self message to trigger another subframes creation and insertion. Need one for every TTI
    cMessage* updateSubframe = new cMessage("updateSubframe");
    updateSubframe->setSchedulingPriority(0);        // Generate the subframe at start of next TTI
    scheduleAt(NOW + TTI, updateSubframe);
}

int LtePhyVUeMode4::translateIndex(int fallBack) {
    if (fallBack > sensingWindowFront_){
        int max = 10 * pStep_;
        if (sensingWindowSizeOverride_ > 0){
            max = sensingWindowSizeOverride_;
        }
        int fromMax = fallBack - sensingWindowFront_;
        return max - fromMax;
    } else{
        return sensingWindowFront_ - fallBack;
    }
}

void LtePhyVUeMode4::finish()
{
    if (getSimulation()->getSimulationStage() != CTX_FINISH)
    {
        // do this only at deletion of the module during the simulation
        //LtePhyUe::finish();
        LteAmc *amc = getAmcModule(masterId_);
        if (amc != NULL)
        {
            amc->detachUser(nodeId_, UL);
            amc->detachUser(nodeId_, DL);
            amc->detachUser(nodeId_, D2D);
        }

        // binder call
        binder_->unregisterNextHop(masterId_, nodeId_);

        // deployer call
        deployer_->detachUser(nodeId_);
    }

    std::vector<std::vector<Subchannel *>>::iterator it;
    for (it=sensingWindow_.begin();it!=sensingWindow_.end();it++)
    {
        std::vector<Subchannel *>::iterator jt;
        for (jt=it->begin();jt!=it->end();jt++)
        {
            delete (*jt);
        }
    }
    sensingWindow_.clear();
}
