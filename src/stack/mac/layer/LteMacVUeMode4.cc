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
 * LteMacVUeMode4 is a new model which implements the functionality of LTE Mode 4 as per 3GPP release 14
 * Author: Brian McCarthy
 * Email: b.mccarthy@cs.ucc.ie
 */

#include "stack/mac/buffer/harq/LteHarqBufferRx.h"
#include "stack/mac/buffer/LteMacQueue.h"
#include "stack/mac/buffer/harq_d2d/LteHarqBufferRxD2DMirror.h"
#include "stack/mac/layer/LteMacVUeMode4.h"
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

Define_Module(LteMacVUeMode4);

LteMacVUeMode4::LteMacVUeMode4() :
    LteMacUeRealisticD2D()
{
}

LteMacVUeMode4::~LteMacVUeMode4()
{
}

void LteMacVUeMode4::initialize(int stage)
{
    if (stage !=inet::INITSTAGE_NETWORK_LAYER_3)
    LteMacUeRealisticD2D::initialize(stage);

    if (stage == inet::INITSTAGE_LOCAL)
    {
        parseUeTxConfig(par("txConfig").xmlValue());
        parseCbrTxConfig(par("txConfig").xmlValue());
        parseRriConfig(par("txConfig").xmlValue());
        resourceReservationInterval_ = validResourceReservationIntervals_.at(0);
        subchannelSize_ = par("subchannelSize");
        numSubchannels_ = par("numSubchannels");
        probResourceKeep_ = par("probResourceKeep");
        usePreconfiguredTxParams_ = par("usePreconfiguredTxParams");
        reselectAfter_ = par("reselectAfter");
        useCBR_ = par("useCBR");
        packetDropping_ = par("packetDropping");
        adjacencyPSCCHPSSCH_ = par("adjacencyPSCCHPSSCH");
        maximumCapacity_ = 0;
        cbr_=0;
        currentCw_=0;
        missedTransmissions_=0;

        expiredGrant_ = false;

        currentCbrIndex_ = defaultCbrIndex_;

        // Register the necessary signals for this simulation

        grantStartTime          = registerSignal("grantStartTime");
        grantBreak              = registerSignal("grantBreak");
        grantBreakTiming        = registerSignal("grantBreakTiming");
        grantBreakSize          = registerSignal("grantBreakSize");
        droppedTimeout          = registerSignal("droppedTimeout");
        grantBreakMissedTrans   = registerSignal("grantBreakMissedTrans");
        missedTransmission      = registerSignal("missedTransmission");
        selectedMCS             = registerSignal("selectedMCS");
        selectedSubchannelIndex = registerSignal("selectedSubchannelIndex");
        selectedNumSubchannels  = registerSignal("selectedNumSubchannels");
        maximumCapacity         = registerSignal("maximumCapacity");
        grantRequests           = registerSignal("grantRequests");
        macNodeID               = registerSignal("macNodeID");
        rrcSelected             = registerSignal("resourceReselectionCounter");
        retainGrant             = registerSignal("retainGrant");
    }
    else if (stage == inet::INITSTAGE_NETWORK_LAYER_3)
    {
        deployer_ = getDeployer();
        numAntennas_ = getNumAntennas();
        mcsScaleD2D_ = deployer_->getMcsScaleUl();
        d2dMcsTable_.rescale(mcsScaleD2D_);

        if (usePreconfiguredTxParams_)
        {
            preconfiguredTxParams_ = getPreconfiguredTxParams();
        }

        // LTE UE Section
        nodeId_ = getAncestorPar("macNodeId");

        emit(macNodeID, nodeId_);

        /* Insert UeInfo in the Binder */
        ueInfo_ = new UeInfo();
        ueInfo_->id = nodeId_;            // local mac ID
        ueInfo_->cellId = cellId_;        // cell ID
        ueInfo_->init = false;            // flag for phy initialization
        ueInfo_->ue = this->getParentModule()->getParentModule();  // reference to the UE module

        // Get the Physical Channel reference of the node
        ueInfo_->phy = check_and_cast<LtePhyBase*>(ueInfo_->ue->getSubmodule("lteNic")->getSubmodule("phy"));

        binder_->addUeInfo(ueInfo_);
    }
}

void LteMacVUeMode4::parseUeTxConfig(cXMLElement* xmlConfig)
{
    if (xmlConfig == 0)
    throw cRuntimeError("No sidelink configuration file specified");

    // Get channel Model field which contains parameters fields
    cXMLElementList ueTxConfig = xmlConfig->getElementsByTagName("userEquipment-txParameters");

    if (ueTxConfig.empty())
        throw cRuntimeError("No userEquipment-txParameters configuration found in configuration file");

    if (ueTxConfig.size() > 1)
        throw cRuntimeError("More than one userEquipment-txParameters configuration found in configuration file.");

    cXMLElement* ueTxConfigData = ueTxConfig.front();

    ParameterMap params;
    getParametersFromXML(ueTxConfigData, params);

    //get lambda max threshold
    ParameterMap::iterator it = params.find("minMCS-PSSCH");
    if (it != params.end())
    {
        minMCSPSSCH_ = (int)it->second;
    }
    else
        minMCSPSSCH_ = (int)par("minMCSPSSCH");
    it = params.find("maxMCS-PSSCH");
    if (it != params.end())
    {
        maxMCSPSSCH_ = (int)it->second;
    }
    else
        maxMCSPSSCH_ = (int)par("minMCSPSSCH");
    it = params.find("minSubchannel-NumberPSSCH");
    if (it != params.end())
    {
        minSubchannelNumberPSSCH_ = (int)it->second;
    }
    else
        minSubchannelNumberPSSCH_ = (int)par("minSubchannelNumberPSSCH");
    it = params.find("maxSubchannel-NumberPSSCH");
    if (it != params.end())
    {
        maxSubchannelNumberPSSCH_ = (int)it->second;
    }
    else
        maxSubchannelNumberPSSCH_ = (int)par("maxSubchannelNumberPSSCH");
    it = params.find("allowedRetxNumberPSSCH");
    if (it != params.end())
    {
        allowedRetxNumberPSSCH_ = (int)it->second;
    }
    else
        allowedRetxNumberPSSCH_ = (int)par("allowedRetxNumberPSSCH");
}

void LteMacVUeMode4::parseCbrTxConfig(cXMLElement* xmlConfig)
{
    if (xmlConfig == 0)
    throw cRuntimeError("No cbr configuration specified");

    // Get channel Model field which contains parameters fields
    cXMLElementList cbrTxConfig = xmlConfig->getElementsByTagName("Sl-CBR-CommonTxConfigList");

    if (cbrTxConfig.empty())
        throw cRuntimeError("No Sl-CBR-CommonTxConfigList found in configuration file");

    cXMLElement* cbrTxConfigData = cbrTxConfig.front();

    ParameterMap params;
    getParametersFromXML(cbrTxConfigData, params);

    //get lambda max threshold
    ParameterMap::iterator it = params.find("default-cbr-ConfigIndex");
    if (it != params.end())
    {
        defaultCbrIndex_ = it->second;
        currentCbrIndex_ = defaultCbrIndex_;
    }

    cXMLElementList cbrLevelConfigs = xmlConfig->getElementsByTagName("cbr-ConfigIndex");

    if (cbrLevelConfigs.empty())
        throw cRuntimeError("No cbr-Levels-Config found in configuration file");

    cXMLElementList::iterator xmlIt;
    for(xmlIt = cbrLevelConfigs.begin(); xmlIt != cbrLevelConfigs.end(); xmlIt++)
    {
        std::unordered_map<std::string, double> cbrLevelsMap;
        ParameterMap cbrLevelsParams;
        getParametersFromXML((*xmlIt), cbrLevelsParams);
        it = cbrLevelsParams.find("cbr-lower");
        if (it != cbrLevelsParams.end())
        {
            cbrLevelsMap.insert(pair<std::string, double>("cbr-lower",  it->second));
        }
        it = cbrLevelsParams.find("cbr-upper");
        if (it != cbrLevelsParams.end())
        {
            cbrLevelsMap.insert(pair<std::string, double>("cbr-upper",  it->second));
        }
        it = cbrLevelsParams.find("cbr-PSSCH-TxConfig-Index");
        if (it != cbrLevelsParams.end())
        {
            cbrLevelsMap.insert(pair<std::string, double>("cbr-PSSCH-TxConfig-Index",  it->second));
        }
        cbrLevels_.push_back(cbrLevelsMap);
    }

    cXMLElementList cbrTxConfigs = xmlConfig->getElementsByTagName("cbr-PSSCH-TxConfig");

    if (cbrTxConfigs.empty())
        throw cRuntimeError("No CBR-TxConfig found in configuration file");

    cXMLElementList cbrTxParams = xmlConfig->getElementsByTagName("txParameters");

    for(xmlIt = cbrTxParams.begin(); xmlIt != cbrTxParams.end(); xmlIt++)
    {
        std::unordered_map<std::string, double> cbrMap;
        ParameterMap cbrParams;
        getParametersFromXML((*xmlIt), cbrParams);
        it = cbrParams.find("minMCS-PSSCH");
        if (it != cbrParams.end())
        {
            cbrMap.insert({"minMCS-PSSCH",  it->second});
        }
        else
            cbrMap.insert({"minMCS-PSSCH",  par("minMCSPSSCH")});
        it = cbrParams.find("maxMCS-PSSCH");
        if (it != cbrParams.end())
        {
            cbrMap.insert({"maxMCS-PSSCH",  it->second});
        }
        else
            cbrMap.insert({"maxMCS-PSSCH",  par("maxMCSPSSCH")});
        it = cbrParams.find("minSubchannel-NumberPSSCH");
        if (it != cbrParams.end())
        {
            cbrMap.insert({"minSubchannel-NumberPSSCH",  it->second});
        }
        else
            cbrMap.insert({"minSubchannel-NumberPSSCH",  par("minSubchannelNumberPSSCH")});
        it = cbrParams.find("maxSubchannel-NumberPSSCH");
        if (it != cbrParams.end())
        {
            cbrMap.insert({"maxSubchannel-NumberPSSCH",  it->second});
        }
        else
            cbrMap.insert({"maxSubchannel-NumberPSSCH",  par("maxSubchannelNumberPSSCH")});
        it = cbrParams.find("allowedRetxNumberPSSCH");
        if (it != cbrParams.end())
        {
            cbrMap.insert({"allowedRetxNumberPSSCH",  it->second});
        }
        else
            cbrMap.insert({"allowedRetxNumberPSSCH",  par("allowedRetxNumberPSSCH")});
        it = cbrParams.find("allowedRRI");
        if (it != cbrParams.end())
        {
            cbrMap.insert({"allowedRRI",  it->second});
        }
        else
            cbrMap.insert({"allowedRRI",  resourceReservationInterval_});
        it = cbrParams.find("cr-Limit");
        if (it != cbrParams.end())
        {
            cbrMap.insert({"cr-Limit",  it->second});
        }
        cbrPSSCHTxConfigList_.push_back(cbrMap);
    }
}

void LteMacVUeMode4::parseRriConfig(cXMLElement* xmlConfig)
{
    if (xmlConfig == 0)
    throw cRuntimeError("No cbr configuration specified");

    // Get channel Model field which contains parameters fields
    cXMLElementList rriConfig = xmlConfig->getElementsByTagName("RestrictResourceReservationPeriodList");

    if (rriConfig.empty())
        throw cRuntimeError("No RestrictResourceReservationPeriodList found in configuration file");

    cXMLElementList rriConfigs = xmlConfig->getElementsByTagName("RestrictResourceReservationPeriod");

    if (rriConfigs.empty())
        throw cRuntimeError("No RestrictResourceReservationPeriods found in configuration file");

    cXMLElementList::iterator xmlIt;
    for(xmlIt = rriConfigs.begin(); xmlIt != rriConfigs.end(); xmlIt++)
    {
        ParameterMap rriParams;
        getParametersFromXML((*xmlIt), rriParams);
        ParameterMap::iterator it = rriParams.find("rri");
        if (it != rriParams.end())
        {
            validResourceReservationIntervals_.push_back(it->second);
        }
    }
}

int LteMacVUeMode4::getNumAntennas()
{
    /* Get number of antennas: +1 is for MACRO */
    return deployer_->getNumRus() + 1;
}

void LteMacVUeMode4::macPduMake()
{
    int64 size = 0;

    macPduList_.clear();

    // In a D2D communication if BSR was created above this part isn't executed
    // Build a MAC PDU for each scheduled user on each codeword
    LteMacScheduleList::const_iterator it;
    for (it = scheduleList_->begin(); it != scheduleList_->end(); it++)
    {
        LteMacPdu* macPkt;
        cPacket* pkt;

        MacCid destCid = it->first.first;
        Codeword cw = it->first.second;

        // get the direction (UL/D2D/D2D_MULTI) and the corresponding destination ID
        LteControlInfo* lteInfo = &(connDesc_.at(destCid));
        MacNodeId destId = lteInfo->getDestId();
        Direction dir = (Direction)lteInfo->getDirection();

        std::pair<MacNodeId, Codeword> pktId = std::pair<MacNodeId, Codeword>(destId, cw);
        unsigned int sduPerCid = it->second;

        MacPduList::iterator pit = macPduList_.find(pktId);

        if (sduPerCid == 0)
        {
            continue;
        }

        // No packets for this user on this codeword
        if (pit == macPduList_.end())
        {
            // Always goes here because of the macPduList_.clear() at the beginning
            // Build the Control Element of the MAC PDU
            UserControlInfo* uinfo = new UserControlInfo();
            uinfo->setSourceId(getMacNodeId());
            uinfo->setDestId(destId);
            uinfo->setLcid(MacCidToLcid(destCid));
            uinfo->setDirection(dir);
            uinfo->setLcid(MacCidToLcid(SHORT_BSR));

            // First translate MCS to CQI
            LteMode4SchedulingGrant* mode4Grant = check_and_cast<LteMode4SchedulingGrant*>(schedulingGrant_);

            if (usePreconfiguredTxParams_)
            {
                //UserTxParams* userTxParams = preconfiguredTxParams_;
                uinfo->setUserTxParams(preconfiguredTxParams_->dup());
                mode4Grant->setUserTxParams(preconfiguredTxParams_->dup());
            }
            else
                uinfo->setUserTxParams(mode4Grant->getUserTxParams()->dup());

            // Create a PDU
            macPkt = new LteMacPdu("LteMacPdu");
            macPkt->setHeaderLength(MAC_HEADER);
            macPkt->setControlInfo(uinfo);
            macPkt->setTimestamp(NOW);
            macPduList_[pktId] = macPkt;
        }
        else
        {
            // Never goes here because of the macPduList_.clear() at the beginning
            macPkt = pit->second;
        }

        while (sduPerCid > 0)
        {
            // Add SDU to PDU
            // Find Mac Pkt
            if (mbuf_.find(destCid) == mbuf_.end())
                throw cRuntimeError("Unable to find mac buffer for cid %d", destCid);

            if (mbuf_[destCid]->empty())
                throw cRuntimeError("Empty buffer for cid %d, while expected SDUs were %d", destCid, sduPerCid);

            pkt = mbuf_[destCid]->popFront();

            // multicast support
            // this trick gets the group ID from the MAC SDU and sets it in the MAC PDU
            int32 groupId = check_and_cast<LteControlInfo*>(pkt->getControlInfo())->getMulticastGroupId();
            if (groupId >= 0) // for unicast, group id is -1
                check_and_cast<LteControlInfo*>(macPkt->getControlInfo())->setMulticastGroupId(groupId);

            drop(pkt);

            macPkt->pushSdu(pkt);
            sduPerCid--;
        }

        // consider virtual buffers to compute BSR size
        size += macBuffers_[destCid]->getQueueOccupancy();

        if (size > 0)
        {
            // take into account the RLC header size
            if (connDesc_[destCid].getRlcType() == UM)
                size += RLC_HEADER_UM;
            else if (connDesc_[destCid].getRlcType() == AM)
                size += RLC_HEADER_AM;
        }
    }

    // Put MAC PDUs in H-ARQ buffers
    MacPduList::iterator pit;
    for (pit = macPduList_.begin(); pit != macPduList_.end(); pit++)
    {
        MacNodeId destId = pit->first.first;
        Codeword cw = pit->first.second;
        // Check if the HarqTx buffer already exists for the destId
        // Get a reference for the destId TXBuffer
        LteHarqBufferTx* txBuf;
        HarqTxBuffers::iterator hit = harqTxBuffers_.find(destId);
        if ( hit != harqTxBuffers_.end() )
        {
            // The tx buffer already exists
            txBuf = hit->second;
        }
        else
        {
            // The tx buffer does not exist yet for this mac node id, create one
            LteHarqBufferTx* hb;
            // FIXME: hb is never deleted
            UserControlInfo* info = check_and_cast<UserControlInfo*>(pit->second->getControlInfo());
            if (info->getDirection() == UL)
                hb = new LteHarqBufferTx((unsigned int) ENB_TX_HARQ_PROCESSES, this, (LteMacBase*) getMacByMacNodeId(destId));
            else // D2D or D2D_MULTI
                hb = new LteHarqBufferTxD2D((unsigned int) ENB_TX_HARQ_PROCESSES, this, (LteMacBase*) getMacByMacNodeId(destId));
            harqTxBuffers_[destId] = hb;
            txBuf = hb;
        }

        // search for an empty unit within current harq process
        UnitList txList = txBuf->getEmptyUnits(currentHarq_);
        EV << "LteMacUeRealisticD2D::macPduMake - [Used Acid=" << (unsigned int)txList.first << "] , [curr=" << (unsigned int)currentHarq_ << "]" << endl;

        //Get a reference of the LteMacPdu from pit pointer (extract Pdu from the MAP)
        LteMacPdu* macPkt = pit->second;

        EV << "LteMacUeRealisticD2D: pduMaker created PDU: " << macPkt->info() << endl;

        if (txList.second.empty())
        {
            EV << "LteMacUeRealisticD2D() : no available process for this MAC pdu in TxHarqBuffer" << endl;
            delete macPkt;
        }
        else
        {
            //Insert PDU in the Harq Tx Buffer
            //txList.first is the acid
            txBuf->insertPdu(txList.first,cw, macPkt);
        }
    }
}

UserTxParams* LteMacVUeMode4::getPreconfiguredTxParams()
{
    UserTxParams* txParams = new UserTxParams();

    // default parameters for D2D
    txParams->isSet() = true;
    txParams->writeTxMode(SINGLE_ANTENNA_PORT0);
    Rank ri = 1;                                              // rank for TxD is one
    txParams->writeRank(ri);
    txParams->writePmi(intuniform(1, pow(ri, (double) 2)));   // taken from LteFeedbackComputationRealistic::computeFeedback

    BandSet b;
    for (Band i = 0; i < deployer_->getNumBands(); ++i) b.insert(i);
    txParams->writeBands(b);

    RemoteSet antennas;
    antennas.insert(MACRO);
    txParams->writeAntennas(antennas);

    return txParams;
}

double LteMacVUeMode4::calculateChannelOccupancyRatio(int period){
    int b;
    int a;
    int subchannelsUsed = 0;
    // CR limit calculation
    // determine b
    if (schedulingGrant_ != NULL){
        LteMode4SchedulingGrant* mode4Grant = check_and_cast<LteMode4SchedulingGrant*>(schedulingGrant_);
        if (expirationCounter_ > 499){
            b = 499;
        } else {
            b = expirationCounter_;
        }
        subchannelsUsed += (b / period) * mode4Grant->getNumSubchannels();
    } else {
        b = 0;
    }
    // determine a
    a = 999 - b;

    // determine previous transmissions -> Need to account for if we have already done a drop. Must maintain a
    // history of past transmissions i.e. subchannels used and subframe in which they occur. delete entries older
    // than 1000.
    std::unordered_map<double, int>::const_iterator it = previousTransmissions_.begin();
    while (it != previousTransmissions_.end()){
        if (it->first < NOW.dbl() - 1){
            it = previousTransmissions_.erase(it);
            continue;
        } else if (it->first > NOW.dbl() - (0.001 * a)) {
            subchannelsUsed += it->second;
        }
        it++;
    }
    // calculate cr
    return subchannelsUsed /(numSubchannels_ * 1000.0);
}

void LteMacVUeMode4::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage())
    {
        LteMacUeRealisticD2D::handleMessage(msg);
        return;
    }

    cPacket* pkt = check_and_cast<cPacket *>(msg);
    cGate* incoming = pkt->getArrivalGate();

    if (incoming == down_[IN])
    {
        if (strcmp(pkt->getName(), "CSRs") == 0)
        {
            EV << "LteMacVUeMode4::handleMessage - Received packet " << pkt->getName() <<
            " from port " << pkt->getArrivalGate()->getName() << endl;

            // message from PHY_to_MAC gate (from lower layer)
            emit(receivedPacketFromLowerLayer, pkt);

            // call handler
            macHandleSps(pkt);

            return;
        }
        if (strcmp(pkt->getName(), "CBR") == 0)
        {
            Cbr* cbrPkt = check_and_cast<Cbr*>(pkt);
            cbr_ = cbrPkt->getCbr();

            if (useCBR_) {

                std::vector<std::unordered_map<std::string, double>>::iterator it;
                for (it = cbrLevels_.begin(); it!=cbrLevels_.end(); it++)
                {
                    double cbrUpper = (*it).at("cbr-upper");
                    double cbrLower = (*it).at("cbr-lower");
                    double index = (*it).at("cbr-PSSCH-TxConfig-Index");
                    if (cbrLower == 0){
                        if (cbr_< cbrUpper)
                        {
                            currentCbrIndex_ = (int)index;
                            break;
                        }
                    } else if (cbrUpper == 1){
                        if (cbr_ > cbrLower)
                        {
                            currentCbrIndex_ = (int)index;
                            break;
                        }
                    } else {
                        if (cbr_ > cbrLower && cbr_<= cbrUpper)
                        {
                            currentCbrIndex_ = (int)index;
                            break;
                        }
                    }
                }
            }

            int period = 0;
            if (schedulingGrant_ != NULL){
                period = schedulingGrant_->getPeriod();
            }
            channelOccupancyRatio_ = calculateChannelOccupancyRatio(period);

            // message from PHY_to_MAC gate (from lower layer)
            emit(receivedPacketFromLowerLayer, pkt);

            LteMacBase::sendUpperPackets(cbrPkt);

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
            else if ((schedulingGrant_ != NULL && periodCounter_ > remainingTime_))
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



void LteMacVUeMode4::handleSelfMessage()
{
    EV << "----- UE MAIN LOOP -----" << endl;

    // extract pdus from all harqrxbuffers and pass them to unmaker
    HarqRxBuffers::iterator hit = harqRxBuffers_.begin();
    HarqRxBuffers::iterator het = harqRxBuffers_.end();
    LteMacPdu *pdu = NULL;
    std::list<LteMacPdu*> pduList;

    for (; hit != het; ++hit)
    {
        pduList=hit->second->extractCorrectPdus();
        while (! pduList.empty())
        {
            pdu=pduList.front();
            pduList.pop_front();
            macPduUnmake(pdu);
        }
    }

    EV << NOW << "LteMacVUeMode4::handleSelfMessage " << nodeId_ << " - HARQ process " << (unsigned int)currentHarq_ << endl;
    // updating current HARQ process for next TTI

    //unsigned char currentHarq = currentHarq_;

    // no grant available - if user has backlogged data, it will trigger scheduling request
    // no harq counter is updated since no transmission is sent.

    LteMode4SchedulingGrant* mode4Grant = dynamic_cast<LteMode4SchedulingGrant*>(schedulingGrant_);

    if (mode4Grant == NULL)
    {
        EV << NOW << " LteMacVUeMode4::handleSelfMessage " << nodeId_ << " NO configured grant" << endl;

        // No configured Grant simply continue
    }
    else if (mode4Grant->getPeriodic() && mode4Grant->getStartTime() <= NOW)
    {
        // Periodic checks
        if(--expirationCounter_ == mode4Grant->getPeriod())
        {
            // Gotten to the point of the final tranmission must determine if we reselect or not.
            double randomReReserve = dblrand(1);
            if (randomReReserve < probResourceKeep_)
            {
                int expiration = 0;
                if (resourceReservationInterval_ == 0.5){
                    expiration = intuniform(10, 30, 3);
                } else if (resourceReservationInterval_ == 0.2){
                    expiration = intuniform(25, 75, 3);
                } else {
                    expiration = intuniform(5, 15, 3);
                }
                mode4Grant -> setResourceReselectionCounter(expiration);
                mode4Grant -> setFirstTransmission(true);
                expirationCounter_ = expiration * mode4Grant->getPeriod();
                emit(rrcSelected, expiration);
                emit(retainGrant, 1);
            }
        }
        if (--periodCounter_>0 && !mode4Grant->getFirstTransmission())
        {
            return;
        }
        else if (expirationCounter_ > 0)
        {
            // resetting grant period
            periodCounter_=mode4Grant->getPeriod();
            // this is periodic grant TTI - continue with frame sending
        }
        else if (expirationCounter_ <= 0)
        {
            emit(grantBreak, 1);
            mode4Grant->setExpiration(0);
            expiredGrant_ = true;
        }
    }
    bool requestSdu = false;
    if (mode4Grant!=NULL && mode4Grant->getStartTime() <= NOW) // if a grant is configured
    {
        if (mode4Grant->getFirstTransmission())
        {
            mode4Grant->setFirstTransmission(false);
        }
        if(!firstTx)
        {
            EV << "\t currentHarq_ counter initialized " << endl;
            firstTx=true;
            // the eNb will receive the first pdu in 2 TTI, thus initializing acid to 0
//            currentHarq_ = harqRxBuffers_.begin()->second->getProcesses() - 2;
            currentHarq_ = UE_TX_HARQ_PROCESSES - 2;
        }
        EV << "\t " << schedulingGrant_ << endl;

        EV << NOW << " LteMacVUeMode4::handleSelfMessage " << nodeId_ << " entered scheduling" << endl;

        bool retx = false;
        bool availablePdu = false;

        HarqTxBuffers::iterator it2;
        LteHarqBufferTx * currHarq;
        for(it2 = harqTxBuffers_.begin(); it2 != harqTxBuffers_.end(); it2++)
        {
            EV << "\t Looking for retx in acid " << (unsigned int)currentHarq_ << endl;
            currHarq = it2->second;

            // check if the current process has unit ready for retx
            retx = currHarq->getProcess(currentHarq_)->hasReadyUnits();
            CwList cwListRetx = currHarq->getProcess(currentHarq_)->readyUnitsIds();

            if (it2->second->isSelected())
            {
                LteHarqProcessTx* selectedProcess = it2->second->getSelectedProcess();
                // Ensure that a pdu is not already on the HARQ buffer awaiting sending.
                if (selectedProcess != NULL)
                {
                    for (int cw=0; cw<MAX_CODEWORDS; cw++)
                    {
                        if (selectedProcess->getPduLength(cw) != 0)
                        {
                            availablePdu = true;
                        }
                    }
                }
            }

            EV << "\t [process=" << (unsigned int)currentHarq_ << "] , [retx=" << ((retx)?"true":"false")
               << "] , [n=" << cwListRetx.size() << "]" << endl;

            // if a retransmission is needed
            if(retx)
            {
                UnitList signal;
                signal.first=currentHarq_;
                signal.second = cwListRetx;
                currHarq->markSelected(signal,schedulingGrant_->getUserTxParams()->getLayers().size());
            }
        }
        // if no retx is needed, proceed with normal scheduling
        // TODO: This may yet be changed to appear after MCS selection, issue is that if you pick max then you might get more sdus then you want
        // Basing it on the previous mcs value is at least more realistic as to the size of the pdu you will get.
        if(!retx && !availablePdu)
        {
            scheduleList_ = lcgScheduler_->schedule();
            bool sent = macSduRequest();

            if (!sent)
            {
                macPduMake();
            }

            requestSdu = sent;
        }
        // Message that triggers flushing of Tx H-ARQ buffers for all users
        // This way, flushing is performed after the (possible) reception of new MAC PDUs
        cMessage* flushHarqMsg = new cMessage("flushHarqMsg");
        flushHarqMsg->setSchedulingPriority(1);        // after other messages
        scheduleAt(NOW, flushHarqMsg);
    }
    //============================ DEBUG ==========================
    HarqTxBuffers::iterator it;

    EV << "\n htxbuf.size " << harqTxBuffers_.size() << endl;

    int cntOuter = 0;
    int cntInner = 0;
    for(it = harqTxBuffers_.begin(); it != harqTxBuffers_.end(); it++)
    {
        LteHarqBufferTx* currHarq = it->second;
        BufferStatus harqStatus = currHarq->getBufferStatus();
        BufferStatus::iterator jt = harqStatus.begin(), jet= harqStatus.end();

        EV_DEBUG << "\t cicloOuter " << cntOuter << " - bufferStatus.size=" << harqStatus.size() << endl;
        for(; jt != jet; ++jt)
        {
            EV_DEBUG << "\t\t cicloInner " << cntInner << " - jt->size=" << jt->size()
               << " - statusCw(0/1)=" << jt->at(0).second << "/" << jt->at(1).second << endl;
        }
    }
    //======================== END DEBUG ==========================

    unsigned int purged =0;
    // purge from corrupted PDUs all Rx H-HARQ buffers
    for (hit= harqRxBuffers_.begin(); hit != het; ++hit)
    {
        // purge corrupted PDUs only if this buffer is for a DL transmission. Otherwise, if you
        // purge PDUs for D2D communication, also "mirror" buffers will be purged
        if (hit->first == cellId_)
            purged += hit->second->purgeCorruptedPdus();
    }
    EV << NOW << " LteMacVUeMode4::handleSelfMessage Purged " << purged << " PDUS" << endl;

    if (!requestSdu)
    {
        // update current harq process id
        currentHarq_ = (currentHarq_+1) % harqProcesses_;
    }

    EV << "--- END UE MAIN LOOP ---" << endl;
}




void LteMacVUeMode4::macHandleSps(cPacket* pkt)
{
    /**   This is where we add the subchannels to the actual scheduling grant, so a few things
     * 1. Need to ensure in the self message part that if at any point we have a scheduling grant without assigned subchannels, we have to wait
     * 2. Need to pick at random from the SPS list of CSRs
     * 3. Assign the CR
     * 4. return
     */
    SpsCandidateResources* candidatesPacket = check_and_cast<SpsCandidateResources *>(pkt);
    std::vector<std::tuple<double, int, int>> CSRs = candidatesPacket->getCSRs();

    LteMode4SchedulingGrant* mode4Grant = check_and_cast<LteMode4SchedulingGrant*>(schedulingGrant_);

    // Select random element from vector
    int index = intuniform(0, CSRs.size()-1, 1);

    std::tuple<double, int, int> selectedCR = CSRs[index];

//    //    Changes have been made to fix the <subframe, subchannel> for all vehicles
//        std::tuple<double, int, int> selectedCR ;
//        if(nodeId_ % 5 == 0)
//            selectedCR = std::make_tuple(-128, 1, 0);
//        else
//            selectedCR = std::make_tuple(-128, 5, 0);


    // Gives us the time at which we will send the subframe.
    simtime_t selectedStartTime = (simTime() + SimTime(std::get<1>(selectedCR), SIMTIME_MS) - TTI).trunc(SIMTIME_MS);

    emit(grantStartTime, selectedStartTime);

    int initiailSubchannel = std::get<2>(selectedCR);
    int finalSubchannel = initiailSubchannel + mode4Grant->getNumSubchannels(); // Is this actually one additional subchannel?

    // Emit statistic about the use of resources, i.e. the initial subchannel and it's length.
    emit(selectedSubchannelIndex, initiailSubchannel);
    emit(selectedNumSubchannels, mode4Grant->getNumSubchannels());

    // Determine the RBs on which we will send our message
    RbMap grantedBlocks;
    int totalGrantedBlocks = 0;
    if (adjacencyPSCCHPSSCH_){
        // Adjacent mode just provide the allocated bands

        for (int i=initiailSubchannel;i<finalSubchannel;i++)
        {
            int initialBand = i * subchannelSize_;
            for (Band b = initialBand; b < initialBand + subchannelSize_ ; b++)
            {
                grantedBlocks[MACRO][b] = 1;
                ++totalGrantedBlocks;
            }
        }
    } else {
        // Start at subchannel numsubchannels.
        // Remove 1 subchannel from each grant.
        // Account for the two blocks taken for the SCI
        int initialBand;
        if (initiailSubchannel == 0){
            // If first subchannel to use need to make sure to add the number of subchannels for the SCI messages
            initialBand = numSubchannels_ * 2;
        } else {
            initialBand = (numSubchannels_ * 2) + (initiailSubchannel * (subchannelSize_ - 2));
        }
        for (Band b = initialBand; b < initialBand + (subchannelSize_ * mode4Grant->getNumSubchannels()) ; b++) {
            grantedBlocks[MACRO][b] = 1;
            ++totalGrantedBlocks;
        }
    }

    mode4Grant->setStartTime(selectedStartTime);
    mode4Grant->setPeriodic(true);
    mode4Grant->setGrantedBlocks(grantedBlocks);
    mode4Grant->setTotalGrantedBlocks(totalGrantedBlocks); // account for the 2 RBs used for the sci message
    mode4Grant->setDirection(D2D_MULTI);
    mode4Grant->setCodewords(1);
    mode4Grant->setStartingSubchannel(initiailSubchannel);
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
    // Simply flips the codeword.
    currentCw_ = MAX_CODEWORDS - currentCw_;

    periodCounter_= mode4Grant->getPeriod();
    expirationCounter_= (mode4Grant->getResourceReselectionCounter() * periodCounter_) + 1;

    // TODO: Setup for HARQ retransmission, if it can't be satisfied then selection must occur again.

    CSRs.clear();

    delete pkt;
}

void LteMacVUeMode4::macGenerateSchedulingGrant(double maximumLatency, int priority)
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

    if (useCBR_)
    {
        int cbrMinSubchannelNum;
        int cbrMaxSubchannelNum;
        std::unordered_map<std::string,double> cbrMap = cbrPSSCHTxConfigList_.at(currentCbrIndex_);

        std::unordered_map<std::string,double>::const_iterator got = cbrMap.find("allowedRetxNumberPSSCH");
        if ( got == cbrMap.end() )
            allowedRetxNumberPSSCH_ = allowedRetxNumberPSSCH_;
        else
            allowedRetxNumberPSSCH_ = min((int)got->second, allowedRetxNumberPSSCH_);

        got = cbrMap.find("minSubchannel-NumberPSSCH");
        if ( got == cbrMap.end() )
            cbrMinSubchannelNum = minSubchannelNumberPSSCH_;
        else
            cbrMinSubchannelNum = (int)got->second;

        got = cbrMap.find("maxSubchannel-NumberPSSCH");
        if ( got == cbrMap.end() )
            cbrMaxSubchannelNum = maxSubchannelNumberPSSCH_;
        else
            cbrMaxSubchannelNum = (int)got->second;

        /**
         * Need to pick the number of subchannels for this reservation
         */
        if (maxSubchannelNumberPSSCH_ < cbrMinSubchannelNum || cbrMaxSubchannelNum < minSubchannelNumberPSSCH_)
        {
            // No overlap therefore I will use the cbr values (this is left to the UE, the opposite approach is also entirely valid).
            minSubchannelNumberPSSCH = cbrMinSubchannelNum;
            maxSubchannelNumberPSSCH = cbrMaxSubchannelNum;
        }
        else
        {
            minSubchannelNumberPSSCH = max(minSubchannelNumberPSSCH_, cbrMinSubchannelNum);
            maxSubchannelNumberPSSCH = min(maxSubchannelNumberPSSCH_, cbrMaxSubchannelNum);
        }
    }
    // Selecting the number of subchannel at random as there is no explanation as to the logic behind selecting the resources in the range unlike when selecting MCS.
    int numSubchannels = intuniform(minSubchannelNumberPSSCH, maxSubchannelNumberPSSCH, 2);

    mode4Grant -> setNumberSubchannels(numSubchannels);

    // Based on restrictResourceReservation interval But will be between 1 and 15
    // Again technically this needs to reconfigurable as well. But all of that needs to come in through ini and such.
    int resourceReselectionCounter = 0;

    if (resourceReservationInterval_ == 0.5) {
        resourceReselectionCounter = intuniform(10, 30, 3);
    } else if (resourceReservationInterval_ == 0.2) {
        resourceReselectionCounter = intuniform(25, 75, 3);
    } else {
        resourceReselectionCounter = intuniform(5, 15, 3);
    }

    mode4Grant -> setResourceReselectionCounter(resourceReselectionCounter);
    mode4Grant -> setExpiration(resourceReselectionCounter * resourceReservationInterval);
    emit(rrcSelected, resourceReselectionCounter);

    LteMode4SchedulingGrant* phyGrant = mode4Grant->dup();

    UserControlInfo* uinfo = new UserControlInfo();
    uinfo->setSourceId(getMacNodeId());
    uinfo->setDestId(getMacNodeId());
    uinfo->setFrameType(GRANTPKT);

    phyGrant->setControlInfo(uinfo);

    sendLowerPackets(phyGrant);

    schedulingGrant_ = mode4Grant;

    emit(grantRequests, 1);
}

void LteMacVUeMode4::flushHarqBuffers()
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

                            if (!mode4Grant->getUserTxParams())
                            {
                                mode4Grant->setUserTxParams(preconfiguredTxParams_->dup());
                            }

                            LteMode4SchedulingGrant* phyGrant = mode4Grant->dup();

                            UserControlInfo* uinfo = new UserControlInfo();
                            uinfo->setSourceId(getMacNodeId());
                            uinfo->setDestId(getMacNodeId());
                            uinfo->setFrameType(GRANTPKT);
                            uinfo->setTxNumber(1);
                            uinfo->setDirection(D2D_MULTI);
                            uinfo->setUserTxParams(preconfiguredTxParams_->dup());
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
            uinfo->setUserTxParams(preconfiguredTxParams_->dup());

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

void LteMacVUeMode4::finish()
{
    binder_->removeUeInfo(ueInfo_);

    delete preconfiguredTxParams_;
    delete ueInfo_;
}


