//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//

#include "corenetwork/deployer/LteDeployer.h"
#include "corenetwork/binder/LteBinder.h"
#include "stack/pdcp_rrc/layer/LtePdcpRrc.h"

Define_Module(LtePdcpRrcUe);
Define_Module(LtePdcpRrcEnb);
Define_Module(LtePdcpRrcRelayUe);
Define_Module(LtePdcpRrcRelayEnb);

LtePdcpRrcBase::LtePdcpRrcBase()
{
    ht_ = new ConnectionsTable();
    nonIpHt_ = new NonIpConnectionsTable();
    lcid_ = 1;
}

LtePdcpRrcBase::~LtePdcpRrcBase()
{
    delete ht_;
    delete nonIpHt_;

    PdcpEntities::iterator it = entities_.begin();
    for (; it != entities_.end(); ++it)
    {
        delete it->second;
    }
    entities_.clear();
}

void LtePdcpRrcBase::headerCompress(cPacket* pkt, int headerSize)
{
    if (headerCompressedSize_ != -1)
    {
        pkt->setByteLength(
            pkt->getByteLength() - headerSize + headerCompressedSize_);
        EV << "LtePdcp : Header compression performed\n";
    }
}

void LtePdcpRrcBase::headerDecompress(cPacket* pkt, int headerSize)
{
    if (headerCompressedSize_ != -1)
    {
        pkt->setByteLength(
            pkt->getByteLength() + headerSize - headerCompressedSize_);
        EV << "LtePdcp : Header decompression performed\n";
    }
}

        /*
         * TODO
         * Osservando le porte tira fuori:
         * lteInfo->setApplication();
         * lteInfo->setDirection();
         * lteInfo->setTraffic();
         * lteInfo->setRlcType();
         */
void LtePdcpRrcBase::setTrafficInformation(cPacket* pkt,
    LteControlInfo* lteInfo)
{
    if ((strcmp(pkt->getName(), "VoIP")) == 0)
    {
        lteInfo->setApplication(VOIP);
        lteInfo->setTraffic(CONVERSATIONAL);
        lteInfo->setRlcType((int) par("conversationalRlc"));
    }
    else if ((strcmp(pkt->getName(), "gaming")) == 0)
    {
        lteInfo->setApplication(GAMING);
        lteInfo->setTraffic(INTERACTIVE);
        lteInfo->setRlcType((int) par("interactiveRlc"));
    }
    else if ((strcmp(pkt->getName(), "VoDPacket") == 0)
        || (strcmp(pkt->getName(), "VoDFinishPacket") == 0))
    {
        lteInfo->setApplication(VOD);
        lteInfo->setTraffic(STREAMING);
        lteInfo->setRlcType((int) par("streamingRlc"));
    }
    else
    {
        lteInfo->setApplication(CBR);
        lteInfo->setTraffic(BACKGROUND);
        lteInfo->setRlcType((int) par("backgroundRlc"));
    }

    lteInfo->setDirection(getDirection());
}

/*
 * Upper Layer handlers
 */

void LtePdcpRrcBase::fromDataIn(cPacket *pkt)
{
    emit(receivedPacketFromUpperLayer, pkt);

    // Control Informations
    LteControlInfo* lteInfo = check_and_cast<LteControlInfo*>(pkt->removeControlInfo());
    LogicalCid mylcid;
    // get the PDCP entity for this LCID
    LtePdcpEntity* entity;

    setTrafficInformation(pkt, lteInfo);

    if (ipBased_)
    {
        FlowControlInfo* ipInfo = check_and_cast<FlowControlInfo*>(lteInfo);
        ipInfo->setDestId(getDestId(ipInfo));
        headerCompress(pkt, ipInfo->getHeaderSize()); // header compression

        // Cid Request
        EV << "LteRrc : Received CID request for Traffic [ " << "Source: "
           << IPv4Address(ipInfo->getSrcAddr()) << "@" << ipInfo->getSrcPort()
           << " Destination: " << IPv4Address(ipInfo->getDstAddr()) << "@"
           << ipInfo->getDstPort() << " ]\n";

        // TODO: Since IP addresses can change when we add and remove nodes, maybe node IDs should be used instead of them
        if ((mylcid = ht_->find_entry(ipInfo->getSrcAddr(), ipInfo->getDstAddr(),
            ipInfo->getSrcPort(), ipInfo->getDstPort())) == 0xFFFF)
        {
            // LCID not found
            mylcid = lcid_++;

            EV << "LteRrc : Connection not found, new CID created with LCID " << mylcid << "\n";

            ht_->create_entry(ipInfo->getSrcAddr(), ipInfo->getDstAddr(),
                ipInfo->getSrcPort(), ipInfo->getDstPort(), mylcid);
        }
        entity= getEntity(mylcid);

        // get the sequence number for this PDCP SDU.
        // Note that the numbering depends on the entity the packet is associated to.
        unsigned int sno = entity->nextSequenceNumber();

        // set sequence number
        ipInfo->setSequenceNumber(sno);
    }
    else
    {
        // NonIp flow
        FlowControlInfoNonIp* nonIpInfo = check_and_cast<FlowControlInfoNonIp*>(lteInfo);
        nonIpInfo->setDestId(getDestId(nonIpInfo));

        // Cid Request
        EV << "LteRrc : Received CID request for Traffic [ " << "Source: "
           << nonIpInfo->getSrcAddr() << " Destination: " << nonIpInfo->getDstAddr() << " ]\n";

        if ((mylcid = nonIpHt_->find_entry(nonIpInfo->getSrcAddr(), nonIpInfo->getDstAddr())) == 0xFFFF)
        {
            // LCID not found
            mylcid = lcid_++;

            EV << "LteRrc : Connection not found, new CID created with LCID " << mylcid << "\n";

            nonIpHt_->create_entry(nonIpInfo->getSrcAddr(), nonIpInfo->getDstAddr(), mylcid);
        }

        entity= getEntity(mylcid);

        // get the sequence number for this PDCP SDU.
        // Note that the numbering depends on the entity the packet is associated to.
        unsigned int sno = entity->nextSequenceNumber();

        // set sequence number
        nonIpInfo->setSequenceNumber(sno);
    }

    EV << "LteRrc : Assigned Lcid: " << mylcid << "\n";
    EV << "LteRrc : Assigned Node ID: " << nodeId_ << "\n";

    // NOTE setLcid and setSourceId have been anticipated for using in "ctrlInfoToMacCid" function
    lteInfo->setLcid(mylcid);
    lteInfo->setSourceId(nodeId_);

    // PDCP Packet creation
    LtePdcpPdu* pdcpPkt = new LtePdcpPdu("LtePdcpPdu");
    pdcpPkt->setByteLength(
        lteInfo->getRlcType() == UM ? PDCP_HEADER_UM : PDCP_HEADER_AM);
    pdcpPkt->encapsulate(pkt);

    EV << "LtePdcp : Preparing to send "
       << lteTrafficClassToA((LteTrafficClass) lteInfo->getTraffic())
       << " traffic\n";
    EV << "LtePdcp : Packet size " << pdcpPkt->getByteLength() << " Bytes\n";

    lteInfo->setSourceId(nodeId_);
    lteInfo->setLcid(mylcid);
    pdcpPkt->setControlInfo(lteInfo);

    EV << "LtePdcp : Sending packet " << pdcpPkt->getName() << " on port "
       << (lteInfo->getRlcType() == UM ? "UM_Sap$o\n" : "AM_Sap$o\n");

    // Send message
    send(pdcpPkt, (lteInfo->getRlcType() == UM ? umSap_[OUT] : amSap_[OUT]));
    emit(sentPacketToLowerLayer, pdcpPkt);
}

void LtePdcpRrcBase::fromEutranRrcSap(cPacket *pkt)
{
    // TODO For now use LCID 1000 for Control Traffic coming from RRC
    FlowControlInfo* lteInfo = new FlowControlInfo();
    lteInfo->setSourceId(nodeId_);
    lteInfo->setLcid(1000);
    lteInfo->setRlcType(TM);
    pkt->setControlInfo(lteInfo);
    EV << "LteRrc : Sending packet " << pkt->getName() << " on port TM_Sap$o\n";
    send(pkt, tmSap_[OUT]);
}

/*
 * Lower layer handlers
 */

void LtePdcpRrcBase::toDataOut(cPacket *pkt)
{
    emit(receivedPacketFromLowerLayer, pkt);

    LtePdcpPdu* pdcpPkt = check_and_cast<LtePdcpPdu*>(pkt);
    cPacket* upPkt;

    if (ipBased_)
    {
        FlowControlInfo* lteInfo = check_and_cast<FlowControlInfo*>(
            pdcpPkt->removeControlInfo());

        EV << "LtePdcp : Received packet with CID " << lteInfo->getLcid() << "\n";
        EV << "LtePdcp : Packet size " << pdcpPkt->getByteLength() << " Bytes\n";

        upPkt = pdcpPkt->decapsulate(); // Decapsulate packet
        delete pdcpPkt;

        headerDecompress(upPkt, lteInfo->getHeaderSize()); // Decompress packet header
        handleControlInfo(upPkt, lteInfo);
    }
    else
    {
        FlowControlInfoNonIp* lteInfo = check_and_cast<FlowControlInfoNonIp*>(
                pdcpPkt->removeControlInfo());
        EV << "LtePdcp : Received packet with CID " << lteInfo->getLcid() << "\n";
        EV << "LtePdcp : Packet size " << pdcpPkt->getByteLength() << " Bytes\n";

        upPkt = pdcpPkt->decapsulate(); // Decapsulate packet
        delete pdcpPkt;
        upPkt->setControlInfo(lteInfo);
    }

    EV << "LtePdcp : Sending packet " << upPkt->getName()
       << " on port DataOut\n";
    // Send message
    send(upPkt, dataOut_);
    emit(sentPacketToUpperLayer, upPkt);
}

void LtePdcpRrcBase::toEutranRrcSap(cPacket *pkt)
{
    cPacket* upPkt = pkt->decapsulate();
    delete pkt;

    EV << "LteRrc : Sending packet " << upPkt->getName()
       << " on port EUTRAN_RRC_Sap$o\n";
    send(upPkt, eutranRrcSap_[OUT]);
}

/*
 * Main functions
 */

void LtePdcpRrcBase::initialize(int stage)
{
    if (stage == inet::INITSTAGE_LOCAL)
    {
        dataIn_ = gate("DataIn");
        dataOut_ = gate("DataOut");
        eutranRrcSap_[IN] = gate("EUTRAN_RRC_Sap$i");
        eutranRrcSap_[OUT] = gate("EUTRAN_RRC_Sap$o");
        tmSap_[IN] = gate("TM_Sap$i");
        tmSap_[OUT] = gate("TM_Sap$o");
        umSap_[IN] = gate("UM_Sap$i");
        umSap_[OUT] = gate("UM_Sap$o");
        amSap_[IN] = gate("AM_Sap$i");
        amSap_[OUT] = gate("AM_Sap$o");

        binder_ = getBinder();
        headerCompressedSize_ = par("headerCompressedSize"); // Compressed size
        ipBased_ = par("ipBased");
        nodeId_ = getAncestorPar("macNodeId");

        // statistics

        pdcpdrop0_ = registerSignal("pdcpdrop0");
        pdcpdrop1_ = registerSignal("pdcpdrop1");
        pdcpdrop2_ = registerSignal("pdcpdrop2");
        pdcpdrop3_ = registerSignal("pdcpdrop3");
        receivedPacketFromUpperLayer = registerSignal("receivedPacketFromUpperLayer");
        receivedPacketFromLowerLayer = registerSignal("receivedPacketFromLowerLayer");
        sentPacketToUpperLayer = registerSignal("sentPacketToUpperLayer");
        sentPacketToLowerLayer = registerSignal("sentPacketToLowerLayer");

        three_hundred = 0;

        // TODO WATCH_MAP(gatemap_);
        WATCH(headerCompressedSize_);
        WATCH(nodeId_);
        WATCH(lcid_);

        setNodeType(par("nodeType").stdstringValue());

        // Moved here from IP2Lte to remove dependency on IP2Lte for simulations that are not IP based.
        if (nodeType_ == ENODEB)
        {
            // TODO not so elegant
            cModule *enodeb = getParentModule()->getParentModule();
            MacNodeId cellId = getBinder()->registerNode(enodeb, nodeType_);
            LteDeployer * deployer = check_and_cast<LteDeployer*>(enodeb->getSubmodule("deployer"));
            binder_->registerDeployer(deployer, cellId);
            nodeId_ = cellId;
        }
    }
}

void LtePdcpRrcBase::setNodeType(std::string s)
{
    nodeType_ = aToNodeType(s);
    EV << "Node type: " << s << " -> " << nodeType_ << endl;
}

void LtePdcpRrcBase::handleMessage(cMessage* msg)
{
    cPacket* pkt = check_and_cast<cPacket *>(msg);
    EV << "LtePdcp : Received packet " << pkt->getName() << " from port "
       << pkt->getArrivalGate()->getName() << endl;

    cGate* incoming = pkt->getArrivalGate();
    if (incoming == dataIn_)
    {
        fromDataIn(pkt);
    }
    else if (incoming == eutranRrcSap_[IN])
    {
        fromEutranRrcSap(pkt);
    }
    else if (incoming == tmSap_[IN])
    {
        toEutranRrcSap(pkt);
    }
    else
    {
        toDataOut(pkt);
    }
    return;
}

void LtePdcpRrcBase::setDrop(MacCid cid, unsigned int layer,
    double probability)
{
    Enter_Method("setDrop");

    dropMap_[cid].drop = true;
    dropMap_[cid].layer = layer;
    dropMap_[cid].probability = probability;
}

void LtePdcpRrcBase::clearDrop(MacCid cid)
{
    Enter_Method("clearDrop");
    dropMap_[cid].clear();
}

LtePdcpEntity* LtePdcpRrcBase::getEntity(LogicalCid lcid)
{
    // Find entity for this LCID
    PdcpEntities::iterator it = entities_.find(lcid);
    if (it == entities_.end())
    {
        // Not found: create
        LtePdcpEntity* ent = new LtePdcpEntity();
        entities_[lcid] = ent;    // Add to entities map

        EV << "LtePdcpRrcBase::getEntity - Added new PdcpEntity for Lcid: " << lcid << "\n";

        return ent;
    }
    else
    {
        // Found
        EV << "LtePdcpRrcBase::getEntity - Using old PdcpEntity for Lcid: " << lcid << "\n";

        return it->second;
    }

}

void LtePdcpRrcBase::finish()
{
    // TODO make-finish
}

void LtePdcpRrcEnb::initialize(int stage)
{
    LtePdcpRrcBase::initialize(stage);
    if (stage == inet::INITSTAGE_LOCAL)
        nodeId_ = getAncestorPar("macNodeId");
}

void LtePdcpRrcUe::initialize(int stage)
{
    LtePdcpRrcBase::initialize(stage);
    if (stage == inet::INITSTAGE_NETWORK_LAYER_3)
    {
        nodeId_ = getAncestorPar("macNodeId");
    }
}
