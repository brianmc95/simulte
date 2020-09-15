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

#include "stack/phy/layer/LtePhyUeD2D.h"
#include "stack/phy/packet/SidelinkControlInformation_m.h"
#include "stack/mac/packet/LteSchedulingGrant.h"
#include "stack/mac/allocator/LteAllocationModule.h"
#include "stack/phy/layer/Subchannel.h"
#include <unordered_map>

class LtePhyVUeMode4 : public LtePhyUeD2D
{
  protected:

    // D2D Tx Power
    double d2dTxPower_;

    bool adjacencyPSCCHPSSCH_;
    int pStep_;
    int numSubchannels_;
    int subchannelSize_ ;
    int selectionWindowStartingSubframe_;
    int thresholdRSSI_;
    int cbrCountDown_;
    int sensingWindowSizeOverride_;

    bool transmitting_;
    bool randomScheduling_;
    bool oneShotMechanism_;
    bool oneShotSensing_;

    std::unordered_map<int, std::set<int>> oneShotCSRs_;

    int oneShotT2_;
    int oneShotT3_;
    int oneShotSubchannel_;
    simtime_t oneShotSignalTime_;
    simtime_t oneShotStartTime_;
    cMessage* sendOneShotSignal_;

    bool rssiFiltering_;
    bool rsrpFiltering_;

    std::map<MacNodeId, simtime_t> previousTransmissionTimes_;

    std::vector<int> ThresPSSCHRSRPvector_;

    std::vector<LteAirFrame*> tbFrames_; // airframes received in the current TTI. Only one will be decoded
    cMessage* d2dDecodingTimer_; // timer for triggering decoding at the end of the TTI. Started when the first airframe is received

    std::vector<std::vector<double>> tbRsrpVectors_;
    std::vector<std::vector<double>> tbRssiVectors_;
    std::vector<std::vector<double>> tbSinrVectors_;
    std::vector<double> tbAttenuations_;

    std::vector<std::vector<Subchannel*>> sensingWindow_;
    int sensingWindowFront_;
    LteMode4SchedulingGrant* sciGrant_;
    std::vector<std::vector<double>> sciRsrpVectors_;
    std::vector<std::vector<double>> sciRssiVectors_;
    std::vector<std::vector<double>> sciSinrVectors_;
    std::vector<double> sciAttenuations_;
    std::vector<LteAirFrame*> sciFrames_;
    std::vector<cPacket*> scis_;

    std::vector<std::vector<double>> oneShotRsrpVectors_;
    std::vector<std::vector<double>> oneShotRssiVectors_;
    std::vector<std::vector<double>> oneShotSinrVectors_;
    std::vector<double> oneShotAttenuations_;
    std::vector<LteAirFrame*> oneShotFrames_;

    // One Shot Stats

    simsignal_t oneShotSent;
    simsignal_t oneShotReceived;
    simsignal_t oneShotDecoded;
    simsignal_t oneShotFailedHalfDuplex;
    simsignal_t oneShotFailedDueToProp;
    simsignal_t oneShotFailedDueToInterference;
    simsignal_t oneShotUnsensed;
    simsignal_t txRxDistanceOneShot;
    int oneShotReceived_;
    int oneShotDecoded_;
    int oneShotFailedHalfDuplex_;
    int oneShotFailedDueToProp_;
    int oneShotFailedDueToInterference_;
    int oneShotUnsensed_;

    // SCI stats
    simsignal_t sciSent;

    simsignal_t sciReceived;
    simsignal_t sciDecoded;

    simsignal_t sciFailedHalfDuplex;
    simsignal_t sciFailedDueToProp;
    simsignal_t sciFailedDueToInterference;
    simsignal_t sciUnsensed;

    simsignal_t txRxDistanceSCI;

    int sciReceived_;
    int sciDecoded_;

    int sciFailedHalfDuplex_;
    int sciFailedDueToProp_;
    int sciFailedDueToInterference_;
    int sciUnsensed_;


    // Tb Stats
    simsignal_t tbSent;

    simsignal_t tbReceived;
    simsignal_t tbDecoded;

    simsignal_t tbFailedDueToNoSCI;
    simsignal_t tbFailedButSCIReceived;
    simsignal_t tbAndSCINotReceived;

    simsignal_t tbFailedHalfDuplex;
    simsignal_t tbFailedDueToProp;
    simsignal_t tbFailedDueToInterference;

    simsignal_t tbFailedDueToPropIgnoreSCI;
    simsignal_t tbFailedDueToInterferenceIgnoreSCI;
    simsignal_t tbDecodedIgnoreSCI;

    simsignal_t txRxDistanceTB;

    int tbReceived_;
    int tbDecoded_;
    int tbFailedDueToNoSCI_;
    int tbFailedButSCIReceived_;
    int tbAndSCINotReceived_;
    int tbFailedHalfDuplex_;

    int tbFailedDueToProp_;
    int tbFailedDueToInterference_;


    int tbFailedDueToPropIgnoreSCI_;
    int tbFailedDueToInterferenceIgnoreSCI_;
    int tbDecodedIgnoreSCI_;


    // General stats
    simsignal_t cbr;
    simsignal_t threshold;
    simsignal_t subchannelReceived;
    simsignal_t subchannelsUsed;
    simsignal_t senderID;
    simsignal_t subchannelSent;
    simsignal_t subchannelsUsedToSend;
    simsignal_t interPacketDelay;
    simsignal_t posX;
    simsignal_t posY;

    int subchannelReceived_;
    int subchannelsUsed_;

    RbMap availableRBs_;

    LteAllocationModule* allocator_;

    void storeAirFrame(LteAirFrame* newFrame);
    LteAirFrame* extractAirFrame();
    void decodeAirFrame(LteAirFrame* frame, UserControlInfo* lteInfo, std::vector<double> &rsrpVector, std::vector<double> &rssiVector, std::vector<double> &sinrVector, double &attenuation);
    // ---------------------------------------------------------------- //

    virtual void initialize(int stage);
    virtual void finish();
    virtual void handleAirFrame(cMessage* msg);
    virtual void handleUpperMessage(cMessage* msg);
    virtual void handleSelfMessage(cMessage *msg);

    // Helper function which prepares a frame for sending
    virtual LteAirFrame* prepareAirFrame(cMessage* msg, UserControlInfo* lteInfo);

    // Generate an SCI message corresponding to a Grant
    virtual SidelinkControlInformation* createSCIMessage();

    virtual RbMap sendSciMessage(cMessage* sci, UserControlInfo* lteInfo);

    virtual SidelinkControlInformation* createOneShotSCIMessage(int subframe, int subchannelIndex);

    virtual void sendOneShotMessage(UserControlInfo* lteInfo, int subframe, int subchannelIndex);

    // Compute Candidate Single Subframe Resources which the MAC layer can use for transmission
    virtual void computeCSRs(LteMode4SchedulingGrant* &grant);

    virtual void computeRandomCSRs(LteMode4SchedulingGrant* &grant);

    virtual void updateSubframe();

    virtual std::vector<std::tuple<double, int, int>> selectBestRSSIs(std::unordered_map<int, std::set<int>> possibleCSRs, LteMode4SchedulingGrant* &grant, int totalPossibleCSRs);

    virtual std::vector<std::tuple<double, int, int>> selectBestRSRPs(std::unordered_map<int, std::set<int>> possibleCSRs, LteMode4SchedulingGrant* &grant, int totalPossibleCSRs);

    virtual std::tuple<int,int> decodeRivValue(SidelinkControlInformation* sci, UserControlInfo* sciInfo);

    virtual void updateCBR();

    virtual void initialiseSensingWindow();

    virtual int translateIndex(int index);

  public:
    LtePhyVUeMode4();
    virtual ~LtePhyVUeMode4();

    virtual double getTxPwr(Direction dir = UNKNOWN_DIRECTION)
    {
        if (dir == D2D)
            return d2dTxPower_;
        return txPower_;
    }
};

