//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//

#ifndef _LTE_LTEDUMMYCHANNELMODEL_H_
#define _LTE_LTEDUMMYCHANNELMODEL_H_

#include "stack/phy/ChannelModel/LteChannelModel.h"

class LteDummyChannelModel : public LteChannelModel
{
  protected:
    double per_;
    double harqReduction_;
    public:
    LteDummyChannelModel(ParameterMap& params, int band);
    virtual ~LteDummyChannelModel();
    /*
     * Compute the error probability of the transmitted packet
     *
     * @param frame pointer to the packet
     * @param lteinfo pointer to the user control info
     */
    virtual bool error(LteAirFrame *frame, UserControlInfo* lteInfo);
    /*
     * Compute Attenuation caused by pathloss and shadowing (optional)
     */
    virtual double getAttenuation(MacNodeId nodeId, Direction dir, inet::Coord coord)
    {
        return 0;
    }
    /*
     * Compute FAKE sir for each band for user nodeId according to multipath fading
     *
     * @param frame pointer to the packet
     * @param lteinfo pointer to the user control info
     */
    virtual std::vector<double> getSIR(LteAirFrame *frame, UserControlInfo* lteInfo);
    /*
     * Compute FAKE sinr for each band for user nodeId according to pathloss, shadowing (optional) and multipath fading
     *
     * @param frame pointer to the packet
     * @param lteinfo pointer to the user control info
     */
    virtual std::vector<double> getSINR(LteAirFrame *frame, UserControlInfo* lteInfo);
    /*
     * Compute the error probability of the transmitted packet according to cqi used, txmode, and the received power
     * after that it throws a random number in order to check if this packet will be corrupted or not
     *
     * @param frame pointer to the packet
     * @param lteinfo pointer to the user control info
     * @param rsrpVector the received signal for each RB, if it has already been computed
     */
    virtual bool error_D2D(LteAirFrame *frame, UserControlInfo* lteInfo, std::vector<double> rsrpVector);
    /*
     * Compute the error probability of the transmitted packet according to mcs used, txmode, and the received power
     * after that it throws a random number in order to check if this packet will be corrupted or not
     *
     * @param frame pointer to the packet
     * @param lteinfo pointer to the user control info
     * @param rsrpVector the received signal for each RB, if it has already been computed
     * @param mcs the modulation and coding scheme used in sending the message.
     * @returns a tuple specifying whether it was successfully received based on SNR and SINR
     */
    virtual std::tuple<bool, bool> error_Mode4(LteAirFrame *frame, UserControlInfo* lteInfo, std::vector<double> rsrpVector, std::vector<double> sinrVector, int mcs);
    /*
     * Compute Received useful signal for D2D transmissions
     */
    virtual std::tuple<std::vector<double>, double> getRSRP_D2D(LteAirFrame *frame, UserControlInfo* lteInfo_1, MacNodeId destId, inet::Coord destCoord);
    /*
     * Compute FAKE SINR (D2D) for each band for user nodeId according to pathloss, shadowing (optional) and multipath fading
     *
     * @param frame pointer to the packet
     * @param lteinfo pointer to the user control info
     */
    virtual std::vector<double> getSINR_D2D(LteAirFrame *frame, UserControlInfo* lteInfo_1, MacNodeId destId, inet::Coord destCoord,MacNodeId enbId);
    virtual std::vector<double> getSINR_D2D(LteAirFrame *frame, UserControlInfo* lteInfo_1, MacNodeId destId, inet::Coord destCoord,MacNodeId enbId,std::vector<double> rsrpVector, bool interference);
    /*
     * Compute FAKE RSSI (D2D) for each band for user nodeId according to pathloss, shadowing (optional) and multipath fading
     *
     * @param frame pointer to the packet
     * @param lteinfo pointer to the user control info
     */
    virtual std::tuple<std::vector<double>, std::vector<double>> getRSSI_SINR(LteAirFrame *frame, UserControlInfo* lteInfo_1, MacNodeId destId, inet::Coord destCoord,MacNodeId enbId,std::vector<double> rsrpVector);

    //TODO
    virtual bool errorDas(LteAirFrame *frame, UserControlInfo* lteI)
    {
        throw cRuntimeError("DAS PHY LAYER TO BE IMPLEMENTED");
        return false;
    }

    virtual double getTxRxDistance(UserControlInfo* lteInfo);
};

#endif
