// 
//                           SimuLTE
// 
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself, 
// and cannot be removed from it.
// 


package lte.corenetwork.nodes.cars;

import inet.mobility.contract.IMobility;
import lte.stack.phy.ILteNic;
import lte.stack.phy.LteNicUe;
import lte.stack.phy.LteNicUeD2D;
import lte.stack.phy.LteNicVUeMode4;
import lte.apps.mode4App.*;

// 
// Car Module
//
module CarNonIp
{
    parameters:
        @networkNode();
        @display("i=car.png;bgb=400,518");

        //# Mobility
        string mobilityType = default("VeinsInetMobility");

        //# Node specs
        string nodeType = "UE";  // DO NOT CHANGE
        int masterId = default(0); // Not yet removed from SimuLTE for Mode 4 simulations
        int macNodeId = default(0); // TODO: this is not a real parameter
        int macCellId = default(0); // TODO: this is not a real parameter
        
        string applType; //type of the application layer

        //# D2D capability
        bool d2dCapable = default(false);
        bool mode4D2D = default(false);
        
        // Check if the car is D2D, if it is further check if it's mode4, otherwise it's a simple Ue
        string nicType = d2dCapable ? mode4D2D ? "LteNicVUeMode4" : "LteNicUeD2D" : "LteNicUe";

    gates:
        input radioIn @directIn;     // connection to master

    submodules:
        mobility: <mobilityType> like IMobility {
            @display("p=50,175;is=s");
        }
        
        appl: <applType> like lte.apps.mode4App.Mode4BaseApp {
            parameters:
                @display("p=60,50");
        }
        
        // NOTE: instance must be named "lteNic"
        lteNic: <nicType> like ILteNic {
            nodeType = nodeType;
            d2dCapable = d2dCapable;
            @display("p=250,407");
        }
    connections allowunconnected:
        //# 
        //# Internal TCP/UDP applications connections with IP stack
        //#

        lteNic.upperLayerOut --> appl.lowerGateIn;
        lteNic.upperLayerIn <-- appl.lowerGateOut;

        lteNic.radioIn <-- radioIn;
}
