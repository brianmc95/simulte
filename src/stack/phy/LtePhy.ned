// 
//                           SimuLTE
// 
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself, 
// and cannot be removed from it.
// 


package lte.stack.phy;

// 
// Interface for the LtePhy module of PHY Layer
//
moduleinterface LtePhy {
    parameters:
        @display("i=block/wrxtx");
        bool usePropagationDelay;
        xml channelModel;
        
    gates:
        input upperGateIn;
        output upperGateOut;
        input radioIn;
}

// 
// Base LtePhy module of PHY Layer
//
simple LtePhyBase like LtePhy {
    parameters:
        @display("i=block/wrxtx");
        bool usePropagationDelay = default(true);                     // Should transmission delay be simulated?
        xml channelModel;
        double ueTxPower =default(26);
        double eNodeBTxPower =default(46);
        double relayTxPower =default(20);
        double microTxPower =default(30);
        string txDirection = default("OMNI");
        double txAngle = default(0);       
        
        bool dynamicCellAssociation = default(false);
        
        // switch for handover messages handling on UEs
        bool enableHandover = default(false);
        double handoverLatency @unit(s) = default(0.05s);
               
    gates:
        input upperGateIn;       // from upper layer
        output upperGateOut;    // to upper layer
        input radioIn;           // gate to receive message via sendDirect
}

// 
// User Equipment LtePhy module of PHY Layer
//
simple LtePhyUe extends LtePhyBase {
     parameters:
         @class("LtePhyUe");
         
         @signal[servingCell];
         @statistic[servingCell](title="ID of the serving eNodeB for the UE"; unit=""; source="servingCell"; record=vector);
        
         //# CQI statistics
         @signal[averageCqiDl];
         @statistic[averageCqiDl](title="Average Cqi reported in DL"; unit="cqi"; source="averageCqiDl"; record=mean,vector);
         @signal[averageCqiUl];
         @statistic[averageCqiUl](title="Average Cqi reported in UL"; unit="cqi"; source="averageCqiUl"; record=mean,vector);
}

// 
// D2D-capable User Equipment LtePhy module of PHY Layer
//
simple LtePhyUeD2D extends LtePhyUe {
     parameters:
         @class("LtePhyUeD2D");
         double d2dTxPower =default(23);
         bool d2dMulticastCaptureEffect = default(true);
         string d2dMulticastCaptureEffectFactor = default("RSRP");  // or distance
         
         //# D2D CQI statistic
         @signal[averageCqiD2D];
         @statistic[averageCqiD2D](title="Average Cqi reported in D2D"; unit="cqi"; source="averageCqiD2D"; record=mean,vector);
}

simple LtePhyVUeMode4 extends LtePhyUeD2D {
	parameters:   
	    @class("LtePhyVUeMode4");
	    
	    bool adjacencyPSCCHPSSCH = default(true);
	    bool rssiFiltering = default(true);
	    bool rsrpFiltering = default(false);
	    int pStep = default(100);
        int numSubchannels = default(10);
	    int subchannelSize = default(5);
	    int selectionWindowStartingSubframe = default(1);
	    int shapeFactor = default(6);
	    int thresholdRSSI = default(22);
	    int sensingWindowSizeOverride = default(-1);

	    @signal[cbr];
 		@statistic[cbr](title="Channel Busy Ratio"; source="cbr"; record=mean,vector);
 		@signal[sciReceived];
		@statistic[sciReceived](title="Number of received sci"; source="sciReceived"; record=sum,vector);
		@signal[sciDecoded];
		@statistic[sciDecoded](title="Number of sci successfully decoded"; source="sciDecoded"; record=sum,vector);
		@signal[sciNotDecoded];
        @statistic[sciNotDecoded](title="Number of sci successfully decoded"; source="sciNotDecoded"; record=sum,vector);

		@signal[sciSent];
		@statistic[sciSent](title="Number of sci sent"; source="sciSent"; record=sum,vector);
		@signal[tbSent];
		@statistic[tbSent](title="Number of tb sent"; source="tbSent"; record=sum,vector);
		@signal[subchannelSent];
		@statistic[subchannelSent](title="starting Subchannel used for sent message"; source="subchannelSent"; record=sum,vector);
		@signal[subchannelsUsedToSend];
		@statistic[subchannelsUsedToSend](title="Number of subchannels used for message"; source="subchannelsUsedToSend"; record=sum,vector);

		@signal[tbReceived];
		@statistic[tbReceived](title="Number of tb received"; source="tbReceived"; record=sum,vector);
		@signal[tbDecoded];
		@statistic[tbDecoded](title="Number of tb Decoded"; source="tbDecoded"; record=sum,vector);
		@signal[tbFailedDueToNoSCI];
		@statistic[tbFailedDueToNoSCI](title="Number of tb not decoded due to no SCI"; source="tbFailedDueToNoSCI"; record=sum,vector);
		@signal[tbFailedButSCIReceived];
		@statistic[tbFailedButSCIReceived](title="Number of failed TBs with a decoded SCI"; source="tbFailedButSCIReceived"; record=sum,vector);
		@signal[tbAndSCINotReceived];
		@statistic[tbAndSCINotReceived](title="Number of failed TBs with a failed SCI"; source="tbAndSCINotReceived"; record=sum,vector);
		@signal[threshold];
		@statistic[threshold](title="Increase in RSRP needed to find csr"; source="threshold"; record=mean,vector);

		@signal[senderID];
		@statistic[senderID](title="ID of node who sent message"; source="senderID"; record=vector);

        @signal[sciFailedHalfDuplex];
        @statistic[sciFailedHalfDuplex](title="Number of failed SCIs due to Half Duplex"; source="sciFailedHalfDuplex"; record=sum,vector);
        @signal[tbFailedHalfDuplex];
        @statistic[tbFailedHalfDuplex](title="Number of failed TBs due to Half Duplex"; source="tbFailedHalfDuplex"; record=sum,vector);

        @signal[tbFailedDueToProp];
        @statistic[tbFailedDueToProp](title="Number of failed TBs due to propogation"; source="tbFailedDueToProp"; record=sum,vector);
        @signal[tbFailedDueToInterference];
        @statistic[tbFailedDueToInterference](title="Number of failed TBs due to interference"; source="tbFailedDueToInterference"; record=sum,vector);
        @signal[sciFailedDueToProp];
        @statistic[sciFailedDueToProp](title="Number of failed SCIs due to propogation"; source="sciFailedDueToProp"; record=sum,vector);
        @signal[sciFailedDueToInterference];
        @statistic[sciFailedDueToInterference](title="Number of failed SCIs due to interference"; source="sciFailedDueToInterference"; record=sum,vector);

        @signal[sciUnsensed];
        @statistic[sciUnsensed](title="Number of failed SCIs due to RSRP below threshold"; source="sciUnsensed"; record=sum,vector);

		@signal[txRxDistanceSCI];
		@statistic[txRxDistanceSCI](title="Distance between transmitter and receiver"; source="txRxDistanceSCI"; record=mean,vector);
		@signal[txRxDistanceTB];
        @statistic[txRxDistanceTB](title="Distance between transmitter and receiver"; source="txRxDistanceTB"; record=mean,vector);

        @signal[subchannelReceived];
        @statistic[subchannelReceived](title="Initial Subchannel on which packet is received"; source="subchannelReceived"; record=mean,vector);
        @signal[subchannelsUsed];
        @statistic[subchannelsUsed](title="Number of Subchannels used for the received packet"; source="subchannelsUsed"; record=mean, vector);

        @signal[interPacketDelay];
        @statistic[interPacketDelay](title="Delay between packet receptions from a node"; source="interPacketDelay"; record=mean,vector);
        @signal[posX];
        @statistic[posX](title="X position of node (at reception time)"; source="posX"; record=mean, vector);
        @signal[posY];
        @statistic[posY](title="Y position of node (at reception time)"; source="posY"; record=mean, vector);

        @signal[tbFailedDueToPropIgnoreSCI];
        @statistic[tbFailedDueToPropIgnoreSCI](title="Number of failed TBs due to propogation"; source="tbFailedDueToPropIgnoreSCI"; record=sum,vector);
        @signal[tbFailedDueToInterferenceIgnoreSCI];
        @statistic[tbFailedDueToInterferenceIgnoreSCI](title="Number of failed TBs due to interference"; source="tbFailedDueToInterferenceIgnoreSCI"; record=sum,vector);
        @signal[tbDecodedIgnoreSCI];
        @statistic[tbDecodedIgnoreSCI](title="Number of failed TBs due to interference"; source="tbDecodedIgnoreSCI"; record=sum,vector);
<<<<<<< HEAD
        
        @signal[tbFailedDueToCol];
        @statistic[tbFailedDueToCol](title="Number of failed TBs due to radio resource collision"; source="tbFailedDueToCol"; record=sum,vector);
        
=======
>>>>>>> 51d6febab625bfdb30015c417012510f02f21c46
}

// 
// eNodeB LtePhy module of PHY Layer
//
simple LtePhyEnb extends LtePhyBase {
    @class("LtePhyEnb");
    xml feedbackComputation;
}

// 
// eNodeB LtePhy module of PHY Layer with support for D2D
//
simple LtePhyEnbD2D extends LtePhyEnb {
    parameters:
        @class("LtePhyEnbD2D");
        bool enableD2DCqiReporting = default(false);
}

// 
// Relay LtePhy module of PHY Layer
//
simple LtePhyRelay extends LtePhyBase {
    @class("LtePhyRelay");
}


