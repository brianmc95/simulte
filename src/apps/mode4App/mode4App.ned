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


package lte.apps.mode4App;
import lte.apps.mode4App.Mode4BaseApp;

simple Mode4App like Mode4BaseApp
{
    parameters:
        int packetSize = default(10); // Size of the actual packet itself
        int priority = default(3); // Priority of the packets sent
        int duration = default(1000); // MS before packet must be dropped
        double period @unit("s") = default(0.1s);
        int macNodeId = default(0);
        
        @signal[sentMsg];
        @statistic[sentMsg](title="Messages sent"; unit=""; source="sentMsg"; record=sum,vector);
        @signal[delay];
        @statistic[delay](title="Message Delay"; unit="s"; source="delay"; record=mean,vector);    
        @signal[rcvdMsg]; 
        @statistic[rcvdMsg](title="Messages Received"; unit="s"; source="rcvdMsg"; record=sum,vector);
        @signal[cbr];
        @statistic[cbr](title="Channel Busy Ratio"; unit=""; source="cbr"; record=vector);         
        @display("i=block/source");
    gates:
        input lowerGateIn; // FROM NIC
        output lowerGateOut;  // TO NIC
}
