package v2x_controller_live;

import inet.applications.contract.IApp;

simple V2XStatusSenderApp like IApp
{
    parameters:
        @class(V2XStatusSenderApp);
        string destAddress = default("car[1]");
        int localPort = default(4000);
        int destPort = default(4000);
        double sendInterval @unit(s) = default(0.1s);
        double startTime @unit(s) = default(0.2s);
        int packetSizeBytes = default(300);
        @display("i=block/source");
    gates:
        input socketIn;
        output socketOut;
}

simple V2XStatusReceiverApp like IApp
{
    parameters:
        @class(V2XStatusReceiverApp);
        int localPort = default(4000);
        string packetLogFile = default("v2x_simu5g_delivered_packets.csv");
        @display("i=block/sink");
    gates:
        input socketIn;
        output socketOut;
}
