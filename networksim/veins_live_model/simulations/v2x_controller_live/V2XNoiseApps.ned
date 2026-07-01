package v2x_controller_live;

import inet.applications.contract.IApp;

simple V2XNoiseSenderApp like IApp
{
    parameters:
        @class(V2XNoiseSenderApp);
        string destAddress = default("car[1]");
        int localPort = default(4500);
        int destPort = default(4500);
        double sendInterval @unit(s) = default(0.002s);
        double startTime @unit(s) = default(5s);
        double stopTime @unit(s) = default(10s);
        int packetSizeBytes = default(1200);
        @display("i=block/source");
    gates:
        input socketIn;
        output socketOut;
}

simple V2XNoiseReceiverApp like IApp
{
    parameters:
        @class(V2XNoiseReceiverApp);
        int localPort = default(4500);
        string noiseLogFile = default("v2x_simu5g_noise_packets.csv");
        @display("i=block/sink");
    gates:
        input socketIn;
        output socketOut;
}
