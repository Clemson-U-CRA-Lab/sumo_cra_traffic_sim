#include <omnetpp.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "inet/common/InitStages.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/packet/chunk/BytesChunk.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"

using namespace omnetpp;
using namespace inet;

class V2XNoiseSenderApp : public cSimpleModule
{
  private:
    UdpSocket socket;
    cMessage* sendTimer = nullptr;

    std::string destAddress;
    int localPort = 4500;
    int destPort = 4500;
    simtime_t sendInterval;
    simtime_t startTime;
    simtime_t stopTime;
    int packetSizeBytes = 1200;
    int seq = 0;

  private:
    void sendNoisePacket()
    {
        std::ostringstream payload;
        payload << "noise," << seq << "," << simTime().dbl();

        std::string payloadString = payload.str();

        auto payloadBytes = makeShared<BytesChunk>();
        std::vector<uint8_t> bytes(payloadString.begin(), payloadString.end());

        if ((int)bytes.size() < packetSizeBytes)
            bytes.resize(packetSizeBytes, 0);

        payloadBytes->setBytes(bytes);

        auto packet = new Packet("v2xNoise");
        packet->insertAtBack(payloadBytes);

        try {
            L3Address dest = L3AddressResolver().resolve(destAddress.c_str());
            socket.sendTo(packet, dest, destPort);
            seq++;
        }
        catch (const std::exception& e) {
            EV_WARN << "V2XNoiseSenderApp could not send packet: " << e.what() << "\n";
            delete packet;
        }
    }

  protected:
    virtual int numInitStages() const override
    {
        return NUM_INIT_STAGES;
    }

    virtual void initialize(int stage) override
    {
        if (stage == INITSTAGE_LOCAL) {
            destAddress = par("destAddress").stdstringValue();
            localPort = par("localPort").intValue();
            destPort = par("destPort").intValue();
            sendInterval = par("sendInterval");
            startTime = par("startTime");
            stopTime = par("stopTime");
            packetSizeBytes = par("packetSizeBytes").intValue();

            sendTimer = new cMessage("v2xNoiseSendTimer");
        }
        else if (stage == INITSTAGE_APPLICATION_LAYER) {
            socket.setOutputGate(gate("socketOut"));
            socket.bind(localPort);
            scheduleAt(startTime, sendTimer);
        }
    }

    virtual void handleMessage(cMessage* msg) override
    {
        if (msg == sendTimer) {
            if (simTime() <= stopTime) {
                sendNoisePacket();
                scheduleAt(simTime() + sendInterval, sendTimer);
            }
        }
        else {
            delete msg;
        }
    }

    virtual ~V2XNoiseSenderApp()
    {
        cancelAndDelete(sendTimer);
    }
};

class V2XNoiseReceiverApp : public cSimpleModule
{
  private:
    UdpSocket socket;
    std::ofstream noiseOut;
    int localPort = 4500;

  private:
    std::string bytesToString(Packet* packet)
    {
        auto bytesChunk = packet->peekDataAsBytes();
        const auto& bytes = bytesChunk->getBytes();

        std::string s;
        for (auto b : bytes) {
            if (b == 0)
                break;
            s.push_back(static_cast<char>(b));
        }

        return s;
    }

  protected:
    virtual int numInitStages() const override
    {
        return NUM_INIT_STAGES;
    }

    virtual void initialize(int stage) override
    {
        if (stage == INITSTAGE_LOCAL) {
            localPort = par("localPort").intValue();

            const char* logFile = par("noiseLogFile").stringValue();
            noiseOut.open(logFile, std::ios::out);

            if (!noiseOut.is_open())
                throw cRuntimeError("Could not open V2X noise packet log file: %s", logFile);

            noiseOut << "receive_time,seq,send_time,delay\n";
        }
        else if (stage == INITSTAGE_APPLICATION_LAYER) {
            socket.setOutputGate(gate("socketOut"));
            socket.bind(localPort);
        }
    }

    virtual void handleMessage(cMessage* msg) override
    {
        Packet* packet = dynamic_cast<Packet*>(msg);

        if (packet == nullptr) {
            delete msg;
            return;
        }

        std::string payload = bytesToString(packet);

        std::stringstream ss(payload);
        std::string label;
        std::string seqStr;
        std::string sendTimeStr;

        std::getline(ss, label, ',');
        std::getline(ss, seqStr, ',');
        std::getline(ss, sendTimeStr, ',');

        if (label == "noise" && !seqStr.empty() && !sendTimeStr.empty()) {
            int seq = std::stoi(seqStr);
            double sendTime = std::stod(sendTimeStr);
            double recvTime = simTime().dbl();

            noiseOut << recvTime << ","
                     << seq << ","
                     << sendTime << ","
                     << recvTime - sendTime << "\n";
            noiseOut.flush();
        }

        delete packet;
    }

    virtual void finish() override
    {
        if (noiseOut.is_open())
            noiseOut.close();
    }
};

Define_Module(V2XNoiseSenderApp);
Define_Module(V2XNoiseReceiverApp);
