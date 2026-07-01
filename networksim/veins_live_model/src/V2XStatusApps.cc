#include <omnetpp.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

#include "inet/common/InitStages.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/packet/chunk/BytesChunk.h"
#include "inet/mobility/contract/IMobility.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"

#include "V2XStatusRegistry.h"

using namespace omnetpp;
using namespace inet;

namespace v2x_controller_live {

static V2XStatusPayload latestStatus;

void updateLatestStatus(const V2XStatusPayload& payload)
{
    latestStatus = payload;
    latestStatus.valid = true;
}

bool hasLatestStatus()
{
    return latestStatus.valid;
}

V2XStatusPayload getLatestStatus()
{
    return latestStatus;
}

}

class V2XStatusSenderApp : public cSimpleModule
{
  private:
    UdpSocket socket;
    cMessage* sendTimer = nullptr;

    std::string destAddress;
    int localPort = 4000;
    int destPort = 4000;
    simtime_t sendInterval;
    simtime_t startTime;
    int packetSizeBytes = 300;
    int seq = 0;

    struct LastState {
        double time = 0.0;
        double x = 0.0;
        double y = 0.0;
        double speed = 0.0;
        bool valid = false;
    };

    LastState last;

  private:
    IMobility* findMobility(cModule* module)
    {
        if (module == nullptr)
            return nullptr;

        if (auto mobility = dynamic_cast<IMobility*>(module))
            return mobility;

        for (cModule::SubmoduleIterator it(module); !it.end(); ++it) {
            cModule* child = *it;
            if (auto mobility = findMobility(child))
                return mobility;
        }

        return nullptr;
    }

    std::string buildPayload()
    {
        cModule* car = getParentModule();
        IMobility* mobility = findMobility(car);

        double now = simTime().dbl();
        double x = 0.0;
        double y = 0.0;
        double speed = 0.0;
        double accel = 0.0;

        if (mobility != nullptr) {
            auto pos = mobility->getCurrentPosition();
            x = pos.x;
            y = pos.y;

            if (last.valid) {
                double dt = now - last.time;
                if (dt > 0.0) {
                    double dx = x - last.x;
                    double dy = y - last.y;
                    speed = std::sqrt(dx * dx + dy * dy) / dt;
                    accel = (speed - last.speed) / dt;
                }
            }

            last.time = now;
            last.x = x;
            last.y = y;
            last.speed = speed;
            last.valid = true;
        }

        std::ostringstream payload;
        payload << seq << ","
                << now << ","
                << x << ","
                << y << ","
                << speed << ","
                << accel;

        seq++;
        return payload.str();
    }

    void sendStatusPacket()
    {
        std::string payloadString = buildPayload();

        auto payloadBytes = makeShared<BytesChunk>();
        std::vector<uint8_t> bytes(payloadString.begin(), payloadString.end());

        if ((int)bytes.size() < packetSizeBytes)
            bytes.resize(packetSizeBytes, 0);

        payloadBytes->setBytes(bytes);

        auto packet = new Packet("v2xStatus");
        packet->insertAtBack(payloadBytes);

        try {
            L3Address dest = L3AddressResolver().resolve(destAddress.c_str());
            socket.sendTo(packet, dest, destPort);
        }
        catch (const std::exception& e) {
            EV_WARN << "V2XStatusSenderApp could not send packet: " << e.what() << "\n";
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
            packetSizeBytes = par("packetSizeBytes").intValue();

            sendTimer = new cMessage("v2xStatusSendTimer");
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
            sendStatusPacket();
            scheduleAt(simTime() + sendInterval, sendTimer);
        }
        else {
            delete msg;
        }
    }

    virtual ~V2XStatusSenderApp()
    {
        cancelAndDelete(sendTimer);
    }
};

class V2XStatusReceiverApp : public cSimpleModule
{
  private:
    UdpSocket socket;
    std::ofstream packetOut;
    int localPort = 4000;

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

    bool parsePayload(const std::string& payload, v2x_controller_live::V2XStatusPayload& parsed)
    {
        std::stringstream ss(payload);
        std::string item;
        std::vector<std::string> fields;

        while (std::getline(ss, item, ','))
            fields.push_back(item);

        if (fields.size() < 6)
            return false;

        parsed.seq = std::stoi(fields[0]);
        parsed.sendTime = std::stod(fields[1]);
        parsed.receiveTime = simTime().dbl();
        parsed.x = std::stod(fields[2]);
        parsed.y = std::stod(fields[3]);
        parsed.speed = std::stod(fields[4]);
        parsed.accel = std::stod(fields[5]);
        parsed.valid = true;

        return true;
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

            const char* logFile = par("packetLogFile").stringValue();
            packetOut.open(logFile, std::ios::out);

            if (!packetOut.is_open())
                throw cRuntimeError("Could not open V2X delivered packet log file: %s", logFile);

            packetOut << "receive_time,seq,send_time,delay,x,y,speed,accel\n";
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

        v2x_controller_live::V2XStatusPayload parsed;
        bool ok = parsePayload(payload, parsed);

        if (ok) {
            v2x_controller_live::updateLatestStatus(parsed);

            packetOut << parsed.receiveTime << ","
                      << parsed.seq << ","
                      << parsed.sendTime << ","
                      << parsed.receiveTime - parsed.sendTime << ","
                      << parsed.x << ","
                      << parsed.y << ","
                      << parsed.speed << ","
                      << parsed.accel << "\n";
            packetOut.flush();
        }

        delete packet;
    }

    virtual void finish() override
    {
        if (packetOut.is_open())
            packetOut.close();
    }
};

Define_Module(V2XStatusSenderApp);
Define_Module(V2XStatusReceiverApp);
