#include <omnetpp.h>
#include <fstream>
#include <string>
#include <map>
#include <vector>
#include <deque>
#include <cmath>
#include <sstream>
#include <random>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "inet/mobility/contract/IMobility.h"
#include "veins_inet/VeinsInetMobility.h"
#include "V2XStatusRegistry.h"

using namespace omnetpp;

class V2XLiveStateLogger : public cSimpleModule
{
  private:
    struct LastState {
        double time = 0.0;
        double x = 0.0;
        double y = 0.0;
        double speed = 0.0;
        bool valid = false;
    };

    struct VehicleState {
        std::string moduleName;
        double x = 0.0;
        double y = 0.0;
        double speed = 0.0;
        double accel = 0.0;
        double sourceSendTime = 0.0;
    };

    struct PendingNv0Message {
        int messageId = 0;
        double sendTime = 0.0;
        double deliveryTime = 0.0;
        double delay = 0.0;
        VehicleState state;
    };

    cMessage* tick = nullptr;

    std::ofstream stateOut;
    std::ofstream commandOut;
    std::ofstream networkOut;

    simtime_t interval;

    bool controllerBridgeEnabled = false;
    std::string controllerHost;
    int controllerPort = 5555;

    bool useSimu5gDeliveredNv0 = false;
    bool networkDegradationEnabled = false;
    simtime_t networkBaseDelay;
    simtime_t networkAttackStart;
    simtime_t networkAttackEnd;
    simtime_t networkAttackDelay;
    simtime_t networkAttackJitter;
    double networkAttackDropProbability = 0.0;

    int nextMessageId = 0;
    bool hasDeliveredNv0 = false;
    VehicleState latestDeliveredNv0;

    std::deque<PendingNv0Message> pendingNv0Messages;
    std::map<int, LastState> lastByModuleId;

    std::mt19937 rng;
    std::uniform_real_distribution<double> unitDist;
    std::uniform_real_distribution<double> jitterDist;

  private:
    inet::IMobility* findMobility(cModule* module)
    {
        if (module == nullptr)
            return nullptr;

        if (auto mobility = dynamic_cast<inet::IMobility*>(module))
            return mobility;

        for (cModule::SubmoduleIterator it(module); !it.end(); ++it) {
            cModule* child = *it;
            if (auto mobility = findMobility(child))
                return mobility;
        }

        return nullptr;
    }

    veins::VeinsInetMobility* findVeinsInetMobility(cModule* module)
    {
        if (module == nullptr)
            return nullptr;

        if (auto mobility = dynamic_cast<veins::VeinsInetMobility*>(module))
            return mobility;

        for (cModule::SubmoduleIterator it(module); !it.end(); ++it) {
            cModule* child = *it;
            if (auto mobility = findVeinsInetMobility(child))
                return mobility;
        }

        return nullptr;
    }

    bool applySpeedCommandToNv1(cModule* nv1Car, double currentSpeed, double accCmd)
    {
        auto mobility = findVeinsInetMobility(nv1Car);

        if (mobility == nullptr) {
            EV_WARN << "Could not find VeinsInetMobility for nv1 car module\n";
            return false;
        }

        double targetSpeed = currentSpeed + accCmd * interval.dbl();

        if (targetSpeed < 0.0)
            targetSpeed = 0.0;

        try {
            mobility->getVehicleCommandInterface()->setSpeedMode(96);
            mobility->getVehicleCommandInterface()->setSpeed(targetSpeed);
            return true;
        }
        catch (const std::exception& e) {
            EV_WARN << "Failed to apply speed command to nv1: " << e.what() << "\n";
            return false;
        }
    }

    bool queryPythonController(const VehicleState& nv0, const VehicleState& nv1, double& accCmd)
    {
        int sock = ::socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0)
            return false;

        timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        ::setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

        sockaddr_in serverAddr;
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(controllerPort);

        if (::inet_pton(AF_INET, controllerHost.c_str(), &serverAddr.sin_addr) <= 0) {
            ::close(sock);
            return false;
        }

        if (::connect(sock, (sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
            ::close(sock);
            return false;
        }

        std::ostringstream request;
        request << simTime().dbl() << ","
                << nv0.accel << "," << nv0.speed << "," << nv0.x << ","
                << nv1.accel << "," << nv1.speed << "," << nv1.x << "\n";

        std::string requestString = request.str();

        ssize_t sent = ::send(sock, requestString.c_str(), requestString.size(), 0);
        if (sent < 0) {
            ::close(sock);
            return false;
        }

        char buffer[256];
        ssize_t received = ::recv(sock, buffer, sizeof(buffer) - 1, 0);

        if (received <= 0) {
            ::close(sock);
            return false;
        }

        buffer[received] = '\0';

        try {
            accCmd = std::stod(std::string(buffer));
        }
        catch (...) {
            ::close(sock);
            return false;
        }

        ::close(sock);
        return true;
    }

    void enqueueNv0NetworkMessage(const VehicleState& directNv0)
    {
        double now = simTime().dbl();

        bool inAttackWindow =
            now >= networkAttackStart.dbl() && now <= networkAttackEnd.dbl();

        double delay = networkBaseDelay.dbl();
        bool dropped = false;

        if (networkDegradationEnabled && inAttackWindow) {
            delay += networkAttackDelay.dbl();
            delay += jitterDist(rng);

            if (delay < 0.0)
                delay = 0.0;

            dropped = unitDist(rng) < networkAttackDropProbability;
        }

        int msgId = nextMessageId++;

        networkOut << msgId << ","
                   << now << ","
                   << now + delay << ","
                   << delay << ","
                   << inAttackWindow << ","
                   << dropped << ","
                   << directNv0.x << ","
                   << directNv0.speed << ","
                   << directNv0.accel << "\n";
        networkOut.flush();

        if (!dropped) {
            PendingNv0Message msg;
            msg.messageId = msgId;
            msg.sendTime = now;
            msg.deliveryTime = now + delay;
            msg.delay = delay;
            msg.state = directNv0;
            msg.state.sourceSendTime = now;

            pendingNv0Messages.push_back(msg);
        }
    }

    void deliverAvailableNv0Messages()
    {
        double now = simTime().dbl();

        while (!pendingNv0Messages.empty() && pendingNv0Messages.front().deliveryTime <= now) {
            latestDeliveredNv0 = pendingNv0Messages.front().state;
            hasDeliveredNv0 = true;
            pendingNv0Messages.pop_front();
        }
    }

    void logVehicleStates()
    {
        cModule* network = getParentModule();
        if (network == nullptr)
            return;

        int carVectorSize = network->getSubmoduleVectorSize("car");
        std::vector<VehicleState> states;

        for (int i = 0; i < carVectorSize; ++i) {
            cModule* car = network->getSubmodule("car", i);
            if (car == nullptr)
                continue;

            inet::IMobility* mobility = findMobility(car);
            if (mobility == nullptr)
                continue;

            auto pos = mobility->getCurrentPosition();

            int id = car->getId();
            double now = simTime().dbl();

            double speed = 0.0;
            double accel = 0.0;

            auto& last = lastByModuleId[id];

            if (last.valid) {
                double dt = now - last.time;

                if (dt > 0.0) {
                    double dx = pos.x - last.x;
                    double dy = pos.y - last.y;

                    speed = std::sqrt(dx * dx + dy * dy) / dt;
                    accel = (speed - last.speed) / dt;
                }
            }

            VehicleState state;
            state.moduleName = car->getFullName();
            state.x = pos.x;
            state.y = pos.y;
            state.speed = speed;
            state.accel = accel;
            state.sourceSendTime = now;

            states.push_back(state);

            stateOut << now << ","
                     << car->getFullName() << ","
                     << pos.x << ","
                     << pos.y << ","
                     << speed << ","
                     << accel << "\n";

            last.time = now;
            last.x = pos.x;
            last.y = pos.y;
            last.speed = speed;
            last.valid = true;
        }

        stateOut.flush();

        if (states.size() < 2)
            return;

        const VehicleState& directNv0 = states[0];
        const VehicleState& nv1 = states[1];

        VehicleState controllerNv0 = directNv0;
        std::string nv0Source = "direct";

        if (useSimu5gDeliveredNv0) {
            if (v2x_controller_live::hasLatestStatus()) {
                auto pkt = v2x_controller_live::getLatestStatus();

                controllerNv0.moduleName = "simu5g_packet_nv0";
                controllerNv0.x = pkt.x;
                controllerNv0.y = pkt.y;
                controllerNv0.speed = pkt.speed;
                controllerNv0.accel = pkt.accel;
                controllerNv0.sourceSendTime = pkt.sendTime;

                nv0Source = "simu5g_packet";
            }
        }
        else {
            enqueueNv0NetworkMessage(directNv0);
            deliverAvailableNv0Messages();

            if (networkDegradationEnabled && hasDeliveredNv0) {
                controllerNv0 = latestDeliveredNv0;
                nv0Source = "delivered";
            }
        }

        if (!controllerBridgeEnabled)
            return;

        double accCmd = 0.0;
        bool controllerOk = queryPythonController(controllerNv0, nv1, accCmd);

        bool actuationOk = false;
        double targetSpeed = nv1.speed;

        if (controllerOk) {
            targetSpeed = nv1.speed + accCmd * interval.dbl();

            if (targetSpeed < 0.0)
                targetSpeed = 0.0;

            cModule* nv1Car = network->getSubmodule("car", 1);
            actuationOk = applySpeedCommandToNv1(nv1Car, nv1.speed, accCmd);
        }

        double nv0InfoAge = simTime().dbl() - controllerNv0.sourceSendTime;

        commandOut << simTime().dbl() << ","
                   << nv0Source << ","
                   << directNv0.moduleName << ","
                   << directNv0.x << ","
                   << directNv0.speed << ","
                   << directNv0.accel << ","
                   << controllerNv0.x << ","
                   << controllerNv0.speed << ","
                   << controllerNv0.accel << ","
                   << nv0InfoAge << ","
                   << nv1.moduleName << ","
                   << nv1.x << ","
                   << nv1.speed << ","
                   << nv1.accel << ","
                   << controllerOk << ","
                   << accCmd << ","
                   << actuationOk << ","
                   << targetSpeed << "\n";
        commandOut.flush();
    }

  protected:
    virtual void initialize() override
    {
        interval = par("interval");

        controllerBridgeEnabled = par("controllerBridgeEnabled").boolValue();
        controllerHost = par("controllerHost").stdstringValue();
        controllerPort = par("controllerPort").intValue();

        useSimu5gDeliveredNv0 = par("useSimu5gDeliveredNv0").boolValue();

        networkDegradationEnabled = par("networkDegradationEnabled").boolValue();
        networkBaseDelay = par("networkBaseDelay");
        networkAttackStart = par("networkAttackStart");
        networkAttackEnd = par("networkAttackEnd");
        networkAttackDelay = par("networkAttackDelay");
        networkAttackJitter = par("networkAttackJitter");
        networkAttackDropProbability = par("networkAttackDropProbability").doubleValue();

        int networkSeed = par("networkSeed").intValue();
        rng.seed(networkSeed);

        unitDist = std::uniform_real_distribution<double>(0.0, 1.0);
        jitterDist = std::uniform_real_distribution<double>(
            -networkAttackJitter.dbl(),
            networkAttackJitter.dbl()
        );

        const char* logFile = par("logFile").stringValue();
        stateOut.open(logFile, std::ios::out);
        if (!stateOut.is_open())
            throw cRuntimeError("Could not open V2X live state log file: %s", logFile);

        stateOut << "sim_time,module,x,y,speed,acceleration\n";

        const char* commandLogFile = par("commandLogFile").stringValue();
        commandOut.open(commandLogFile, std::ios::out);
        if (!commandOut.is_open())
            throw cRuntimeError("Could not open V2X live controller command log file: %s", commandLogFile);

        commandOut << "sim_time,nv0_source,"
                   << "direct_nv0_module,direct_nv0_x,direct_nv0_speed,direct_nv0_accel,"
                   << "controller_nv0_x,controller_nv0_speed,controller_nv0_accel,nv0_info_age,"
                   << "nv1_module,nv1_x,nv1_speed,nv1_accel,"
                   << "controller_ok,acc_cmd_nv1,actuation_ok,target_speed_nv1\n";

        const char* networkMessageLogFile = par("networkMessageLogFile").stringValue();
        networkOut.open(networkMessageLogFile, std::ios::out);
        if (!networkOut.is_open())
            throw cRuntimeError("Could not open V2X live network message log file: %s", networkMessageLogFile);

        networkOut << "message_id,send_time,delivery_time,delay,in_attack_window,dropped,"
                   << "nv0_x,nv0_speed,nv0_accel\n";

        tick = new cMessage("v2xStateLoggerTick");
        scheduleAt(simTime(), tick);
    }

    virtual void handleMessage(cMessage* msg) override
    {
        if (msg == tick) {
            logVehicleStates();
            scheduleAt(simTime() + interval, tick);
        }
    }

    virtual void finish() override
    {
        if (stateOut.is_open())
            stateOut.close();

        if (commandOut.is_open())
            commandOut.close();

        if (networkOut.is_open())
            networkOut.close();
    }

    virtual ~V2XLiveStateLogger()
    {
        cancelAndDelete(tick);
    }
};

Define_Module(V2XLiveStateLogger);
