package v2x_controller_live;

simple V2XLiveStateLogger
{
    parameters:
        @class(V2XLiveStateLogger);

        string logFile = default("v2x_live_vehicle_states.csv");
        string commandLogFile = default("v2x_live_controller_commands.csv");
        string networkMessageLogFile = default("v2x_live_network_messages.csv");

        double interval @unit(s) = default(0.1s);

        bool controllerBridgeEnabled = default(false);
        string controllerHost = default("127.0.0.1");
        int controllerPort = default(5555);

        // Neutral by default. Experiments override these in the run script.
        bool useSimu5gDeliveredNv0 = default(false);

        bool networkDegradationEnabled = default(false);
        double networkBaseDelay @unit(s) = default(0s);
        double networkAttackStart @unit(s) = default(5s);
        double networkAttackEnd @unit(s) = default(10s);
        double networkAttackDelay @unit(s) = default(0s);
        double networkAttackJitter @unit(s) = default(0s);
        double networkAttackDropProbability = default(0);
        int networkSeed = default(1);

        @display("i=block/table");
}
