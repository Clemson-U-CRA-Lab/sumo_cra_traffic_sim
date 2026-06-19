package v2x_bottleneck;

import inet.node.inet.StandardHost;
import inet.node.ethernet.EthernetSwitch;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import ned.DatarateChannel;

network V2XBottleneckNetwork
{
    parameters:
        @display("bgb=750,420");

    types:
        channel AccessLink extends DatarateChannel
        {
            datarate = default(100Mbps);
            delay = default(1ms);
        }

        channel BottleneckLink extends DatarateChannel
        {
            datarate = default(10Mbps);
            delay = default(2ms);
        }

    submodules:
        configurator: Ipv4NetworkConfigurator {
            @display("p=70,50");
        }

        rsu: StandardHost {
            @display("p=120,160");
        }

        attacker: StandardHost {
            @display("p=120,290");
        }

        switch: EthernetSwitch {
            @display("p=360,225");
        }

        vehicle: StandardHost {
            @display("p=620,225");
        }

    connections:
        rsu.ethg++ <--> AccessLink <--> switch.ethg++;
        attacker.ethg++ <--> AccessLink <--> switch.ethg++;
        switch.ethg++ <--> BottleneckLink <--> vehicle.ethg++;
}
