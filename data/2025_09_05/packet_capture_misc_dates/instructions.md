to run on rsu:

capture RSPC-RSU on 7002: tcpdump -i eth0 port 7002 -s 0 -w r1.pcap

capture llc OBU-RSU channel: tcpdump -i llc-ch-ipv6 -s 0 -w r1.pcap


scp user@[fe80::6e5:48ff:fe30:0820%enxa0cec8e84936]:/home/user/pcaps/capturerspcsumo.pcap .


scp user@[fe80::6e5:48ff:fe30:0820%enxa0cec8e84936]:/home/user/pcaps/*.pcap /home/cra/sumo_ws/data/packet_capture/
