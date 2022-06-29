# LoRaWAN-SIM
A LoRaWAN simulator for confirmed/unconfirmed transmissions and multiple gateways

If you want to cite the simulator, you can use the following bib entry:

@Article{zorbas2021optimal,  
  AUTHOR = {Zorbas, Dimitrios and Caillouet, Christelle and Abdelfadeel Hassan, Khaled and Pesch, Dirk},  
  TITLE = {Optimal Data Collection Time in LoRa Networks—A Time-Slotted Approach},  
  JOURNAL = {Sensors},  
  VOLUME = {21},  
  YEAR = {2021},  
  NUMBER = {4}  
}

## Features:
- Multiple half-duplex gateways
- 1% radio duty cycle for uplink transmissions
- 1 or 10% radio duty cycle for downlink transmissions
- Two receive windows (RX1, RX2) for ACKs and commands
- Non-orthogonal SF transmissions
- Capture effect
- Path-loss signal attenuation model
- Multiple channels
- Collision handling for both uplink+downlink transmissions
- Proper header overhead
- Node energy consumption calculation (uplink+downlink)
- ADR (Tx power adjustment)
- Gateway policies
- Adjustable packet size and rate

## Dependencies:
- https://metacpan.org/pod/Math::Random
- https://metacpan.org/pod/Term::ProgressBar
- https://metacpan.org/pod/GD::SVG

## Usage:
```
perl generate_terrain.pl terrain_side_size_(m) num_of_nodes num_of_gateways > terrain.txt
perl LoRaWAN.pl packets_per_hour simulation_time(secs) ack_policy(1-3) terrain.txt
```

### Example with 1500x1500m terrain size, 100 nodes, 2 gateways, 1pkt/5min, ~3h sim time:
```
perl generate_terrain.pl 1500 100 2 > terrain.txt
perl LoRaWAN.pl 12 10000 2 terrain.txt
```

### Output sample:  
```
Simulation time = 9997.862 secs
Avg node consumption = 7.95017 mJ
Min node consumption = 5.84691 mJ
Max node consumption = 13.48866 mJ
Total number of transmissions = 4008
Total number of re-transmissions = 592
Total number of unique transmissions = 3316
Stdv of unique transmissions = 0.39
Total packets delivered = 3620
Total packets acknowledged = 3316
Total confirmed packets dropped = 0
Total unconfirmed packets dropped = 0
Packet Delivery Ratio = 1.00000
Packet Reception Ratio = 0.90319
No GW available in RX1 = 1944 times
No GW available in RX1 or RX2 = 156 times
Total downlink time = 509.405184000007 sec
Script execution time = 0.2465 secs
-----
GW A sent out 1381 acks
GW B sent out 2083 acks
Downlink fairness = 0.038
Avg number of retransmissions = 0.179
Stdev of retransmissions = 5.633
-----
# of nodes with SF7: 18
# of nodes with SF8: 21
# of nodes with SF9: 18
# of nodes with SF10: 31
# of nodes with SF11: 7
# of nodes with SF12: 5
Avg SF = 9.030
Avg packet size = 30.110 bytes
````
### Generated image:
<img src="https://user-images.githubusercontent.com/6707477/176494005-28cd637f-0faa-4ec4-a584-7b2e935c9a6e.svg" width="400" height="400">
