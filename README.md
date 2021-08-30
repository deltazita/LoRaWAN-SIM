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

## Dependencies:
- https://metacpan.org/pod/Math::Random
- https://metacpan.org/pod/Term::ProgressBar
- https://metacpan.org/pod/GD::SVG

## Usage:
```
perl generate_terrain.pl terrain_side_size_(m) num_of_nodes num_of_gateways > terrain.txt
perl LoRaWAN.pl packets_per_hour simulation_time(secs) ack_policy(1-3) terrain.txt
```

### Example with 1000x1000m terrain size, 100 nodes, 5 gateways, 1pkt/5min, ~3h sim time:
```
perl generate_terrain.pl 1000 100 5 > terrain.txt
perl LoRaWAN.pl 12 10000 2 terrain.txt
```

### Output sample:  
```
Simulation time = 10000.150 secs
Avg node consumption = 47.85191 mJ
Min node consumption = 20.75202 mJ
Max node consumption = 97.19977 mJ
Total number of transmissions = 91548
Total number of re-transmissions = 77286
Total number of unique transmissions = 19573
Total packets delivered = 72176
Total packets acknowledged = 13762
Total confirmed dropped = 5311
Total unconfirmed packets dropped = 0
Packet Delivery Ratio = 0.70311
Packet Reception Ratio = 0.78840
No GW available in RX1 = 67393 times
No GW available in RX1 or RX2 = 57897 times
Total downlink time = 2840.29696000017 sec
Script execution time = 5.5947 secs
-----
GW A sent out 8033 acks
GW B sent out 6246 acks
Mean downlink fairness = 0.379
Stdv of downlink fairness = 0.303
-----
# of nodes with SF7: 54
# of nodes with SF8: 58
# of nodes with SF9: 104
# of nodes with SF10: 116
# of nodes with SF11: 97
# of nodes with SF12: 71
Avg SF = 9.714
````
