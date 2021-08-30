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
perl LoRaWAN.pl packets_per_hour simulation_time(secs) terrain.txt
```

### Example with 1000x1000m terrain size, 100 nodes, 5 gateways, 1pkt/5min, ~3h sim time:
```
perl generate_terrain.pl 1000 100 5 > terrain.txt
perl LoRaWAN.pl 12 10000 terrain.txt
```

### Output sample:  
```
Simulation time = 10000.130 secs
Avg node consumption = 4.22738 mJ
Min node consumption = 0.14028 mJ
Max node consumption = 24.44699 mJ
Total number of transmissions = 7425
Total number of re-transmissions = 1457
Total number of unique transmissions = 5968
Total packets acknowledged = 5578
Total packets dropped = 1
Packet Delivery Ratio 1 = 0.93465
Packet Delivery Ratio 2 = 0.80377
No GW available in RX1 = 112 times
No GW available in RX1 or RX2 = 21 times
Script execution time = 34.3681 secs
-----
GW A sent out 1731 acks
GW B sent out 722 acks
GW C sent out 1129 acks
GW D sent out 1340 acks
GW E sent out 666 acks
```
