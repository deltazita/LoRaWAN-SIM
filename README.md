# LoRaWAN-SIM
A LoRaWAN simulator for confirmable transmissions and multiple gateways

## Features:
- Multiple half-duplex gateways
- 1% radio duty cycle for the nodes
- 10% radio duty cycle for the gateways
- Non-orthogonal SF transmissions
- Capture effect
- Acks with two receive windows (RX1, RX2)
- Path-loss signal attenuation model
- Multiple channels
- ADR (under development)

## Assumptions (or work in progress):
- Acks do not collide with other transmissions

## Dependencies:
- https://metacpan.org/pod/Math::Random
- https://metacpan.org/pod/Term::ProgressBar
- https://metacpan.org/pod/GD::SVG (optional for draw_terrain script)

## Usage:
```
perl generate_terrain.pl terrain_side_size_(m) num_of_nodes num_of_gateways > terrain.txt
perl LoRaWAN.pl full_collision_check(0/1) packets_per_hour simulation_time(secs) terrain.txt
```

### Example with 1000x1000m terrain size, 100 nodes, 5 gateways, 1pkt/5min, ~3h sim time:
```
perl generate_terrain.pl 1000 100 5 > terrain.txt
perl LoRaWAN.pl 1 12 10000 terrain.txt
```

### Output sample:  
```
Simulation time = 9995.851 secs
Avg node consumption = 55.71215 mJ
Min node consumption = 4.68841 mJ
Max node consumption = 411.96288 mJ
Total number of transmissions = 6657
Total number of re-transmissions = 5738
Total number of unique transmissions = 919
Total packets dropped = 537
Packet Delivery Ratio 1 = 0.22089
Packet Delivery Ratio 2 = 0.13805
No GW available in RX1 = 6381 times
No GW available in RX1 or RX2 = 6306 times
Script execution time = 0.9655 secs  
```
