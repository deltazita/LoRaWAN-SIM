# LoRaWAN-SIM
A LoRaWAN simulator for confirmed/unconfirmed transmissions and multiple gateways

The first version of the simulator was used here:

@article{zorbas2021optimal,
  AUTHOR = {Zorbas, Dimitrios and Caillouet, Christelle and Abdelfadeel Hassan, Khaled and Pesch, Dirk},<br />
  TITLE = {{Optimal Data Collection Time in LoRa Networks—A Time-Slotted Approach}},<br />
  JOURNAL = {Sensors},<br />
  VOLUME = {21},<br />
  YEAR = {2021},<br />
  NUMBER = {4}<br />
}

- Downlink policies:

@inproceedings{zorbas2022policies,
  author = {Javed, Shahzeb and Zorbas, Dimitrios},<br />
  title = {{LoRaWAN Downlink Policies for Improved Fairness}},<br />
  booktitle = {IEEE Conference on Standards for Communications and Networking (CSCN '22)},<br />
  year = {2022},<br />
  pages = {1--6},<br />
  month = {Nov},<br />
  location={Thessaloniki, Greece},<br />
  publisher={IEEE}<br />
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
- Downlink policies
- Adjustable packet size and rate

## Dependencies:
- https://metacpan.org/pod/Math::Random
- https://metacpan.org/pod/GD::SVG
- https://metacpan.org/dist/Statistics-Basic/view/lib/Statistics/Basic.pod

Debian: apt install libmath-random-perl libgd-svg-perl libstatistics-basic-perl

## Usage:
```
perl generate_terrain.pl terrain_side_size_(m) num_of_nodes num_of_gateways > terrain.txt
perl LoRaWAN.pl packets_per_hour simulation_time_(hours) terrain.txt
```

### Example with 3000x3000m terrain size, 1000 nodes, 5 gateways, 1pkt/5min, 10h sim time:
```
perl generate_terrain.pl 3000 1000 5 > terrain.txt
(or perl generate_terrain.pl 3000 1000 > terrain.txt to automatically select the number of required gateways)
perl LoRaWAN.pl 12 10 terrain.txt
```

### Output sample:  
```
Simulation time = 35999.408 secs
Avg node consumption = 50.50573 J
Min node consumption = 32.32120 J
Max node consumption = 157.91968 J
Total number of transmissions = 119862
Total number of unique transmissions = 119658
Stdv of unique transmissions = 0.47
Total packets delivered = 96832
Total packets acknowledged = 0
Total confirmed packets dropped = 0
Total unconfirmed packets dropped = 22826
Packet Delivery Ratio = 0.80924
Packet Reception Ratio = 0.80924
Uplink fairness = 0.088
Script execution time = 6.2148 secs
-----
# of nodes with SF7: 197, Avg retransmissions: 0.00
# of nodes with SF8: 119, Avg retransmissions: 0.00
# of nodes with SF9: 229, Avg retransmissions: 0.00
# of nodes with SF10: 279, Avg retransmissions: 0.00
# of nodes with SF11: 159, Avg retransmissions: 0.00
# of nodes with SF12: 17, Avg retransmissions: 0.00
Avg SF = 9.135
Avg packet size = 35.912 bytes
````
### Generated image:
<img src="https://user-images.githubusercontent.com/6707477/176494005-28cd637f-0faa-4ec4-a584-7b2e935c9a6e.svg" width="400" height="400">
