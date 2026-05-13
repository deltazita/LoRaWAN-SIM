# LoRaWAN-SIM
A LoRaWAN simulator for confirmed/unconfirmed transmissions and multiple gateways

## List of papers where the simulator (or a variant) has been used:
- D. Zorbas, S. Tulebayeva, "AI-Assistance for LoRaWAN Simulators". The corresponding paper can be found here: https://www.researchgate.net/publication/404804983_AI-Assistance_for_LoRaWAN_Simulators
- D. Zorbas, C. Caillouet, K. Abdelfadeel, D. Pesch, "Optimal Data Collection Time in LoRa Networks: a Time-Slotted Approach", Sensors, Vol. 21, no. 4, 2021
- D. Zorbas, "Improving LoRaWAN Downlink Performance in the EU868 Spectrum", Computer Communications, Vol. 195, Nov. 2022, pp. 303-314
- D. Zorbas, "Downlink Spreading Factor Selection in LoRaWAN", Computer Communications, Vol. 215, Feb. 2024, pp. 112-119
- D. Zorbas and A. Sabyrbek, "Supporting Critical Downlink Traffic in LoRaWAN", Computer Communications, Vol. 228, 107981, Dec. 2024
- D. Zorbas, "LoRaWAN Network Coexistence in the EU868 Spectrum", 9th IEEE Conference on Standards for Communications and Networking, Belgrade, Serbia, Nov. 2024
- S. Javed, D. Zorbas, "A LoRaWAN Adaptive Retransmission Mechanism", IEEE Conference on Standards for Communications and Networking, Munich, Germany, Nov. 2023
- S. Javed, D. Zorbas, "Downlink Traffic Demand-Based Gateway Activation in LoRaWAN", 28th IEEE Symposium on Computers and Communications (ISCC), Tunis, Tunisia, July 2023
- S. Javed, D. Zorbas, "LoRaWAN Downlink Policies for Improved Fairness", IEEE Conference on Standards for Communications and Networking (CSCN), Thessaloniki, Greece, Nov. 2022

## Features:
- EU868 or US915 frequency plans
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
- AI-assisted recommendation system for improving PRR, PDR, or energy consumption

## Dependencies:
- https://metacpan.org/pod/Math::Random
- https://metacpan.org/pod/GD::SVG
- https://metacpan.org/dist/Statistics-Basic/view/lib/Statistics/Basic.pod

Debian: apt install libmath-random-perl libgd-svg-perl libstatistics-basic-perl

Python dependencies for the recommendation system:
- joblib
- pandas
- xgboost
- tabulate

Install them with:
```
pip install joblib pandas xgboost tabulate
```

## Usage:
```
perl generate_terrain.pl terrain_side_size_(m) num_of_nodes num_of_gateways > terrain.txt
perl LoRaWAN.pl packets_per_hour simulation_time_(hours) terrain.txt
```

### Example with 3000x3000m terrain size, 1000 nodes, 5 gateways, 1pkt/5min, 10h sim time:
```
perl generate_terrain.pl 3000 1000 5 > terrain.txt
(or perl generate_terrain-m.pl 3000 1000 > terrain.txt to automatically select the number of required gateways)
perl LoRaWAN.pl 12 10 terrain.txt
```


## JSON-based simulator input
The simulator can also be launched with a JSON configuration file. This mode is useful when the simulator is called by external tools, such as the recommendation system.

Example `config.json`:
```json
{
  "packets_per_hour": 12,
  "simulation_time": 5,
  "nodes": 2000,
  "gateways": 2,
  "terrain_side": 3000,
  "number_of_bands": 2,
  "rx2sf": 12,
  "with_ack": 0,
  "max_retr": 1,
  "pkt_size": 16,
  "adr": 1,
  "double_gws": 0
}
```

Run the JSON-capable simulator with:
```
perl LoRaWAN.pl --json config.json
```

If the JSON file does not include a terrain file, the simulator generates one internally using `generate_terrain.pl` and the values of `terrain_side`, `nodes`, and `gateways`. The original positional-argument workflow remains supported.

## AI-assisted recommendation system
The repository includes a Python launcher that runs the LoRaWAN simulator once for a baseline configuration and then uses trained ML models to recommend parameter changes. The recommendation targets are:

- `prr`: improve Packet Reception Ratio.
- `pdr`: improve Packet Delivery Ratio.
- `energy`: reduce node energy consumption.

The corresponding paper can be found here: https://www.researchgate.net/publication/404804983_AI-Assistance_for_LoRaWAN_Simulators

Expected files:
```
LoRaWAN.pl
launcher.py
generate_terrain.pl
model/xgb_classifier.pkl
nodel/xgb_energy_regressor.pkl
```

Run the recommendation system with a configuration file:
```
python3 launcher.py --config-file config.json --target prr
python3 launcher.py --config-file config.json --target pdr
python3 launcher.py --config-file config.json --target energy
```

The launcher uses `LoRaWAN.pl` as its default Perl simulator. It writes the normalized configuration to a temporary JSON file, calls the Perl simulator with `--json`, parses the baseline PRR, PDR, and energy values, and then reports a ranked list of recommendations.

Useful options:
```
python3 launcher.py --config-file config.json --target prr --top-n 10
python3 launcher.py --config-file config.json --target energy --max-params 2
python3 launcher.py --config-file config.json --target pdr --show-baseline-output
python3 launcher.py --config '{"packets_per_hour":12,"simulation_time":5,"nodes":2000,"gateways":1}' --target prr
```

Notes:
- Missing configuration fields fall back to the launcher's default values.
- The PRR/PDR targets require the classifier model `model/xgb_classifier.pkl`.
- The energy target requires the energy regressor `model/xgb_energy_regressor.pkl`.
- For confirmed traffic, set `with_ack` to `1`; for unconfirmed traffic, set it to `0`.
- `max_retr` is only meaningful when confirmed traffic is enabled.

### Output sample of the basic simulator:  
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
