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
- LR-FHSS uplinks with frequency hopping, header replicas and coded fragment recovery
- Multiple half-duplex gateways
- 1% radio duty cycle for uplink transmissions
- 1 or 10% radio duty cycle for downlink transmissions
- Two receive windows (RX1, RX2) for ACKs and commands
- Non-orthogonal SF transmissions
- Capture effect
- Path-loss signal attenuation model
- Optional Rayleigh block fading on desired signals and interferers
- Terrain generation with nominal gateway coverage for every ED
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

## Rayleigh fading and coverage

Rayleigh fading is enabled by default for LoRa and LR-FHSS, including positional
and JSON input and the Python launcher. Add `--no-rayleigh` to the simulator
command, or set `"rayleigh_fading": 0` in JSON, to disable it. The command-line
flag overrides the JSON value. The selected path loss and Gaussian shadowing
remain active. The function
`rayleigh_gain_db(...)` is implemented in `LoRaWAN.pl`; no fading module is needed.
For example:

```sh
perl LoRaWAN.pl --json json-examples/rayleigh-us915.json
perl LoRaWAN.pl 12 1 terrain.txt --fhss --no-rayleigh
perl LoRaWAN.pl --json config.json --no-rayleigh
```

Rayleigh fading multiplies received power by a unit-mean exponential gain
(`H = -ln(U)`), or adds `10*log10(H)` in dB. The existing path loss and
Gaussian shadowing remain in effect. Each transmission/receiver pair gets one
gain, reused for desired-signal and interference calculations. For LR-FHSS,
this is flat block fading: the same gain applies to all hops in a frame.
Retransmissions and NbTrans copies get independent samples. Samples are keyed
by `seed`, transmitter, receiver, transmission start and frame counter, so they
do not consume scheduling or hopping randomness. No Doppler/time correlation
or frequency-selective fading is modeled.

The deterministic sampler uses Perl's native `Digest::MD5` as a fast, non-security
mixer, taking 32 bits for the uniform sample. Transmission times are encoded as
big-endian binary doubles instead of formatting decimal strings. This replaces
the earlier SHA-256 sampler: old seeds now produce different fading realizations,
but repeated runs with the same seed remain reproducible. The Rayleigh model is
unchanged, and scheduling/hopping random streams are still separate.

LoRa uses a bounded gain cache with a single key-construction step per lookup.
LR-FHSS reuses the radio's per-frame/receiver received-power cache instead of
maintaining a second fading cache. With `--no-rayleigh`, received-power calculations
skip the fading function entirely.

`coverage_margin_db` defaults to `10` dB, matching the installation margin in
[ChirpStack](https://www.chirpstack.io/docs/chirpstack/configuration.html), and
must be finite and nonnegative. This shared margin is used for terrain coverage,
initial LoRa SF selection and ADR transmit-power adjustment.
At startup every ED must have at least one nominally reachable gateway, using
its initial transmit power, the most robust permitted LoRa SF or selected
LR-FHSS DR, and that margin. Invalid supplied terrains fail with the uncovered
ED IDs. Coverage means a viable uplink before random shadowing/fading; it does
not guarantee reception of every packet or successful downlinks, and ADR may
subsequently change transmit power.

Both terrain generators accept the same coverage options after their existing
positional arguments. The optional positional seed is supported by both:

```sh
perl generate_terrain.pl 4000 1000 2 42 --frequency-plan US915 --fhss --lrfhss-dr 6 --coverage-margin-db 10 > terrain.txt
perl generate_terrain-m.pl 4000 1000 42 --frequency-plan US915 --fhss --lrfhss-dr 6 --coverage-margin-db 10 > terrain.txt
```

Use `--fhss` when generating terrain for an LR-FHSS simulation; omitting it
generates LoRa terrain. Both generators use DR8 in EU868 or DR5 in US915 unless
`--lrfhss-dr` is specified. Use the same region, DR and margin when simulating.

`--lrfhss-sensitivity-dbm` supports the receiver-threshold override. Defaults
remain EU868 LoRa, with initial ED power of 14 dBm (20 dBm in US915).
The fixed-count generator resamples ED positions until covered. The `-m`
generator selects gateways greedily from random candidates, supplementing them
near uncovered EDs if necessary; it no longer needs `Algorithm::SetCovering`.
Both verify unique coordinates and emit a complete covered terrain or fail.
Automatic generation receives the simulator's actual PHY and margin settings.

The Python launcher passes through `rayleigh_fading`, `coverage_margin_db`,
`environment` and `seed`, and includes them in its simulation cache key. These controls do not
change the trained recommendation models' feature set or retrain those models.

## Propagation environment

`--environment legacy|oulu|bonn` selects a log-distance path-loss and Gaussian
shadowing profile. JSON uses the `environment` key, for example `"environment": "oulu"`. This setting is independent
of `--fhss` and applies to LoRa, LR-FHSS, downlinks and interfering signals.
The simulator, fixed-count generator and greedy generator share the same
parameters and nominal coverage calculation.

The model is `PL(d) = L0 + 10*n*log10(d/d0) + X`, where `X` is zero-mean Gaussian
in dB. The existing 0.2 m minimum model distance is retained.

| Environment | Reference distance d0 | Loss L0 | Exponent n | Shadowing stddev |
|---|---:|---:|---:|---:|
| `legacy` (default) | 40 m | 110 dB | 2.08 | 3.57 dB |
| `oulu` | 1000 m | 128.95 dB | 2.32 | 7.8 dB |
| `bonn` | 1000 m | 132.41 dB | 1.58 | 9.9 dB |

The legacy curve is equivalent to 139.08 dB median loss at 1 km. It remains the
default so that existing experiments do not silently acquire a different model.

### Measurement sources and applicability

`oulu` uses the on-ground/car fit in Table IV of
[Petajajarvi et al., ITST 2015](https://cc.oulu.fi/~kmikhayl/site-assets/pdfs/2015_ITST.pdf).
The measurements were at 868 MHz in Oulu, Finland, using a car-roof device about
2 m above ground and a fixed gateway antenna reported at 24 m above sea level.
This is the land-based fit, not the paper's separate boat/over-water fit.

`bonn` uses the urban fit from
[Rademacher et al., Path Loss in Urban LoRa Networks: A Large-Scale Measurement Study](https://arxiv.org/abs/2109.07768).
Their 868 MHz deployment used gateways on roofs/poles and sensors on garbage
truck roofs. The fitted low exponent depends on the included long-distance
links and local topography; it is not a universal dense-urban exponent.

These are reproducible scenario presets, not a substitute for calibrating the
target site. They do not calculate building penetration, antenna-height effects
or antenna gains separately. Do not infer that every rural, suburban or urban
deployment shares either curve. There is no automatic height or frequency
correction: selecting these 868 MHz fits for US915 is an unvalidated transfer,
not a measured 915 MHz calibration. Extrapolation outside the studies' distance
and deployment conditions should be treated cautiously.

### Usage and interpretation

```sh
perl generate_terrain.pl 3000 2000 3 --fhss --environment oulu > terrain-oulu.txt
perl LoRaWAN.pl 12 1 terrain-oulu.txt --fhss --environment oulu
perl generate_terrain-m.pl 3000 2000 --fhss --environment bonn > terrain-bonn.txt
```

Use the same environment, region, DR, sensitivity and margin on both sides.
Automatic terrain generation forwards the environment. Non-legacy generated
files record the selected coefficients in a `# propagation:` header.

At EU868's initial 14 dBm and the default 10 dB margin, the calculated nominal
LoRa SF12 / LR-FHSS DR8 ranges are:

| Environment | LoRa SF12 | LR-FHSS DR8 |
|---|---:|---:|
| `legacy` | 1.93 km | 1.24 km |
| `oulu` | 4.92 km | 3.31 km |
| `bonn` | 6.26 km | 3.50 km |

These are median-path-loss planning distances, not guaranteed reception ranges.
The measured presets also have greater shadowing variation. A lower median loss
therefore does not guarantee a particular PDR improvement.

Rayleigh fading remains independently enabled by default. The presets retain
the simulator's existing per-transmission shadowing sampling; they do not
reproduce spatial correlation or the studies' full temporal statistics. Measured
RSSI scatter may already include unresolved small-scale fading. When reproducing
one of the fitted log-normal models, use `--no-rayleigh` (or set
`"rayleigh_fading": 0` in JSON) as the baseline; jointly calibrate shadowing and
Rayleigh before treating them as independent extra impairments. LR-FHSS fading is still flat over the whole frame.

The launcher forwards the environment and separates its cache entries. Existing
recommendation models are not retrained or validated for these new environments.

## LR-FHSS uplinks

Run from the repository directory:

```sh
perl LoRaWAN.pl 12 10 terrain.txt --fhss
perl LoRaWAN.pl 12 10 terrain.txt --fhss --frequency-plan US915 --lrfhss-dr 6
perl LoRaWAN.pl --json json-examples/lrfhss-eu868.json
perl LoRaWAN.pl --json json-examples/lrfhss-us915.json
```

`--fhss` enables LR-FHSS without JSON. `--frequency-plan` (also `--region`)
defaults to EU868; the default DR is 8 for EU868 or 5 for US915.
`--lrfhss-dr`, `--lrfhss-sensitivity-dbm`, `--lrfhss-capture-db`, `--seed` and
`--picture 0|1` expose the corresponding JSON settings. Options may appear
before or after positional arguments, including the 12/13-argument automatic
terrain form, and override values loaded with `--json`, `--config` or a bare
JSON filename. Without `--fhss`, the JSON modulation setting is used, or LoRa
if none is specified.
Use `--help` for usage.

Both terrain generators also use `--fhss` to plan coverage for LR-FHSS:

```sh
perl generate_terrain.pl 3000 2000 3 --fhss > terrain-fhss.txt
perl generate_terrain-m.pl 3000 2000 --fhss > terrain-fhss.txt
```

Without `--fhss` they plan for LoRa. Their `--frequency-plan` and `--lrfhss-dr`
options should match the simulator settings (defaults: EU868 DR8 or US915 DR5).

For an existing JSON configuration, add `"uplink_modulation": "LRFHSS"`, `"frequency_plan": "EU868"` or `"US915"`, and `"lrfhss_dr"`.
Omitting the modulation keeps the existing LoRa mode.
Each run uses one uplink modulation and one LR-FHSS data rate for all nodes.
The Python recommendation models were trained for LoRa, so they are not applicable for LR-FHSS experiments.

Keep `lib/LoRaWAN/LRFHSS.pm` in the repository's `lib` directory next to the simulator.
The JSON modulation value also accepts `"LR-FHSS"`.

The supported profiles and reception assumptions are described below.

### Regional profiles

| Region | DR | CR  | OCW (kHz) | Headers | Max application bytes | RX1 LoRa SF/BW |
|--------|----|-----|-----------|---------|-----------------------|----------------|
| EU868  | 8  | 1/3 | 137       | 3       | 50                    | 11/125 kHz     |
| EU868  | 9  | 2/3 | 137       | 2       | 115                   | 10/125 kHz     |
| EU868  | 10 | 1/3 | 336       | 3       | 50                    | 11/125 kHz     |
| EU868  | 11 | 2/3 | 336       | 2       | 115                   | 10/125 kHz     |
| US915  | 5  | 1/3 | 1523      | 3       | 50                    | 10/500 kHz     |
| US915  | 6  | 2/3 | 1523      | 2       | 125                   | 9/500 kHz      |

These profiles, RX1 mappings at offset zero, and the sample-based airtime formula
follow [LoRa Alliance RP002-1.0.5](https://resources.lora-alliance.org/technical-specifications/rp002-1-0-5-lorawan-regional-parameters),
sections 3.4, 3.5 and 5.3. Each header lasts 233.472 ms; payload hops last up to
102.4 ms. Airtime includes the short final hop, payload CRC, trellis termination,
and hop preambles. For example, a 16-byte application message plus 13-byte LoRaWAN
overhead occupies 2.326528 s at DR8 or 1.280000 s at DR9.

RX2 uses `rx2sf` and the regional downlink bandwidth. Its existing default is SF9;
the US915 example explicitly selects SF12. Downlinks always use LoRa.

### LR-FHSS configuration

- `lrfhss_dr`: defaults to 8 in EU868 or 5 in US915. Incompatible profiles are rejected.
- `lrfhss_sensitivity_dbm`: receiver threshold; defaults to -137 dBm for CR1/3 and
  -134 dBm for CR2/3. These are adjustable simulation assumptions, not a certified
  gateway specification.
- `lrfhss_capture_db`: required power advantage over each overlapping interferer;
  defaults to 6 dB. This is also a simulation assumption.
- `pkt_size`: application bytes, before the existing 13-byte LoRaWAN overhead.
  With `adr: 1`, two bytes are reserved for LinkADRAns, reducing the permitted
  application size by two. Oversized configurations fail with an explanation.
- `with_ack`, `max_retr`, `nbtrans`, `adr`, and `double_gws`: use the existing MAC
  configuration. ADR adapts power and NbTrans; the selected LR-FHSS DR stays fixed.
- `number_of_bands`: EU868 duty-cycle bands. For 137 kHz profiles, existing channel
  centers are used. For 336 kHz, centers are 868.3 MHz in band 48 and 866.9, 867.3,
  867.7 MHz in band 47. One-band operation uses only 868.3 MHz. These are simulator
  channel choices. US915 uses centers 903.0 through 914.2 MHz at 1.6 MHz intervals.
- `seed`: seeds the simulator's Perl and Math::Random generators and the terrain
  generator. Identical inputs reproduce runs on the same software stack, excluding
  execution-time statistics and generated filenames.
- `picture`: 0 disables SVG output, 1 retains the existing terrain-adjacent SVG.
- `rayleigh_fading`: 1 (default) or 0 to disable. `--no-rayleigh` disables it
  from the command line, overriding JSON. When enabled, one normalized Rayleigh
  power gain is held across all hops of each transmission/receiver pair; fresh
  transmission attempts get fresh samples. Fading also applies to LoRa downlinks
  and interferers. The sampler is a function in `LoRaWAN.pl`.
- `coverage_margin_db`: defaults to 10 dB, matching ChirpStack's installation
  margin. It is also used by ADR. Every ED must nominally reach a gateway
  at the selected LR-FHSS DR and initial TX power with this margin. The two
  terrain generators use the same rule; supplied uncovered terrains are rejected.
  Actual packets can still be lost to random fading, shadowing or collisions.
- `environment`: `legacy` (default), `oulu` or `bonn`. Selects the shared median
  path-loss and shadowing parameters for LoRa and LR-FHSS, including downlinks
  and interference. The simulator and both generators accept `--environment`.
  Use matching settings; see [Propagation environment](#propagation-environment) for the measured fits.
- `debug`: 1 prints per-attempt LR-FHSS starts, ends, sizes and hop counts, plus MAC
  activity. Default 0.

### Reception model and limits

`lib/LoRaWAN/LRFHSS.pm` constructs a fresh hopping pattern for each transmission
attempt, including retransmissions and NbTrans copies. It chooses a grid and
shuffles its frequency slots without replacement, cycling as needed. This is a
balanced statistical hopping model; it does not implement Semtech's on-air
hopping sequence identifiers or a bit-level modem.

Transmissions enter the radio model only when they start. Overlapping hop
intervals and spectra determine losses independently at each gateway. Full
containment counts as overlap; touching endpoints do not. A gateway transmitting
a downlink cannot receive overlapping uplink hops on any frequency. Cancelled
future copies cannot interfere. Packet reception is decided at transmission end,
after every overlapping start has been processed.

An ideal erasure decoder requires one intact header and at least the coding-rate
fraction of coded payload bits, weighting a short final fragment by its actual
size. This approximation is not a convolutional decoder or a BER/PER curve.
Gateways decode independently, without combining fragments between gateways.
Their number of concurrent LR-FHSS decoders is unlimited in this model.

Capture is evaluated against each interferer separately, with rectangular spectra
and interferer power scaled to spectral overlap. Aggregate interference, oscillator
errors, adjacent-channel leakage, gateway processing limits, and external traffic
are not modeled. Shadowing is sampled once per transmission/receiver pair.

Energy uses the existing TX, RX and MCU power tables, integrating LR-FHSS airtime
and LoRa receive windows. It accounts for ADR power changes and cancelled copies,
but does not add radio retuning transients or long-term sleep consumption.
As in the original simulator, duty cycle is applied per EU868 band; there is no
US915 duty-cycle multiplier. A run stops starting new uplinks at its time limit
and finishes in-flight receptions and their downlinks, so reported end time can
slightly exceed the requested duration.

### LR-FHSS output and checks

Existing delivery, reception, retry, duty-cycle and energy statistics remain
available. LR-FHSS runs replace uplink SF statistics with the selected profile
and header/fragment observation and loss counts. Each observation belongs to one
packet at one gateway; losses include sensitivity, collision, and half-duplex
outages, so these counts are not unique transmitted fragments.

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
```

### Generated image:

<img src="https://user-images.githubusercontent.com/6707477/176494005-28cd637f-0faa-4ec4-a584-7b2e935c9a6e.svg" width="400" height="400">
