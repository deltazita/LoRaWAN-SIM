# LR-FHSS simulation

Run from the repository directory:

```sh
perl LoRaWAN.pl --json json-examples/lrfhss-eu868.json
perl LoRaWAN.pl --json json-examples/lrfhss-us915.json
```

For an existing JSON configuration, add `"uplink_modulation": "LRFHSS"`, `"frequency_plan": "EU868"` or `"US915"`, and `"lrfhss_dr"`.
Omitting the modulation keeps the existing LoRa mode.
Each run uses one uplink modulation and one LR-FHSS data rate for all nodes.
The Python recommendation models were trained for LoRa, so they are not applicable for LR-FHSS experiments.

Supported funcionalities and descriptions are following.

## Regional profiles

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

## Configuration

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
- `debug`: 1 prints per-attempt LR-FHSS starts, ends, sizes and hop counts, plus MAC
  activity. Default 0.

## Reception model and limits

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

## Output and checks

Existing delivery, reception, retry, duty-cycle and energy statistics remain
available. LR-FHSS runs replace uplink SF statistics with the selected profile
and header/fragment observation and loss counts. Each observation belongs to one
packet at one gateway; losses include sensitivity, collision, and half-duplex
outages, so these counts are not unique transmitted fragments.
