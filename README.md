# LoRaWAN-SIM
A LoRaWAN simulator for confirmable transmissions and multiple gateways

# Features:
-- Multiple half-duplex gateways
-- 1% radio duty cycle for the nodes
-- 10% radio duty cycle for the gateways
-- Non-orthogonal SF transmissions
-- Capture effect
-- Acks with two receive windows (RX1, RX2)
-- Path-loss signal attenuation model

# Assumptions (or work in progress):
-- All uplink transmissions are performed over the same channel
-- Acks do not collide to each other         

# Dependencies:
- https://metacpan.org/pod/Math::Random
- https://metacpan.org/pod/GD::SVG (optional for draw_terrain script)

# Usage:
perl generate_terrain.pl terrain_side_size_(m) num_of_nodes num_of_gateways > terrain.txt
perl LoRaWAN.pl full_collision_check(0/1) packets_per_hour simulation_time(secs) terrain.txt

# Example with 1000x1000m terrain size, 100 nodes, 5 gateways, 1pkt/5min, ~3h sim time:
perl generate_terrain.pl 1000 100 5 > terrain.txt
perl LoRaWAN.pl 1 12 10000 terrain.txt
