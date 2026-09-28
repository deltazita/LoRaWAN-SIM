#!/usr/bin/perl -w

###################################################################################
#          Event-based simulator for (un)confirmed LoRaWAN transmissions          #
#                                   v2026.9.14                                    #
#                                                                                 #
# Features:                                                                       #
# -- EU868 or US915 spectrum                                                      #
# -- Multiple half-duplex gateways                                                #
# -- 1% radio duty cycle per band for the nodes (EU868)                           #
# -- 1 or 10% radio duty cycle for the gateways (EU868)                           #
# -- Acks with two receive windows (RX1, RX2)                                     #
# -- LR-FHSS uplinks (header replicas, hopping, coded fragment recovery)          #
# -- Non-orthogonal SF transmissions                                              #
# -- Periodic or non-periodic (exponential) transmission rate                     #
# -- Percentage of nodes required confirmed transmissions                         #
# -- Capture effect                                                               #
# -- Path-loss signal attenuation model (uplink+downlink)                         #
# -- Multiple channels                                                            #
# -- Collision handling for both uplink and downlink transmissions                #
# -- Energy consumption calculation (uplink+downlink)                             #
# -- ADR support (TX power + ChirpStack-like NbTrans for unconfirmed uplinks)     #
# -- Network server policies (downlink packet & gw selection)                     #
#                                                                                 #
# author: Dr. Dimitrios Zorbas                                                    #
# email: dimzorbas@ieee.org                                                       #
# distributed under GNUv2 General Public Licence                                  #
###################################################################################

use strict;
use FindBin qw($Bin);
use lib "$Bin/lib";
use LoRaWAN::LRFHSS;
use POSIX;
use List::Util qw(min max sum);
use Time::HiRes qw(time);
use Math::Random qw(random_uniform random_exponential random_normal);
use GD::SVG;
use Statistics::Basic qw(:all);
use JSON::PP qw(decode_json);
use File::Basename qw(dirname);
use File::Path qw(make_path);
use File::Spec;
use Cwd qw(abs_path);

my @CONFIG_KEYS = (
	"packets_per_hour",
	"simulation_time",
	"nodes",
	"gateways",
	"terrain_side",
	"number_of_bands",
	"rx2sf",
	"with_ack",
	"max_retr",
	"pkt_size",
	"adr",
	"double_gws",
);

my %CONFIG_ALIASES = (
	"packet_rate" => "packets_per_hour",
	"sim_time" => "simulation_time",
	"simulation_time_hours" => "simulation_time",
	"num_nodes" => "nodes",
	"num_gateways" => "gateways",
	"terrain" => "terrain_side",
	"terrain_size" => "terrain_side",
	"bands" => "number_of_bands",
	"num_bands" => "number_of_bands",
	"confirmed" => "with_ack",
	"ack" => "with_ack",
	"max_retries" => "max_retr",
	"packet_size" => "pkt_size",
	"payload_size" => "pkt_size",
	"adr_on" => "adr",
	"nb_trans" => "nbtrans",
	"modulation" => "uplink_modulation",
	"region" => "frequency_plan",
);

sub usage {
	return "usage: $0 <packets_per_hour> <simulation_time_(hours)> <terrain_file!>\n" .
		   "   or: $0 --json <config.json>\n" .
		   "   or: $0 <config.json>\n" .
		   "   or: $0 <packets_per_hour> <simulation_time> <nodes> <gateways> <terrain_side> <number_of_bands> <rx2sf> <with_ack> <max_retr> <pkt_size> <adr> <double_gws> [nbtrans]\n";
}

sub canonicalize_config {
	my ($cfg_ref) = @_;
	foreach my $alias (keys %CONFIG_ALIASES) {
		my $canonical = $CONFIG_ALIASES{$alias};
		if (exists $cfg_ref->{$alias} && !exists $cfg_ref->{$canonical}) {
			$cfg_ref->{$canonical} = $cfg_ref->{$alias};
		}
	}
}

sub read_json_config {
	my ($json_file) = @_;
	open(my $fh, "<", $json_file) or die "Error: could not open JSON config file $json_file\n";
	local $/;
	my $json_text = <$fh>;
	close($fh);

	my $cfg = eval { decode_json($json_text) };
	die "Error: could not parse JSON config file $json_file: $@\n" if ($@);
	die "Error: JSON config file must contain one object at the top level\n" unless (ref($cfg) eq "HASH");

	$cfg->{__config_dir} = dirname(abs_path($json_file));
	canonicalize_config($cfg);
	return %{$cfg};
}

sub parse_simulator_config {
	my @args = @_;
	die usage() if (scalar @args == 0);

	if ((scalar @args == 2) && (($args[0] eq "--json") || ($args[0] eq "--config"))) {
		return read_json_config($args[1]);
	}
	if ((scalar @args == 1) && ($args[0] =~ /\.json$/i)) {
		return read_json_config($args[0]);
	}
	if (scalar @args == 3) {
		return (
			"packets_per_hour" => $args[0],
			"simulation_time" => $args[1],
			"terrain_file" => $args[2],
		);
	}
	if ((scalar @args == scalar @CONFIG_KEYS) || (scalar @args == scalar @CONFIG_KEYS + 1)) {
		my %cfg = ();
		for (my $i = 0; $i < scalar @CONFIG_KEYS; $i += 1) {
			$cfg{$CONFIG_KEYS[$i]} = $args[$i];
		}
		# Optional 13th positional parameter, kept optional for backward compatibility.
		$cfg{"nbtrans"} = $args[-1] if (scalar @args == scalar @CONFIG_KEYS + 1);
		return %cfg;
	}

	die usage();
}

sub config_value {
	my ($cfg_ref, $key, $default) = @_;
	return exists $cfg_ref->{$key} ? $cfg_ref->{$key} : $default;
}

sub require_config_value {
	my ($cfg_ref, $key) = @_;
	die "Error: missing required configuration key '$key'\n" unless (exists $cfg_ref->{$key});
	return $cfg_ref->{$key};
}

sub as_int {
	my ($value, $key) = @_;
	die "Error: missing value for '$key'\n" unless defined $value;
	die "Error: '$key' must be an integer, got '$value'\n" unless ($value =~ /^-?\d+$/);
	return int($value);
}

sub as_number {
	my ($value, $key) = @_;
	die "Error: missing value for '$key'\n" unless defined $value;
	die "Error: '$key' must be numeric, got '$value'\n"
		unless ($value =~ /^-?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$/);
	return 0 + $value;
}

sub resolve_path {
	my ($path, $base_dir) = @_;
	return $path if File::Spec->file_name_is_absolute($path);
	return File::Spec->catfile($base_dir, $path) if defined $base_dir;
	return File::Spec->catfile(Cwd::getcwd(), $path);
}

sub script_dir {
	return dirname(abs_path($0));
}

sub generate_terrain_from_config {
	my ($cfg_ref) = @_;
	my $side = as_int(require_config_value($cfg_ref, "terrain_side"), "terrain_side");
	my $nodes = as_int(require_config_value($cfg_ref, "nodes"), "nodes");
	my $gateways = as_int(require_config_value($cfg_ref, "gateways"), "gateways");

	die "Terrain side must be higher than 0\n" if ($side <= 0);
	die "Number of nodes must be higher than 0\n" if ($nodes <= 0);
	die "Number of gateways must be higher than 0\n" if ($gateways <= 0);

	my $dir = script_dir();
	my $generator = File::Spec->catfile($dir, "generate_terrain.pl");
	die "Error: missing companion script generate_terrain.pl next to $0\n" unless (-e $generator);

	my $tmp_dir = File::Spec->catdir($dir, "tmp");
	make_path($tmp_dir) unless (-d $tmp_dir);
	my $terrain_file = File::Spec->catfile($tmp_dir, "terrain_${side}_${nodes}_${gateways}_$$.txt");

	my @generator_args = ($side, $nodes, $gateways);
	push @generator_args, as_int($cfg_ref->{seed}, "seed") if exists $cfg_ref->{seed};
	open(my $gen_fh, "-|", "perl", $generator, @generator_args)
		or die "Error: could not execute $generator\n";
	open(my $out_fh, ">", $terrain_file)
		or die "Error: could not create generated terrain file $terrain_file\n";
	while (my $line = <$gen_fh>) {
		print $out_fh $line;
	}
	close($out_fh);
	close($gen_fh) or die "Error: generate_terrain.pl failed while creating $terrain_file\n";

	return $terrain_file;
}

my %CONFIG = parse_simulator_config(@ARGV);
canonicalize_config(\%CONFIG);

my $packets_per_hour = as_int(require_config_value(\%CONFIG, "packets_per_hour"), "packets_per_hour");
my $simulation_time_h = as_int(require_config_value(\%CONFIG, "simulation_time"), "simulation_time");
my $number_of_bands = as_int(config_value(\%CONFIG, "number_of_bands", 2), "number_of_bands");
my $configured_rx2sf = as_int(config_value(\%CONFIG, "rx2sf", 9), "rx2sf");
my $configured_with_ack = as_number(config_value(\%CONFIG, "with_ack", 0), "with_ack");
my $configured_max_retr = as_int(config_value(\%CONFIG, "max_retr", 1), "max_retr");
my $configured_pkt_size = as_int(config_value(\%CONFIG, "pkt_size", 16), "pkt_size");
my $configured_adr = as_int(config_value(\%CONFIG, "adr", 1), "adr");
my $configured_double_gws = as_int(config_value(\%CONFIG, "double_gws", 0), "double_gws");
my $configured_nbtrans = as_int(config_value(\%CONFIG, "nbtrans", 1), "nbtrans"); # initial NbTrans; ChirpStack ADR subsequently adapts it in the range 1..3
my $fplan = uc(config_value(\%CONFIG, "frequency_plan", "EU868"));
die "frequency_plan must be EU868 or US915\n" unless $fplan eq 'EU868' || $fplan eq 'US915';
my $modulation = config_value(\%CONFIG, "uplink_modulation", "LoRa"); # leave LoRa as the default modulation. FHSS can only be used via a json configuration file
$modulation =~ s/[-_]//g;
die "uplink_modulation must be LoRa or LRFHSS\n" unless $modulation eq 'LoRa' || $modulation eq 'LRFHSS';
my $lrfhss = $modulation eq 'LRFHSS';
my $lr_profile;
my %lr_keys = map { $_ => 1 } qw(lrfhss_dr lrfhss_sensitivity_dbm lrfhss_capture_db);
foreach my $key (grep { /^lrfhss_/ } keys %CONFIG) { # make sure all required info is given
	die "Unknown LR-FHSS configuration key '$key'\n" unless $lr_keys{$key};
}
if ($lrfhss) {
	my $dr = as_int(config_value(\%CONFIG, "lrfhss_dr", $fplan eq 'EU868' ? 8 : 5), "lrfhss_dr");
	$lr_profile = LoRaWAN::LRFHSS::profile($fplan, $dr);
	my $limit = $lr_profile->{max_payload} - ($configured_adr ? 2 : 0);
	die "pkt_size exceeds LR-FHSS DR$dr application limit ($limit bytes, including space reserved for ADR answers)\n"
		if $configured_pkt_size > $limit;
} elsif (grep { /^lrfhss_/ } keys %CONFIG) {
	die "lrfhss_* settings require uplink_modulation LRFHSS\n";
}
# Receiver assumptions, configurable independently of the regional PHY profile.
my $lr_sensitivity = as_number(config_value(\%CONFIG, "lrfhss_sensitivity_dbm", $lrfhss && $lr_profile->{cr_num} == 2 ? -134 : -137), "lrfhss_sensitivity_dbm");
my $lr_capture = as_number(config_value(\%CONFIG, "lrfhss_capture_db", 6), "lrfhss_capture_db");
die "lrfhss_capture_db must be nonnegative\n" if $lr_capture < 0;
if (exists $CONFIG{seed}) {
	my $seed = as_int($CONFIG{seed}, "seed");
	srand($seed);
	Math::Random::random_set_seed_from_phrase("LoRaWAN-SIM:$seed");
}
my $terrain_file = exists $CONFIG{"terrain_file"}
	? resolve_path($CONFIG{"terrain_file"}, $CONFIG{"__config_dir"})
	: generate_terrain_from_config(\%CONFIG);

$number_of_bands = 1 if ($number_of_bands < 1);
$number_of_bands = 2 if ($number_of_bands > 2);

die "Packet rate must be higher than or equal to 1pkt per hour\n" if ($packets_per_hour < 1);
die "Simulation time must be longer than or equal to 1h\n" if ($simulation_time_h < 1);
die "RX2 SF must be between 7 and 12\n" if (($configured_rx2sf < 7) || ($configured_rx2sf > 12));
die "with_ack must be between 0 and 1\n" if (($configured_with_ack < 0) || ($configured_with_ack > 1));
die "max_retr must be higher than or equal to 0\n" if ($configured_max_retr < 0);
die "pkt_size must be higher than 0\n" if ($configured_pkt_size < 1);
die "adr must be 0 or 1\n" if (($configured_adr != 0) && ($configured_adr != 1));
die "double_gws must be 0 or 1\n" if (($configured_double_gws != 0) && ($configured_double_gws != 1));
die "nbtrans must be between 1 and 3\n" if (($configured_nbtrans < 1) || ($configured_nbtrans > 3));

# node attributes
my %ncoords = (); # node coordinates
my %nconsumption = (); # consumption
my %nretransmissions = (); # retransmissions per node (per packet)
my %surpressed = ();
my %nreachablegws = (); # reachable gws
my %nptx = (); # transmit power index
my %nresponse = (); # 0/1 (1 = LinkADRAns will be sent in the next uplink)
my %nnbtrans = (); # currently applied NbTrans per node
my %nnbtrans_count = (); # current copy number for this FCntUp
my %nconfirmed = (); # confirmed transmissions or not
my %nunique = (); # unique transmissions per node (equivalent to FCntUp)
my %nacked = (); # unique acked packets (for confirmed transmissions)
my %ndeliv = (); # unique delivered packets (for non-confirmed transmissions)
my %nperiod = (); 
my %ndc = (); # indicates when a node can transmit again per band (according to dc)
my %npkt = (); # packet size per node
my %ntotretr = (); # number of retransmissions per node (total)
my %nlast_ch = (); # last transmission time
my %ndeliv_seq = (); # 1 = this FCntUp has not yet been counted as delivered
my %nuplink_fcnt_history = (); # ChirpStack-style ADR history: last 20 received unique FCntUp values per ED

# gw attributes
my %gcoords = (); # gw coordinates
my @gw_ids = (); # just for iterations
my %gunavailability_d = (); # unavailable gw time due to downlinks
my %gunavailability_u = (); # unavailable gw time due to uplinks
my %gdc = (); # gw duty cycle (1% uplink channel is used for RX1, 10% downlink channel is used for RX2)
my %gresponses = (); # acks carried out per gw
my %gdest = (); # [node, downlink start, uplink SF, RX1/2, channel, power index, NbTrans, uplink FCntUp]
my %gdublicate = (); # exists if it's a double gw
my %gtime = (); # gateway downlink time per band or channel

# LoRa PHY and LoRaWAN parameters
my @sensis = ([7,-124,-122,-116], [8,-127,-125,-119], [9,-130,-128,-122], [10,-133,-130,-125], [11,-135,-132,-128], [12,-137,-135,-129]); # sensitivities per SF/BW (SX1262)
my @gw_sensis = ([7,-127,-122,-116], [8,-129,-125,-119], [9,-132.5,-128,-122], [10,-135.5,-130,-125], [11,-138,-132,-128], [12,-141,-135,-129]); # SX1302/3 for BW125
my @thresholds = ([1,-8,-9,-9,-9,-9], [-11,1,-11,-12,-13,-13], [-15,-13,1,-13,-14,-15], [-19,-18,-17,1,-17,-18], [-22,-22,-21,-20,1,-20], [-25,-25,-25,-24,-23,1]); # capture effect power thresholds per SF[SF] for non-orthogonal transmissions
my $margin = 5;
my $var = 3.57; # variance
my ($dref, $Lpld0, $gamma) = (40, 110, 2.08); # attenuation model parameters
my $noise = -90; # noise level for snr calculation
my $bw125 = 125000; # channel bandwidth
my $cr = 1; # Coding Rate
my $volt = 3.5; # avg voltage
my $Ptx_gw = 25; # gateway tx power (dBm)
my @Ptx_l = (2, 5, 8, 11, 14, 17, 20); # dBm
my @Ptx_w = (12*$volt, 20*$volt, 32*$volt, 51*$volt, 76*$volt, 90*$volt, 105*$volt); # Ptx cons. for 2, 5, 8, 11, 14, 17, and 20dBm (mA * V = mW)
my $Prx_w = 46 * $volt;
my $Pidle_w = 30 * $volt; # this is actually the consumption of the microcontroller in idle mode
# Frequency plan is selected by JSON; positional input defaults to EU868.
# ------ EU868 ------- #
my @channels = (868100000, 868300000, 868500000, 867100000, 867300000, 867500000, 867700000, 867900000); # TTN channels
my @bands = ("48", "47");
my %band = (868100000=>"48", 868300000=>"48", 868500000=>"48", 867100000=>"47", 867300000=>"47", 867500000=>"47", 867700000=>"47", 867900000=>"47"); # band name per channel (all of them with 1% duty cycle)
my $rx2sf = $configured_rx2sf; # SF used for RX2 (LoRaWAN default = SF12, TTN uses SF9)
my $rx2ch = 869525000; # channel used for RX2 (LoRaWAN default = 869.525MHz, TTN uses the same)
my $dutycycle = 99; # airtime multiplier for low duty cycle uplink transmissions (99 = 1% duty cycle, 9 = 10%)
# ------ US915 ------- #
my %uplink_ch_index = ();
if ($fplan eq "US915"){
	@channels = (903900000, 904100000, 904300000, 904500000, 904700000, 904900000, 905100000, 905300000);
	%uplink_ch_index = (903900000=>0, 904100000=>1, 904300000=>2, 904500000=>3, 904700000=>4, 904900000=>5, 905100000=>6, 905300000=>7); # key=uplink channel, value=index of uplink channel in @channels
	$rx2ch = 923300000; # channel used for RX2
}
if (($fplan eq "EU868") && ($number_of_bands == 1)){
	@channels = grep { $band{$_} eq "48" } @channels;
	@bands = ("48");
}
if ($lrfhss) {
	if ($fplan eq 'US915') {
		@channels = map { 903000000 + $_ * 1600000 } 0 .. 7;
		%uplink_ch_index = map { $channels[$_] => $_ } 0 .. 7;
	} elsif ($lr_profile->{bandwidth} == 336000) {
		# Fully contained in bands 48 (868.0-868.6) and 47 (865-868).
		@channels = (868300000, ($number_of_bands == 2 ? (866900000, 867300000, 867700000) : ()));
		%band = (868300000 => "48", 866900000 => "47", 867300000 => "47", 867700000 => "47");
	}
}
my $bw500 = 500000;
my $window_bw = ($lrfhss && $fplan eq "US915") ? $bw500 : $bw125;
my @channels_d = (923300000, 923900000, 924500000, 925100000, 925700000, 926300000, 926900000, 927500000); # 8x500kHz RX1 downlink channels (all SFs)

# packet specific parameters
my @fpl = (222, 222, 115, 51, 51, 51); # max uplink frame payload per SF (bytes)
@fpl = (222, 125, 53, 11) if ($fplan eq "US915"); # max uplink frame payload per DR(4-0) (bytes)
my $preamble = 8; # in symbols
my $H = 0; # header 0/1
my $hcrc = 0; # HCRC bytes
my $CRC = 1; # 0/1
my $mhdr = 1; # MAC header (bytes)
my $mic = 4; # MIC bytes
my $fhdr = 7; # frame header without fopts
my $linkadr_req = 5; # LinkADRReq: CID (1 B) + payload (4 B)
my $linkadr_ans = 2; # LinkADRAns: CID (1 B) + status (1 B)
my $txdc = 1; # Fopts option for the TX duty cycle (1 Byte)
my $fport_u = 1; # 1B for FPort for uplink
my $fport_d = 0; # 0B for FPort for downlink (commands are included in Fopts, acks have no payload)
my $overhead_u = $mhdr+$mic+$fhdr+$fport_u+$hcrc; # LoRa+LoRaWAN uplink overhead
my $overhead_d = $mhdr+$mic+$fhdr+$fport_d+$hcrc; # LoRa+LoRaWAN downlink overhead
my %overlaps = (); # handles special packet overlaps 

# simulation parameters
my $confirmed_perc = $configured_with_ack; # fraction of nodes that require confirmed transmissions (0=none, 1=all)
my $full_collision = 1; # take into account non-orthogonal SF transmissions or not
my $max_retr = $configured_max_retr; # max number of retransmissions per packet (default value = 1)
my $period = 3600/$packets_per_hour; # time period between transmissions
my $sim_time = $simulation_time_h*3600; # given simulation time
my $debug = as_int(config_value(\%CONFIG, "debug", 0), "debug"); # enable debug mode
my $sim_end = 0;
my ($terrain, $norm_x, $norm_y) = (0, 0, 0); # terrain side, normalised terrain side
my $start_time = time; # just for statistics
my $dropped = 0; # number of dropped packets (for confirmed traffic)
my $dropped_unc = 0; # number of dropped packets (for unconfirmed traffic)
my $total_trans = 0; # number of transm. packets
my $total_retrans = 0; # number of confirmed re-transmission packets
my $total_nbtrans_repetitions = 0; # extra unconfirmed copies due to NbTrans
my $no_rx1 = 0; # no gw was available in RX1
my $no_rx2 = 0; # no gw was available in RX1 or RX2
my $picture = as_int(config_value(\%CONFIG, "picture", 1), "picture"); # generate an energy consumption or a PRR map (see line after stats)
my $fixed_packet_rate = 1; # send packets periodically with a fixed rate (=1) or at random (=0)
my $total_down_time = 0; # total downlink time
my $avg_sf = 0;
my @sf_distr = (0, 0, 0, 0, 0, 0);
my $fixed_packet_size = 1; # all nodes have the same packet size defined in @fpl (=1) or a randomly selected (=0)
my $packet_size = $configured_pkt_size; # default packet size if fixed_packet_size=1 or avg packet size if fixed_packet_size=0 (Bytes)
my $packet_size_distr = "normal"; # uniform / normal (applicable if fixed_packet_size=0)
my $avg_pkt = 0; # actual average packet size
my %sorted_t = (); # keys = channels, values = list of nodes
my @recents = (); # used in auto_simtime
my $auto_simtime = 0; # 1 = the simulation will automatically stop (useful when sim_time>>10000)
my %sf_retrans = (); # number of retransmissions per SF
my $adr_on = $configured_adr; # ADR is used or not (=0)
my $double_gws = $configured_double_gws; # enable 8x2 channel gateways

# ChirpStack default NbTrans adaptation parameters.
# ChirpStack requires 20 received unique uplinks, estimates packet loss from
# gaps in FCntUp, and maps (packet-loss range, current NbTrans) through:
#   <5%   : [1,1,2]
#   <10%  : [1,2,3]
#   <30%  : [2,3,3]
#   >=30% : [3,3,3]
my $nbtrans_history_len = 20;

# application server
my $policy = 1; # gateway selection policy for downlink traffic
die "# The least busy policy cannot be used with US915 frequencies" if (($policy == 3) && ($fplan eq "US915"));
my %appacked = (); # counts the number of acked packets per node
my %appsuccess = (); # counts the number of packets that received from at least one gw per node
my %nogwavail = (); # counts how many time no gw was available (keys = nodes)
my %powers = (); # contains last 10 received powers per node

# precomputations
my %pl_ng; # path-loss (without shadowing) per node-gw pair
my %dist_ng; # node-gw distance 

read_data(); # read terrain file
my $lr_radio = $lrfhss ? LoRaWAN::LRFHSS->new(
	power => \&lr_received_power,
	capture => sub {
		my ($victim, $interferer) = @_;
		return $thresholds[$victim->{sf}-7][$interferer->{sf}-7]
			unless $victim->{profile} || $interferer->{profile};
		return $lr_capture;
	},
) : undef;
my $lr_completed;


# first transmission
my @init_trans = ();
foreach my $n (sort {$a <=> $b} keys %ncoords){
	my $start = random_uniform(1, 0, $period);
	my $sf = $lrfhss ? init_lrfhss_node($n) : min_sf($n);
	$avg_sf += $sf;
	$avg_pkt += $npkt{$n};
	my $airt = uplink_airtime($sf, $npkt{$n});
	my $stop = $start + $airt;
	print "# $n will transmit from $start to $stop (SF $sf)\n" if ($debug == 1 && !$lrfhss);
	$nunique{$n} = 1;
	$ndeliv_seq{$n}{$nunique{$n}} = 1;
	my $ch = $channels[rand @channels];
	push (@init_trans, [$n, $start, $stop, $ch, $sf, $nunique{$n}, undef, undef, undef, $npkt{$n}, $nptx{$n}]);
	$nconsumption{$n} += $airt * $Ptx_w[$nptx{$n}] + $airt * $Pidle_w;
	$total_trans += 1;
	$ndc{$n}{$band{$ch}} = $stop + $dutycycle*$airt if ($fplan ne "US915");
}

# sort transmissions in ascending order
foreach my $t (sort { $a->[1] <=> $b->[1] } @init_trans){
	my ($n, $sta, $end, $ch, $sf, $nuni) = @$t;
	push (@{$sorted_t{$ch}}, $t);
}
undef @init_trans;

# main loop
while (1){
	print "-------------------------------\n" if ($debug == 1);
	my $event = next_transmission();
	last unless $event;
	my ($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $sel_seq) = @$event;
	if (exists $ncoords{$sel}){ # just a progress trick
		if ($sel == 1){
			$| = 1;
			printf STDERR "%.2f%%\r", 100*$sel_end/$sim_time;
		}
	}
	last if (!$lrfhss && $sel_sta > $sim_time);
	print "# grabbed $sel, transmission from $sel_sta -> $sel_end\n" if ($debug == 1);
	$sim_end = $sel_end;
	if ($auto_simtime == 1){
		my $nu = (sum values %nunique);
		$nu = 1 if ($nu == 0);
		if (scalar @recents < 100){
			push(@recents, (sum values %ndeliv)/$nu);
			#printf "stddev = %.5f\n", stddev(\@recents);
		}else{
			if (stddev(\@recents) < 0.0001){ # you can fine-tune that
				print "### Continuing the simulation will not considerably affect the result! ###\n";
				last;
			}
			shift(@recents);
		}
	}
	
	if ($sel =~ /^[0-9]/){ # if the packet is an uplink transmission
		
		# For unconfirmed traffic, a LinkADRReq may have arrived after this event was scheduled.
		# Add LinkADRAns to the first subsequently transmitted uplink at execution time.
		if (!$lrfhss && ($nconfirmed{$sel} == 0) && ($nresponse{$sel} == 1)){
			my $old_at = $sel_end - $sel_sta;
			my $new_at = airtime($sel_sf, $bw125, $npkt{$sel}+$linkadr_ans);
			if ($new_at > $old_at){
				$nconsumption{$sel} += ($new_at-$old_at) * ($Ptx_w[$nptx{$sel}] + $Pidle_w);
				$sel_end = $sel_sta + $new_at;
				$ndc{$sel}{$band{$sel_ch}} = $sel_end + $dutycycle*$new_at if ($fplan ne "US915");
			}
			$nresponse{$sel} = 0;
			print "# $sel includes LinkADRAns in FCntUp=$sel_seq\n" if ($debug == 1);
		}

		my $gw_rc = $lrfhss ? $lr_completed->{received} : node_col($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $sel_seq); # check collisions and return a list of gws that received the uplink pkt
		my $rwindow = 0;
		my $failed = 0;
		$nlast_ch{$sel} = $sel_ch;
		# Keep the last 10 best-gateway SNR samples at the current TX power.
		my $max_snr = -999;
		foreach my $g (@$gw_rc){
			my $snr = @$g[1] - $noise;
			$max_snr = $snr if ($snr > $max_snr);
		}
		push (@{$powers{$sel}}, $max_snr) if ($max_snr > -999);
		shift @{$powers{$sel}} if (scalar @{$powers{$sel}} > 10);
		if ((scalar @$gw_rc > 0) && ($nconfirmed{$sel} == 1)){ # if at least one gateway received the pkt -> successful transmission
			my ($new_ptx, $new_index, $new_nbtrans) = (undef, -1, -1);
			if ((scalar @{$powers{$sel}} == 10) && ($adr_on == 1)){
				($new_ptx, $new_index, $new_nbtrans) = adr($sel, $sel_sf);
			}
			# Preserve the existing confirmed retransmission model; NbTrans is used for unconfirmed traffic.
			$new_nbtrans = -1;
			my $adr_cmd_bytes = ($new_index != -1) ? $linkadr_req : 0;
			if (exists $ndeliv_seq{$sel}{$sel_seq}){
				$ndeliv{$sel} += 1;
				delete $ndeliv_seq{$sel}{$sel_seq};
			}
			$appsuccess{$sel} += 1;
			printf "# $sel 's transmission received by %d gateway(s) (channel $sel_ch)\n", scalar @$gw_rc if ($debug == 1);
			# now we have to find which gateway (if any) can transmit an ack in RX1 or RX2
			# check RX1
			my $sel_gw = gs_policy($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $gw_rc, 1, $adr_cmd_bytes);
			if (defined $sel_gw){
				schedule_downlink($sel_gw, $sel, $sel_sf, $sel_ch, $sel_seq, $sel_end, 1, $new_index, $new_nbtrans);
			}else{
				# check RX2
				$no_rx1 += 1;
				$sel_gw = gs_policy($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $gw_rc, 2, $adr_cmd_bytes);
				if (defined $sel_gw){
					schedule_downlink($sel_gw, $sel, $sel_sf, $rx2ch, $sel_seq, $sel_end, 2, $new_index, $new_nbtrans);
				}else{
					$no_rx2 += 1;
					print "# no gateway is available\n" if ($debug == 1);
					$nogwavail{$sel} += 1;
					$failed = 1;
				}
			}
		}elsif ((scalar @$gw_rc > 0) && ($nconfirmed{$sel} == 0)){ # successful unconfirmed transmission
			# Several NbTrans copies may carry the same FCntUp
			# count delivery and update the NS success history only on the first successfully decoded copy
			if (exists $ndeliv_seq{$sel}{$sel_seq}){
				record_uplink_fcnt($sel, $sel_seq);
				$ndeliv{$sel} += 1;
				delete $ndeliv_seq{$sel}{$sel_seq};
			}
			$appsuccess{$sel} += 1;
			printf "# $sel received by %d gateway(s) (channel $sel_ch, FCntUp=$sel_seq, NbTrans copy $nnbtrans_count{$sel}/$nnbtrans{$sel})\n", scalar @$gw_rc if ($debug == 1);

			# LinkADRReq may update TX power and/or NbTrans. SF stays fixed by min_sf()
			if ((scalar @{$powers{$sel}} == 10) && ($adr_on == 1)){
				my ($new_ptx, $new_index, $new_nbtrans) = adr($sel, $sel_sf);
				if (($new_index != -1) || ($new_nbtrans != -1)){
					my $sel_gw = gs_policy($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $gw_rc, 1, $linkadr_req);
					if (defined $sel_gw){
						schedule_downlink($sel_gw, $sel, $sel_sf, $sel_ch, $sel_seq, $sel_end, 1, $new_index, $new_nbtrans);
					}else{
						$sel_gw = gs_policy($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $gw_rc, 2, $linkadr_req);
						if (defined $sel_gw){
							schedule_downlink($sel_gw, $sel, $sel_sf, $rx2ch, $sel_seq, $sel_end, 2, $new_index, $new_nbtrans);
						}else{
							print "# no LinkADRReq could be sent to $sel\n" if ($debug == 1);
						}
					}
				}
			}
		}else{ # non-successful transmission
			$failed = 1;
		}
		if ($nconfirmed{$sel} == 1){
			# Existing confirmed-uplink retransmission mechanism, kept separate from NbTrans.
			if ($failed == 1){
				my $at = 0;
				my $new_trans = 0;
				my $new_ch = $channels[rand @channels];
				$new_ch = $channels[rand @channels] while (($new_ch == $sel_ch) && (scalar @channels > 1));
				$sel_ch = $new_ch;
				if ($nretransmissions{$sel} < $max_retr){
					$nretransmissions{$sel} += 1;
					$sf_retrans{$sel_sf} += 1;
				}else{
					$dropped += 1;
					$ntotretr{$sel} += $nretransmissions{$sel};
					$nretransmissions{$sel} = 0;
					$new_trans = 1;
					print "# $sel 's packet lost!\n" if ($debug == 1);
				}
				if ($lrfhss) {
					$nconsumption{$sel} += lr_rx_energy($sel_sf);
				} else {
					$nconsumption{$sel} += (2-($preamble+4.25)*(2**$sel_sf)/$window_bw)*$Pidle_w + ($preamble+4.25)*(2**$sel_sf)/$window_bw * ($Prx_w + $Pidle_w);
					$nconsumption{$sel} += ($preamble+4.25)*(2**$rx2sf)/$window_bw * ($Prx_w + $Pidle_w);
				}
				$at = uplink_airtime($sel_sf, $npkt{$sel});
				if ($new_trans == 0){
					$sel_sta = $sel_end + 2 + 1 + rand(2);
				}else{
					$sel_sta = $sel_end + 2 + $nperiod{$sel} + rand(1);
				}
				$sel_sta = max($sel_sta, $sel_end + 2 + ($preamble+4.25)*(2**$rx2sf)/$window_bw) if $lrfhss;
				if ($fplan ne "US915"){
					if ($sel_sta < $ndc{$sel}{$band{$sel_ch}}){
						print "# warning! transmission will be postponed due to duty cycle restrictions!\n" if ($debug == 1);
						$sel_sta = $ndc{$sel}{$band{$sel_ch}};
					}
				}
				$sel_end = $sel_sta+$at;
				my $i = 0;
				foreach my $el (@{$sorted_t{$sel_ch}}){
					my ($n, $sta, $end, $ch_, $sf_, $seq) = @$el;
					last if ($sta > $sel_sta);
					$i += 1;
				}
				if (($new_trans == 1) && ($sel_sta < $sim_time)){
					$nunique{$sel} += 1;
					$ndeliv_seq{$sel}{$nunique{$sel}} = 1;
				}
				$total_trans += 1 if ($sel_sta < $sim_time);
				$total_retrans += 1 if (!$new_trans && $sel_sta < $sim_time);
				splice(@{$sorted_t{$sel_ch}}, $i, 0, [$sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $nunique{$sel}, undef, undef, undef, $npkt{$sel}, $nptx{$sel}]);
				print "# $sel, new transmission at $sel_sta -> $sel_end\n" if ($debug == 1);
				$nconsumption{$sel} += $at * $Ptx_w[$nptx{$sel}] + $at * $Pidle_w if $sel_sta < $sim_time;
				$ndc{$sel}{$band{$sel_ch}} = $sel_end + $dutycycle*$at if ($fplan ne "US915");
			}
		}else{
			# An unconfirmed uplink always opens RX1/RX2, even if no gateway decoded it
			if ($lrfhss) {
				$nconsumption{$sel} += lr_rx_energy($sel_sf);
			} else {
				$nconsumption{$sel} += (2-($preamble+4.25)*(2**$sel_sf)/$window_bw)*$Pidle_w + ($preamble+4.25)*(2**$sel_sf)/$window_bw * ($Prx_w + $Pidle_w);
				$nconsumption{$sel} += ($preamble+4.25)*(2**$rx2sf)/$window_bw * ($Prx_w + $Pidle_w);
			}

			my $repeat_same_fcnt = ($nnbtrans_count{$sel} < $nnbtrans{$sel}) ? 1 : 0;
			if (($repeat_same_fcnt == 0) && (exists $ndeliv_seq{$sel}{$sel_seq})){
				# No copy of this FCntUp reached any GW. ChirpStack does not append an
				# explicit failure to ADR history; the loss is inferred later from the
				# FCntUp gap when a subsequent unique uplink is received.
				$dropped_unc += 1;
				delete $ndeliv_seq{$sel}{$sel_seq};
				print "# $sel 's unconfirmed packet FCntUp=$sel_seq lost after $nnbtrans{$sel} transmission(s)!\n" if ($debug == 1);
			}

			my $new_ch = $channels[rand @channels];
			$new_ch = $channels[rand @channels] while (($new_ch == $sel_ch) && (scalar @channels > 1));
			$sel_ch = $new_ch;
			my $next_seq = $sel_seq;
			if ($repeat_same_fcnt == 1){
				$nnbtrans_count{$sel} += 1;
				$sel_sta = $sel_end + 2 + 1 + rand(2); # after RX2 + random delay
			}else{
				$nnbtrans_count{$sel} = 1;
				$sel_sta = $sel_end + $nperiod{$sel} + rand(1);
			}

			$sel_sta = max($sel_sta, $sel_end + 2 + airtime($rx2sf, $window_bw, $overhead_d+$linkadr_req)) if $lrfhss;
			my $at = uplink_airtime($sel_sf, $npkt{$sel});
			if ($fplan ne "US915"){
				if ($sel_sta < $ndc{$sel}{$band{$sel_ch}}){
					print "# warning! transmission will be postponed due to duty cycle restrictions!\n" if ($debug == 1);
					$sel_sta = $ndc{$sel}{$band{$sel_ch}};
				}
			}
			$sel_end = $sel_sta + $at;
			my $i = 0;
			foreach my $el (@{$sorted_t{$sel_ch}}){
				my ($n, $sta, $end, $ch_, $sf_, $seq) = @$el;
				last if ($sta > $sel_sta);
				$i += 1;
			}
			if ($sel_sta < $sim_time){
				if (!$repeat_same_fcnt) {
					$nunique{$sel} += 1;
					$next_seq = $nunique{$sel};
					$ndeliv_seq{$sel}{$next_seq} = 1;
				} else {
					$total_nbtrans_repetitions += 1;
				}
				my $scheduled_energy = $at * $Ptx_w[$nptx{$sel}] + $at * $Pidle_w;
				my $prev_dc = -1;
				$prev_dc = $ndc{$sel}{$band{$sel_ch}} if ($fplan ne "US915");
				# Tuple fields 6..8 support cancellation; 9..10 snapshot PHY bytes/TX power for LR-FHSS.
				splice(@{$sorted_t{$sel_ch}}, $i, 0, [$sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $next_seq, $scheduled_energy, $prev_dc, $repeat_same_fcnt, $npkt{$sel}, $nptx{$sel}]);
				$total_trans += 1;
				$nconsumption{$sel} += $scheduled_energy;
				$ndc{$sel}{$band{$sel_ch}} = $sel_end + $dutycycle*$at if ($fplan ne "US915");
				print "# $sel, " . ($repeat_same_fcnt ? "NbTrans copy $nnbtrans_count{$sel}/$nnbtrans{$sel} of FCntUp=$next_seq" : "new FCntUp=$next_seq") . " at $sel_sta -> $sel_end\n" if ($debug == 1);
			}
		}
		foreach my $g (@gw_ids){
			delete $surpressed{$sel}{$g}{$sel_seq};
		}
		
		
	}else{ # if the packet is a gw transmission
		
		
		$sel =~ s/[0-9].*//; # keep only the letter(s)
		# remove the unnecessary tuples from gw unavailability
		my @indices = ();
		my $index = 0;
		foreach my $tuple (@{$gunavailability_d{$sel}}){
			my ($sta, $end) = @$tuple;
			push (@indices, $index) if ($end < $sel_sta);
			$index += 1;
		}
		for (sort {$b<=>$a} @indices){
			splice @{$gunavailability_d{$sel}}, $_, 1;
		}
		
		# look for the examined transmission in gdest, get some info, and then remove it 
		my $failed = 0;
		$index = 0;
		# ($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $sel_seq) information we already have
		# sel_sf = SF of the downlink, sf = SF of the corresponding uplink in gdest
		my ($dest, $st, $sf, $rwindow, $ch, $pow, $nbtrans, $uplink_seq);
		foreach my $tup (@{$gdest{$sel}}){
			my ($dest_, $st_, $sf_, $rwindow_, $ch_, $p_, $nb_, $seq_) = @$tup;
			if (($st_ == $sel_sta) && ($ch_ == $sel_ch)){
				($dest, $st, $sf, $rwindow, $ch, $pow, $nbtrans, $uplink_seq) = ($dest_, $st_, $sf_, $rwindow_, $ch_, $p_, $nb_, $seq_);
				last;
			}
			$index += 1;
		}
		splice @{$gdest{$sel}}, $index, 1;
		if ($lrfhss) {
			$failed = @{$lr_completed->{received}} ? 0 : 1;
			# Replace the empty windows prepaid by unconfirmed uplinks with the
			# actual LoRa downlink listening time, once per transmission attempt.
			$nconsumption{$dest} -= lr_rx_energy($sf) unless $nconfirmed{$dest};
			$nconsumption{$dest} += lr_rx_energy($sf, $rwindow, $sel_end-$sel_sta, !$failed);
			print "# LR-FHSS LoRa downlink to $dest " . ($failed ? "lost" : "received") . " (SF$sel_sf)\n" if $debug;
		} else {
		# check if the transmission can reach the node
		my $G = random_normal(1, 0, 1);
		my $d = $dist_ng{$dest}{$sel};
		my $Xs  = $G * $var;
		my $prx = $Ptx_gw - $pl_ng{$dest}{$sel} - $Xs;
		my $cb = $bw125;
		$cb = $bw500 if ($fplan eq "US915");
		if ($prx < $sensis[$sel_sf-7][bwconv($cb)]){
			print "# ack didn't reach node $dest\n" if ($debug == 1);
			$failed = 1;
		}
		# check if transmission time overlaps with other transmissions
		foreach my $tr (@{$sorted_t{$ch}}){
			my ($n, $sta, $end, $ch_, $sf_, $seq) = @$tr;
			last if ($sta > $sel_end);
			$n =~ s/[0-9].*// if ($n =~ /^[A-Z]/);
			next if (($n eq $sel) || ($end < $sel_sta)); # skip non-overlapping transmissions
			if ( (($sel_sta >= $sta) && ($sel_sta <= $end)) || (($sel_end <= $end) && ($sel_end >= $sta)) || (($sel_sta == $sta) && ($sel_end == $end)) ){
				push(@{$overlaps{$sel}}, [$n, $G, $sf_]); # put in here all overlapping transmissions
				push(@{$overlaps{$n}}, [$sel, $G, $sel_sf]); # check future possible collisions with those transmissions
			}
		}
		my %examined = ();
		foreach my $ng (@{$overlaps{$sel}}){
			my ($n, $G_, $sf_) = @$ng;
			next if (exists $examined{$n});
			$examined{$n} = 1;
			my $overlap = 1;
			# SF
			if ($sf_ == $sel_sf){
				$overlap += 2;
			}
			# power 
			my $d_ = 0;
			my $p = 0;
			my $prx_ = 0;
			if ($n =~ /^[0-9]/){
				# there is no precomputed dist/path-loss for node-to-node
				$d_ = distance($ncoords{$dest}[0], $ncoords{$n}[0], $ncoords{$dest}[1], $ncoords{$n}[1]);
				$prx_ = $Ptx_l[$nptx{$n}] - ($Lpld0 + 10*$gamma * log10($d_/$dref)) - $G_*$var;
			}else{
				$prx_ = $Ptx_gw - $pl_ng{$dest}{$n} - $G_*$var;
			}
			if ($overlap == 3){
				if ((abs($prx - $prx_) <= $thresholds[$sel_sf-7][$sf_-7]) ){ # both collide
					$failed = 1;
					print "# ack collided together with $n at node $sel\n" if ($debug == 1);
				}
				if (($prx_ - $prx) > $thresholds[$sel_sf-7][$sf_-7]){ # n suppressed sel
					$failed = 1;
					print "# ack surpressed by $n at node $dest\n" if ($debug == 1);
				}
				if (($prx - $prx_) > $thresholds[$sf_-7][$sel_sf-7]){ # sel suppressed n
					print "# $n surpressed by $sel at node $dest\n" if ($debug == 1);
				}
			}
			if (($overlap == 1) && ($full_collision == 1)){ # non-orthogonality
				if (($prx - $prx_) > $thresholds[$sel_sf-7][$sf_-7]){
					if (($prx_ - $prx) <= $thresholds[$sf_-7][$sel_sf-7]){
						print "# $n surpressed by $sel at node $dest\n" if ($debug == 1);
					}
				}else{
					if (($prx_ - $prx) > $thresholds[$sf_-7][$sel_sf-7]){
						$failed = 1;
						print "# ack surpressed by $n at node $dest\n" if ($debug == 1);
					}else{
						$failed = 1;
						print "# ack collided together with $n at node $dest\n" if ($debug == 1);
					}
				}
			}
		}
		}

		my $new_trans = 0;
		if ($failed == 0){
			if ($nconfirmed{$dest} == 1){
				print "# ack successfully received, $dest 's transmission has been acked\n" if ($debug == 1);
				$ntotretr{$dest} += $nretransmissions{$dest};
				$nacked{$dest} += 1;
				$nretransmissions{$dest} = 0;
				$new_trans = 1;
			}
			my $cb = $bw125;
			$cb = $bw500 if ($fplan eq "US915");
			if (!$lrfhss && $rwindow == 2){ # also count the RX1 window
				$nconsumption{$dest} += (2-($preamble+4.25)*(2**$sf)/$cb)*$Pidle_w + ($preamble+4.25)*(2**$sf)/$cb * ($Prx_w + $Pidle_w);
			}
			my $extra_bytes = 0;
			my $has_linkadr_req = (($pow != -1) || ($nbtrans != -1)) ? 1 : 0;
			if ($has_linkadr_req == 1){
				$extra_bytes = $linkadr_req;
				$nresponse{$dest} = 1;
			}
			if ($pow != -1){
				$nptx{$dest} = $pow;
				# Samples taken before this change describe the old TX power.
				@{$powers{$dest}} = ();
				print "# transmit power of $dest is set to $Ptx_l[$pow]dBm\n" if ($debug == 1);
			}
			if ($nbtrans != -1){
				$nnbtrans{$dest} = $nbtrans;
				print "# NbTrans of $dest is set to $nbtrans\n" if ($debug == 1);
			}
			$nconsumption{$dest} += airtime($sel_sf, $cb, $overhead_d+$extra_bytes) * ($Prx_w + $Pidle_w) unless $lrfhss;
			if ($nconfirmed{$dest} == 0){
				# Any valid Class-A downlink stops remaining copies of the same FCntUp.
				my $cancelled = remove_scheduled_uplink($dest, $uplink_seq);
				if ($cancelled > 0){
					schedule_unconfirmed_after_downlink($dest, $sf, $st-$rwindow, $sel_end);
				}
			}
		}else{ # ack was not received
			if ($nconfirmed{$dest} == 1){
				if ($nretransmissions{$dest} < $max_retr){
					$nretransmissions{$dest} += 1;
					$sf_retrans{$sf} += 1;
				}else{
					$dropped += 1;
					$ntotretr{$dest} += $nretransmissions{$dest};
					$nretransmissions{$dest} = 0;
					$new_trans = 1;
					print "# $dest 's packet lost (no ack received)!\n" if ($debug == 1);
				}
			}
			my $cb = $bw125;
			$cb = $bw500 if ($fplan eq "US915");
			unless ($lrfhss) {
			$nconsumption{$dest} += (2-($preamble+4.25)*(2**$sf)/$cb)*$Pidle_w + ($preamble+4.25)*(2**$sf)/$cb * ($Prx_w + $Pidle_w);
			$nconsumption{$dest} += ($preamble+4.25)*(2**$rx2sf)/$cb * ($Prx_w + $Pidle_w);
			}
		}
		@{$overlaps{$sel}} = ();
		if ($nconfirmed{$dest} == 1){
			# plan next transmission
			do{
				$ch = $channels[rand @channels]; 
			} while (($ch == $nlast_ch{$dest}) && @channels > 1);
			my $extra_bytes = 0;
			if ($nresponse{$dest} == 1){
				$extra_bytes = $linkadr_ans;
				$nresponse{$dest} = 0;
			}
			my $at = uplink_airtime($sf, $npkt{$dest}+$extra_bytes);
			my $new_start = $sel_sta - $rwindow + $nperiod{$dest} + rand(1);
			$new_start = $sel_sta - $rwindow + 2 + 1 + rand(2) if ($failed == 1 && $new_trans == 0);
			if ($lrfhss) {
				$new_start = max($new_start, $sel_end + 0.001);
				$new_start = max($new_start, $sel_sta-$rwindow + 2 + ($preamble+4.25)*(2**$rx2sf)/$window_bw) if $failed;
			}
			if ($fplan ne "US915"){
				if ($new_start < $ndc{$dest}{$band{$ch}}){
					print "# warning! transmission will be postponed due to duty cycle restrictions!\n" if ($debug == 1);
					$new_start = $ndc{$dest}{$band{$ch}};
				}
			}
			if (($new_trans == 1) && ($new_start < $sim_time)){ # do not count transmissions that exceed the simulation time
				$nunique{$dest} += 1;
				$ndeliv_seq{$dest}{$nunique{$dest}} = 1;
			}
			my $new_end = $new_start + $at;
			my $i = 0;
			foreach my $el (@{$sorted_t{$ch}}){
				my ($n, $sta, $end, $ch_, $sf_, $seq) = @$el;
				last if ($sta > $new_start);
				$i += 1;
			}
			splice(@{$sorted_t{$ch}}, $i, 0, [$dest, $new_start, $new_end, $ch, $sf, $nunique{$dest}, undef, undef, undef, $npkt{$dest}+$extra_bytes, $nptx{$dest}]);
			$total_trans += 1 if ($new_start < $sim_time); # do not count transmissions that exceed the simulation time
			$total_retrans += 1 if (($failed == 1) && !$new_trans && ($new_start < $sim_time));
			print "# $dest, new transmission at $new_start -> $new_end\n" if ($debug == 1);
			$nconsumption{$dest} += $at * $Ptx_w[$nptx{$dest}] + $at * $Pidle_w if ($new_start < $sim_time);
			$ndc{$dest}{$band{$ch}} = $new_end + $dutycycle*$at if ($fplan ne "US915");
		}
	}
}
# print "---------------------\n";

my $avg_cons = (sum values %nconsumption)/(scalar keys %nconsumption);
my $min_cons = min values %nconsumption;
my $max_cons = max values %nconsumption;
my $finish_time = time;
printf "Simulation time = %.3f secs\n", $sim_end;
printf "Avg node consumption = %.5f J\n", $avg_cons/1000;
printf "Min node consumption = %.5f J\n", $min_cons/1000;
printf "Max node consumption = %.5f J\n", $max_cons/1000;
print "Total number of transmissions = $total_trans\n";
print "Total number of confirmed re-transmissions = $total_retrans\n" if ($confirmed_perc > 0);
print "Total NbTrans repetitions = $total_nbtrans_repetitions\n";
printf "Avg applied NbTrans = %.3f\n", (sum values %nnbtrans)/(scalar keys %nnbtrans);
printf "Total number of unique transmissions = %d\n", (sum values %nunique);
printf "Stdv of unique transmissions = %.2f\n", stddev(values %nunique);
printf "Total packets received = %d\n", (sum values %appsuccess); # total packets received
printf "Total unique packets acknowledged = %d\n", (sum values %nacked);
print "Total confirmed packets dropped = $dropped\n";
print "Total unconfirmed packets dropped = $dropped_unc\n";
printf "Confirmed Packet Delivery Ratio (unique) = %.5f\n", (sum values %nacked)/(sum values %nunique) if ($confirmed_perc > 0); # unique packets acked / unique packets transmitted
printf "Packet Delivery Ratio = %.5f\n", (sum values %ndeliv)/(sum values %nunique); # total unique packets received / total unique packets transmitted
printf "Packet Reception Ratio = %.5f\n", (sum values %appsuccess)/$total_trans; # total packets received / total packets transmitted
my @fairs = ();
foreach my $n (sort {$a <=> $b} keys %ncoords){
	if ($nconfirmed{$n} == 0){
		push(@fairs, $ndeliv{$n}/$nunique{$n});
	}
}
printf "Unconfirmed uplink fairness = %.3f\n", stddev(\@fairs) if (scalar @fairs > 0); # for unconfirmed traffic
print "-----\n";
print "No GW available in RX1 = $no_rx1 times\n";
print "No GW available in RX1 or RX2 = $no_rx2 times\n";
print "Total downlink time = $total_down_time sec\n";
foreach my $g (@gw_ids){
	print "GW $g sent out $gresponses{$g} acks and commands\n";
	if ($fplan eq "EU868"){
		foreach my $bnd (@bands){
			printf "\t - Total duty cycle in band $bnd: %.2f%%\n", $gtime{$g}{$bnd}*100/$sim_end;
		}
	}elsif ($fplan eq "US915"){
		foreach my $ch (@channels_d){
			printf "\t - Total duty cycle in channel %.1f MHz: %.2f%%\n", $ch/1e6, $gtime{$g}{$ch}*100/$sim_end;
		}
	}
	printf "\t - Total duty cycle in RX2 channel: %.2f%%\n", $gtime{$g}{$rx2ch}*100/$sim_end;
}
if (scalar keys %ntotretr > 0){
	@fairs = ();
	my $avgretr = 0;
	foreach my $n (sort {$a <=> $b} keys %ncoords){
		next if ($nconfirmed{$n} == 0);
		push(@fairs, $nacked{$n}/$nunique{$n});
		$avgretr += $ntotretr{$n}/$nunique{$n};
	}
	printf "Downlink fairness = %.3f\n", stddev(\@fairs) if (scalar @fairs > 0);
	printf "Avg number of retransmissions = %.3f\n", $avgretr/(scalar keys %ntotretr);
	printf "Stdev of retransmissions = %.3f\n", (stddev values %ntotretr);
	print "-----\n";
}
if (!$lrfhss) {
for (my $sf=7; $sf<=12; $sf+=1){
	printf "# of nodes with SF%d: %d, Avg retransmissions: %.2f\n", $sf, $sf_distr[$sf-7], $sf_retrans{$sf}/$sf_distr[$sf-7] if ($sf_distr[$sf-7] > 0);
}
printf "Avg SF = %.3f\n", $avg_sf/(scalar keys %ncoords);
} else {
	print "Uplink modulation = LR-FHSS\n";
	print "Frequency plan = $fplan\n";
	print "LR-FHSS data rate = DR$lr_profile->{dr}\n";
	print "LR-FHSS coding rate = $lr_profile->{cr_num}/3\n";
	print "LR-FHSS header replicas = $lr_profile->{headers}\n";
	my $stats = $lr_radio->stats;
	print "LR-FHSS header observations = $stats->{headers}\n";
	print "LR-FHSS lost header observations = $stats->{lost_headers}\n";
	print "LR-FHSS fragment observations = $stats->{fragments}\n";
	print "LR-FHSS lost fragment observations = $stats->{lost_fragments}\n";
}
# printf "Avg packet size = %.3f bytes\n", $avg_pkt/(scalar keys %ncoords); # includes overhead
printf "Script execution time = %.4f secs\n", $finish_time - $start_time;
generate_picture(1) if ($picture == 1); # 0=energy consumption map, 1=PRR map


sub lr_rx_energy {
	my ($rx1_sf, $window, $duration, $success) = @_;
	my $rx1 = ($preamble+4.25) * (2**$rx1_sf) / $window_bw;
	my $rx2 = ($preamble+4.25) * (2**$rx2sf) / $window_bw;
	return 2*$Pidle_w + $rx1*$Prx_w + $rx2*($Prx_w+$Pidle_w) unless $window;
	if ($window == 1) {
		return $Pidle_w + $duration*($Prx_w+$Pidle_w) if $success;
		return 2*$Pidle_w + $duration*$Prx_w + $rx2*($Prx_w+$Pidle_w);
	}
	return 2*$Pidle_w + $rx1*$Prx_w + $duration*($Prx_w+$Pidle_w);
}

sub uplink_airtime {
	my ($sf, $bytes) = @_;
	return $lrfhss ? LoRaWAN::LRFHSS::airtime($lr_profile, $bytes) : airtime($sf, $bw125, $bytes);
}

sub init_lrfhss_node {
	my ($node) = @_;
	# LR-FHSS reachability is evaluated at reception using its own sensitivity.
	# An out-of-range node still transmits and contributes interference/statistics.
	$npkt{$node} = $packet_size + $overhead_u;
	return $lr_profile->{rx1sf};
}

sub lr_received_power {
	my ($frame, $receiver) = @_;
	my $sender = $frame->{sender};
	my $from = exists $ncoords{$sender} ? $ncoords{$sender} : $gcoords{$sender};
	my $to = exists $ncoords{$receiver} ? $ncoords{$receiver} : $gcoords{$receiver};
	my $d = max(0.2, distance($from->[0], $to->[0], $from->[1], $to->[1]));
	return $frame->{tx_power} - ($Lpld0 + 10*$gamma*log10($d/$dref)) - random_normal(1, 0, 1)*$var;
}

sub next_transmission {
	# LoRa retains its packet-start collision model. LR-FHSS merges actual starts
	# with radio completions, so later interference can erase individual hops
	# before reception, ACK scheduling, or packet-delivery statistics are decided.
	while (1) {
		my ($min_ch, $min_t);
		foreach my $ch (sort {$a <=> $b} keys %sorted_t) {
			my $list = $sorted_t{$ch};
			if ($lrfhss) {
				@$list = grep { $_->[0] !~ /^[0-9]/ || $_->[1] < $sim_time } @$list;
			}
			if (!@$list) { delete $sorted_t{$ch}; next; }
			my $t = $list->[0][1];
			if (!defined($min_t) || $t < $min_t) { ($min_ch, $min_t) = ($ch, $t); }
		}
		if ($lrfhss) {
			my $end = $lr_radio->next_end;
			if (defined($end) && (!defined($min_t) || $end <= $min_t)) {
				$lr_completed = $lr_radio->finish_frame;
				return $lr_completed->{event};
			}
		}
		return undef unless defined $min_ch;
		my $event = shift @{$sorted_t{$min_ch}};
		return $event unless $lrfhss;
		my ($sender, $start, $end, $channel, $sf) = @$event;
		my $frame = { event => $event, start => $start, end => $end, sf => $sf };
		if ($sender =~ /^[0-9]/) {
			my $bytes = $event->[9];
			if (!$nconfirmed{$sender} && $nresponse{$sender}) {
				$bytes += $linkadr_ans;
				$nresponse{$sender} = 0;
				print "# $sender includes LinkADRAns in FCntUp=$event->[5]\n" if $debug;
			}
			my $at = uplink_airtime($sf, $bytes);
			# Power may have changed since this transmission was queued.
			$nconsumption{$sender} += $at * ($Ptx_w[$nptx{$sender}] + $Pidle_w)
				- ($end-$start) * ($Ptx_w[$event->[10]] + $Pidle_w);
			$frame->{end} = $event->[2] = $start + $at;
			$ndc{$sender}{$band{$channel}} = $frame->{end} + $dutycycle*$at if $fplan ne 'US915';
			$frame->{sender} = $sender;
			$frame->{tx_power} = $Ptx_l[$nptx{$sender}];
			$frame->{profile} = $lr_profile;
			$frame->{hops} = LoRaWAN::LRFHSS::hops($lr_profile, $bytes, $start, $channel);
			$frame->{receivers} = [@gw_ids];
			$frame->{sensitivity} = $lr_sensitivity;
			printf "# LR-FHSS node %s FCntUp=%d DR%d start=%.6f end=%.6f channel=%d bytes=%d hops=%d\n",
				$sender, $event->[5], $lr_profile->{dr}, $start, $frame->{end}, $channel, $bytes, scalar @{$frame->{hops}} if $debug;
		} else {
			$sender =~ s/[0-9].*//;
			my ($dest) = map { $_->[0] } grep { $_->[1] == $start && $_->[4] == $channel } @{$gdest{$sender}};
			die "Missing scheduled LR-FHSS downlink destination\n" unless defined $dest;
			my $bw = $fplan eq 'US915' ? $bw500 : $bw125;
			$frame->{sender} = $sender;
			$frame->{tx_power} = $Ptx_gw;
			$frame->{hops} = [{ start => $start, end => $end, frequency => $channel, bandwidth => $bw }];
			$frame->{receivers} = [$dest];
			$frame->{sensitivity} = $sensis[$sf-7][bwconv($bw)];
		}
		$lr_radio->start_frame($frame);
	}
}

sub schedule_downlink{
	my ($sel_gw, $sel, $sel_sf, $sel_ch, $sel_seq, $sel_end, $rwindow, $new_index, $new_nbtrans) = @_;
	my $bnd;
	if ($fplan eq "US915"){
		if ($rwindow == 1){
			$sel_ch = $channels_d[$uplink_ch_index{$sel_ch}];
		}else{
			$sel_ch = $rx2ch;
		}
	} else {
		$bnd = "54"; # band: 54 for 869525000 RX2
		$bnd = $band{$sel_ch} if ($rwindow == 1);
	}
	my $extra_bytes = 0;
	if (($new_index != -1) || ($new_nbtrans != -1)){
		$extra_bytes = $linkadr_req;
		print "# LinkADRReq for $sel:" if ($debug == 1);
		print " TXPower=$Ptx_l[$new_index]dBm" if (($debug == 1) && ($new_index != -1));
		print " NbTrans=$new_nbtrans" if (($debug == 1) && ($new_nbtrans != -1));
		print "\n" if ($debug == 1);
	}
	my $cb = $bw125;
	$cb = $bw500 if ($fplan eq "US915");
	my $down_sf = ($lrfhss && $rwindow == 2) ? $rx2sf : $sel_sf;
	my $airt = airtime($down_sf, $cb, $overhead_d+$extra_bytes);
	my ($ack_sta, $ack_end) = ($sel_end+$rwindow, $sel_end+$rwindow+$airt);
	$total_down_time += $airt;
	print "# gw $sel_gw will transmit an ack (or commands) to $sel (RX$rwindow) (channel $sel_ch)\n" if ($debug == 1);
	$gresponses{$sel_gw} += 1;
	push (@{$gunavailability_d{$sel_gw}}, [$ack_sta, $ack_end]);
	if ($fplan ne "US915"){
		my $dc = $dutycycle;
		$dc = 9 if ($rwindow == 2);
		$gdc{$sel_gw}{$bnd} = $ack_end+$airt*$dc;
	}
	my $new_name = $sel_gw.$gresponses{$sel_gw}; # e.g. A1
	# place new transmission at the correct position
	my $i = 0;
	foreach my $el (@{$sorted_t{$sel_ch}}){
		my ($n, $sta, $end, $ch, $sf, $seq) = @$el;
		last if ($sta > $ack_sta);
		$i += 1;
	}
	###
	$bnd = $rx2ch if ($rwindow == 2);
	$bnd = $sel_ch if ($fplan eq "US915");
	$gtime{$sel_gw}{$bnd} += $airt;
	###
	$appacked{$sel} += 1 if ($nconfirmed{$sel} == 1);
	splice(@{$sorted_t{$sel_ch}}, $i, 0, [$new_name, $ack_sta, $ack_end, $sel_ch, $down_sf, $appacked{$sel}]);
	push (@{$gdest{$sel_gw}}, [$sel, $sel_end+$rwindow, $sel_sf, $rwindow, $sel_ch, $new_index, $new_nbtrans, $sel_seq]);
}

sub gs_policy{ # gateway selection policy
	my ($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $gw_rc, $win, $cmd_bytes) = @_;
	$cmd_bytes = 0 unless defined $cmd_bytes;
	my $sel_gw = undef;
	if ($fplan eq "US915"){
		$sel_ch = $channels_d[$uplink_ch_index{$sel_ch}] if ($win == 1);
	}
	my $bnd;
	if ($fplan ne "US915"){
		$bnd = $band{$sel_ch};
	}
	if ($win == 2){
		$bnd = "54" if ($fplan ne "US915");
		$sel_ch = $rx2ch;
		if (!$lrfhss && $sel_sf < $rx2sf){
			@$gw_rc = @{$nreachablegws{$sel}};
		}
		$sel_sf = $rx2sf;
	}
	my $cb = $bw125;
	$cb = $bw500 if ($fplan eq "US915");
	my ($ack_sta, $ack_end) = ($sel_end+$win, $sel_end+$win+airtime($sel_sf, $cb, $overhead_d+$cmd_bytes));
	my ($min_resp, $sel_p, $min_dc) = (1, -9999999999999, 9999999999999);
	my @avail = ();
	
	foreach my $g (@$gw_rc){
		my ($gw, $p) = @$g;
		my $is_avail = 1;
		if ($fplan ne "US915"){
			if ($gdc{$gw}{$bnd} > ($sel_end+$win)){
				next;
			}
		}
		my ($usta, $uend, $sf) = @{$gunavailability_u{$gw}{$sel_ch} || [-1, -1, 0]};
		if ( (($ack_sta >= $usta) && ($ack_sta <= $uend)) || (($ack_end <= $uend) && ($ack_end >= $usta)) ){
			$is_avail = 0;
			last;
		}
		foreach my $gu (@{$gunavailability_d{$gw}}){
			my ($sta, $end) = @$gu;
			if ( (($ack_sta >= $sta) && ($ack_sta <= $end)) || (($ack_end <= $end) && ($ack_end >= $sta)) ){
				$is_avail = 0;
				last;
			}
		}
		next if ($is_avail == 0);
		push (@avail, $g);
	}
	return (undef, undef) if (scalar @avail == 0);
	
	if ($policy == 4){ # URCB
		my $avgretr = (sum values %nogwavail)/(scalar keys %ncoords);
		if ( ($nogwavail{$sel} < $avgretr) && ((scalar @avail)/(scalar @$gw_rc) < 2/3) ){
			return (undef, undef);
		}
	}
	if ($policy == 5){ # FBS
		my $avgfair = 0;
		foreach my $n (sort {$a <=> $b} keys %ncoords){
			next if ($appsuccess{$n} == 0);
			$avgfair += $appacked{$n}/$appsuccess{$n};
		}
		$avgfair /= (scalar keys %ncoords);
		if ( ($appacked{$sel}/$appsuccess{$sel} >= $avgfair) && ((scalar @avail)/(scalar @$gw_rc) < 2/3) && ($avgfair != 0) ){
			return (undef, undef);
		}
	}
	foreach my $g (@avail){
		my ($gw, $p) = @$g;
		if ($policy == 1){ # FCFS
			my $resp = rand(2)/10;
			if ($resp < $min_resp){
				$min_resp = $resp;
				$sel_gw = $gw;
			}
		}elsif (($policy == 2) || ($policy == 4) || ($policy == 5)){ # RSSI
			if ($p > $sel_p){
				$sel_gw = $gw;
				$sel_p = $p;
			}
		}elsif ($policy == 3){ # least busy gw
			if ($gdc{$gw}{$bnd} < $min_dc){
				$min_dc = $gdc{$gw}{$bnd};
				$sel_gw = $gw;
			}
		}
	}
	return $sel_gw;
}

sub record_uplink_fcnt{
	my ($sel, $fcnt) = @_;
	my $hist = $nuplink_fcnt_history{$sel};

	# ChirpStack ignores re-transmissions / repeated NbTrans copies carrying
	# the same FCntUp as the latest ADR-history entry.
	if ((defined $hist) && (scalar @$hist > 0) && ($hist->[-1] == $fcnt)){
		return;
	}

	push(@{$nuplink_fcnt_history{$sel}}, $fcnt);
	shift(@{$nuplink_fcnt_history{$sel}}) while (scalar @{$nuplink_fcnt_history{$sel}} > $nbtrans_history_len);
}

# according to https://raw.githubusercontent.com/chirpstack/chirpstack/master/chirpstack/src/adr/default.rs
sub nbtrans_policy{
	my $sel = shift;
	my $hist = $nuplink_fcnt_history{$sel};

	# ChirpStack reports 0% loss until 20 received history entries exist.
	my $pkt_loss_rate = 0.0;

	if ((defined $hist) && (scalar @$hist >= $nbtrans_history_len)){
		my $lost_packets = 0;
		my $previous_fcnt = $hist->[0];

		for (my $i = 1; $i < scalar @$hist; $i += 1){
			my $fcnt = $hist->[$i];
			my $gap = $fcnt - $previous_fcnt - 1;
			$lost_packets += $gap if ($gap > 0);
			$previous_fcnt = $fcnt;
		}

		# Matches ChirpStack: lost_packets / uplink_history.len() * 100.
		$pkt_loss_rate = 100.0 * $lost_packets / scalar(@$hist);
	}

	my $current_nbtrans = $nnbtrans{$sel};
	$current_nbtrans = 1 if ($current_nbtrans < 1);
	$current_nbtrans = 3 if ($current_nbtrans > 3);
	my $idx = $current_nbtrans - 1;

	my @loss_lt_5  = (1, 1, 2);
	my @loss_lt_10 = (1, 2, 3);
	my @loss_lt_30 = (2, 3, 3);
	my @loss_ge_30 = (3, 3, 3);

	my $target_nbtrans;
	if ($pkt_loss_rate < 5.0){
		$target_nbtrans = $loss_lt_5[$idx];
	}elsif ($pkt_loss_rate < 10.0){
		$target_nbtrans = $loss_lt_10[$idx];
	}elsif ($pkt_loss_rate < 30.0){
		$target_nbtrans = $loss_lt_30[$idx];
	}else{
		$target_nbtrans = $loss_ge_30[$idx];
	}

	printf "# NbTrans policy for %s: packet loss %.3f%% over %d received unique uplinks -> target NbTrans=%d (current=%d)\n",
		$sel, $pkt_loss_rate, (defined $hist ? scalar(@$hist) : 0),
		$target_nbtrans, $nnbtrans{$sel} if ($debug == 1);

	return $target_nbtrans;
}

sub adr{ # Uplink PHY/rate stays fixed; ADR adjusts TX power and may request NbTrans
	my ($sel, $sel_sf) = @_;
	# Average temporal fading; the strongest historical sample overstates margin.
	my $m_snr = sum(@{$powers{$sel}}) / scalar(@{$powers{$sel}});
	my $sensitivity = $lrfhss ? $lr_sensitivity : $gw_sensis[$sel_sf-7][bwconv($bw125)];
	my $required_snr = $sensitivity - $noise;
	my $mgap = $m_snr - $required_snr - $margin;
	my $nstep = int($mgap/3);
	my $old_index = $nptx{$sel};
	my $new_index = $old_index;
	my $max_index = (scalar @Ptx_l) - 3; # 14 dBm EU868
	$max_index = (scalar @Ptx_l) - 1 if ($fplan eq "US915");

	while (($nstep > 0) && ($new_index > 0)){
		$new_index -= 1;
		$nstep -= 1;
	}
	while (($nstep < 0) && ($new_index < $max_index)){
		$new_index += 1;
		$nstep += 1;
	}

	my $new_ptx = ($new_index == $old_index) ? undef : $Ptx_l[$new_index];
	$new_index = -1 if (!defined $new_ptx);

	my $target_nbtrans = nbtrans_policy($sel);
	$target_nbtrans = 1 if ($target_nbtrans < 1);
	$target_nbtrans = 3 if ($target_nbtrans > 3);
	my $new_nbtrans = ($target_nbtrans == $nnbtrans{$sel}) ? -1 : $target_nbtrans;

	return ($new_ptx, $new_index, $new_nbtrans);
}

sub remove_scheduled_uplink{
	my ($node, $seq) = @_;
	return 0 if !defined $seq;
	my $removed = 0;
	foreach my $channel (keys %sorted_t){
		my @keep = ();
		foreach my $event (@{$sorted_t{$channel}}){
			my ($n, $sta, $end, $ch, $sf, $event_seq, $scheduled_energy, $prev_dc, $is_nbtrans_copy) = @$event;
			if (($n =~ /^[0-9]/) && ($n == $node) && ($event_seq == $seq)){
				$removed += 1;
				$total_trans -= 1;
				$nconsumption{$node} -= $scheduled_energy if defined $scheduled_energy;
				if (($fplan ne "US915") && defined $prev_dc){
					$ndc{$node}{$band{$channel}} = $prev_dc;
				}
				if (defined $is_nbtrans_copy && $is_nbtrans_copy == 1){
					$total_nbtrans_repetitions -= 1 if ($total_nbtrans_repetitions > 0);
				}
				print "# cancelling pending NbTrans copy of node $node FCntUp=$seq after valid downlink\n" if ($debug == 1);
				next;
			}
			push @keep, $event;
		}
		@{$sorted_t{$channel}} = @keep;
	}
	$nnbtrans_count{$node} = 1 if ($removed > 0);
	return $removed;
}

sub schedule_unconfirmed_after_downlink{
	my ($node, $sf, $uplink_end, $downlink_end) = @_;
	my $ch = $channels[rand @channels];
	$ch = $channels[rand @channels] while (($ch == $nlast_ch{$node}) && (scalar @channels > 1));
	my $new_start = $uplink_end + $nperiod{$node} + rand(1);
	$new_start = $downlink_end + rand(1) if ($new_start <= $downlink_end);

	if ($fplan ne "US915"){
		if ($new_start < $ndc{$node}{$band{$ch}}){
			$new_start = $ndc{$node}{$band{$ch}};
		}
	}
	return if ($new_start >= $sim_time);

	$nunique{$node} += 1;
	my $seq = $nunique{$node};
	$ndeliv_seq{$node}{$seq} = 1;
	$nnbtrans_count{$node} = 1;
	my $at = uplink_airtime($sf, $npkt{$node}); # LinkADRAns, if pending, is added when this event executes.
	my $new_end = $new_start + $at;
	my $i = 0;
	foreach my $el (@{$sorted_t{$ch}}){
		my ($n, $sta, $end, $ch_, $sf_, $seq_) = @$el;
		last if ($sta > $new_start);
		$i += 1;
	}
	my $scheduled_energy = $at * $Ptx_w[$nptx{$node}] + $at * $Pidle_w;
	my $prev_dc = -1;
	$prev_dc = $ndc{$node}{$band{$ch}} if ($fplan ne "US915");
	splice(@{$sorted_t{$ch}}, $i, 0, [$node, $new_start, $new_end, $ch, $sf, $seq, $scheduled_energy, $prev_dc, 0, $npkt{$node}, $nptx{$node}]);
	$total_trans += 1;
	$nconsumption{$node} += $scheduled_energy;
	$ndc{$node}{$band{$ch}} = $new_end + $dutycycle*$at if ($fplan ne "US915");
	print "# $node, next FCntUp=$seq scheduled after LinkADRReq at $new_start -> $new_end\n" if ($debug == 1);
}

sub node_col{ # handle node collisions
	my ($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $sel_seq) = @_;
	# check for collisions with other transmissions (time, SF, power) per gw
	my @gw_rc = ();
	foreach my $gw (@gw_ids){
		next if (exists $surpressed{$sel}{$gw}{$sel_seq});
		my $d = $dist_ng{$sel}{$gw};
		my $G = random_normal(1, 0, 1);
		my $prx = $Ptx_l[$nptx{$sel}] - $pl_ng{$sel}{$gw} - $G*$var;
		if ($prx < $gw_sensis[$sel_sf-7][bwconv($bw125)]){
			$surpressed{$sel}{$gw}{$sel_seq} = 1;
			print "# packet didn't reach gw $gw\n" if ($debug == 1);
			next;
		}
		# check if the gw is available for uplink
		my $is_available = 1;
		my ($usta, $uend, $sf) = @{$gunavailability_u{$gw}{$sel_ch} || [-1, -1, 0]};
		if ( (($sel_sta >= $usta) && ($sel_sta <= $uend)) || (($sel_end <= $uend) && ($sel_end >= $usta)) ){
			$is_available = 0 if ($sf == $sel_sf);
		}
		if ($is_available == 1){
			foreach my $gu (@{$gunavailability_d{$gw}}){
				my ($sta, $end) = @$gu;
				if ( (($sel_sta >= $sta) && ($sel_sta <= $end)) || (($sel_end <= $end) && ($sel_end >= $sta)) || (($sel_sta == $sta) && ($sel_end == $end))){
					$is_available = 0;
					last;
				}
			}
		}
		if ($is_available == 0){
			$surpressed{$sel}{$gw}{$sel_seq} = 1;
			print "# gw not available for uplink (channel $sel_ch, SF $sel_sf)\n" if ($debug == 1);
			next;
		}
		foreach my $tr (@{$sorted_t{$sel_ch}}){
			my ($n, $sta, $end, $ch, $sf, $seq) = @$tr;
			last if ($sta > $sel_end);
			if ($n =~ /^[0-9]/){ # node transmission
				next if (($n == $sel) || ($sta > $sel_end) || ($end < $sel_sta));
				my $overlap = 0;
				# time overlap
				if ( (($sel_sta >= $sta) && ($sel_sta <= $end)) || (($sel_end <= $end) && ($sel_end >= $sta)) || (($sel_sta == $sta) && ($sel_end == $end)) ){
					$overlap += 1;
				}
				# SF
				if ($sf == $sel_sf){
					$overlap += 2;
				}
				# power 
				$d = $dist_ng{$n}{$gw};
				my $prx_ = $Ptx_l[$nptx{$n}] - $pl_ng{$n}{$gw} - random_normal(1, 0, 1)*$var;
				if ($overlap == 3){
					if ((abs($prx - $prx_) <= $thresholds[$sel_sf-7][$sf-7]) ){ # both collide
						$surpressed{$sel}{$gw}{$sel_seq} = 1;
						$surpressed{$n}{$gw}{$seq} = 1;
						print "# $sel collided together with $n at gateway $gw\n" if ($debug == 1);
					}
					if (($prx_ - $prx) > $thresholds[$sel_sf-7][$sf-7]){ # n suppressed sel
						$surpressed{$sel}{$gw}{$sel_seq} = 1;
						print "# $sel surpressed by $n at gateway $gw\n" if ($debug == 1);
					}
					if (($prx - $prx_) > $thresholds[$sf-7][$sel_sf-7]){ # sel suppressed n
						$surpressed{$n}{$gw}{$seq} = 1;
						print "# $n surpressed by $sel at gateway $gw\n" if ($debug == 1);
					}
				}
				if (($overlap == 1) && ($full_collision == 1)){ # non-orthogonality
					if (($prx - $prx_) > $thresholds[$sel_sf-7][$sf-7]){
						if (($prx_ - $prx) <= $thresholds[$sf-7][$sel_sf-7]){
							$surpressed{$n}{$gw}{$seq} = 1;
							print "# $n surpressed by $sel\n" if ($debug == 1);
						}
					}else{
						if (($prx_ - $prx) > $thresholds[$sf-7][$sel_sf-7]){
							$surpressed{$sel}{$gw}{$sel_seq} = 1;
							print "# $sel surpressed by $n\n" if ($debug == 1);
						}else{
							$surpressed{$sel}{$gw}{$sel_seq} = 1;
							$surpressed{$n}{$gw}{$seq} = 1;
							print "# $sel collided together with $n\n" if ($debug == 1);
						}
					}
				}
			}else{ # n is a gw in this case
				my $nn = $n;
				$n =~ s/[0-9].*//; # keep only the letter(s)
				next if (($nn eq $gw) || ($sta > $sel_end) || ($end < $sel_sta) || ($ch != $sel_ch));
				# time overlap
				if ( (($sel_sta >= $sta) && ($sel_sta <= $end)) || (($sel_end <= $end) && ($sel_end >= $sta)) || (($sel_sta == $sta) && ($sel_end == $end)) ){
					my $already_there = 0;
					my $G_ = random_normal(1, 0, 1);
					foreach my $ng (@{$overlaps{$sel}}){
						my ($n_, $G_, $sf_) = @$ng;
						if ($n_ eq $n){
							$already_there = 1;
						}
					}
					if ($already_there == 0){
						push(@{$overlaps{$sel}}, [$n, $G_, $sf]); # put in here all overlapping transmissions
					}
					push(@{$overlaps{$nn}}, [$sel, $G, $sel_sf]); # check future possible collisions with those transmissions
				}
				foreach my $ng (@{$overlaps{$sel}}){
					my ($n, $G_, $sf_) = @$ng;
					my $overlap = 1;
					# SF
					if ($sf_ == $sel_sf){
						$overlap += 2;
					}
					# power 
					my $d_ = distance($gcoords{$gw}[0], $gcoords{$n}[0], $gcoords{$gw}[1], $gcoords{$n}[1]); # no precomputation here
					$d_ = 0.2 if ($d_ == 0);
					my $prx_ = $Ptx_gw - ($Lpld0 + 10*$gamma * log10($d_/$dref) + $G_*$var);
					if ($overlap == 3){
						if ((abs($prx - $prx_) <= $thresholds[$sel_sf-7][$sf_-7]) ){ # both collide
							$surpressed{$sel}{$gw}{$sel_seq} = 1;
							print "# $sel collided together with $n at gateway $gw\n" if ($debug == 1);
						}
						if (($prx_ - $prx) > $thresholds[$sel_sf-7][$sf_-7]){ # n suppressed sel
							$surpressed{$sel}{$gw}{$sel_seq} = 1;
							print "# $sel surpressed by $n at gateway $gw\n" if ($debug == 1);
						}
						if (($prx - $prx_) > $thresholds[$sf_-7][$sel_sf-7]){ # sel suppressed n
							print "# $n surpressed by $sel at gateway $gw\n" if ($debug == 1);
						}
					}
					if (($overlap == 1) && ($full_collision == 1)){ # non-orthogonality
						if (($prx - $prx_) > $thresholds[$sel_sf-7][$sf_-7]){
							if (($prx_ - $prx) <= $thresholds[$sf_-7][$sel_sf-7]){
								print "# $n surpressed by $sel\n" if ($debug == 1);
							}
						}else{
							if (($prx_ - $prx) > $thresholds[$sf_-7][$sel_sf-7]){
								$surpressed{$sel}{$gw}{$sel_seq} = 1;
								print "# $sel surpressed by $n\n" if ($debug == 1);
							}else{
								$surpressed{$sel}{$gw}{$sel_seq} = 1;
								print "# $sel collided together with $n\n" if ($debug == 1);
							}
						}
					}
				}
			}
		}
		if (!exists $surpressed{$sel}{$gw}{$sel_seq}){
			push (@gw_rc, [$gw, $prx]);
			# set the gw unavailable (exclude preamble's first 3 symbols) and lock to the specific transmission
			my $Tsym = (2**$sel_sf)/$bw125;
			my $Tpream = ($preamble-3 + 4.25)*$Tsym;
# 			push(@{$gunavailability{$gw}}, [$sel_sta+$Tpream, $sel_end, $sel_ch, $sel_sf, "u"]);
			$gunavailability_u{$gw}{$sel_ch} = [$sel_sta+$Tpream, $sel_end, $sel_sf];
		}
	}
	@{$overlaps{$sel}} = ();
	return (\@gw_rc);
}

sub min_sf{
	my $n = shift;
	my $G = 0; # assume that variance is 0
	my $Xs = $var*$G;
	my $sf = 13;
	my $bwi = bwconv($bw125);
	my $max_sf = 12;
	$max_sf = 10 if ($fplan eq "US915");
	foreach my $gw (@gw_ids){
		next if (exists $gdublicate{$gw});
		my $gf = 13;
		my $d0 = $dist_ng{$n}{$gw};
		for (my $f=7; $f<=$max_sf; $f+=1){
			my $S = $gw_sensis[$f-7][$bwi];
			my $Prx = $Ptx_l[$nptx{$n}] - $pl_ng{$n}{$gw} - $Xs;
			if (($Prx - $margin) > $S){
				$gf = $f;
				$f = 13;
				last;
			}
		}
		$sf = $gf if ($gf < $sf);
	}
	# check which gateways can be reached with rx2sf
	foreach my $gw (@gw_ids){
		my $d0 = $dist_ng{$n}{$gw};
		my $S = $gw_sensis[$rx2sf-7][$bwi];
		my $Prx = $Ptx_l[$nptx{$n}] - $pl_ng{$n}{$gw} - $Xs;
		if (($Prx - $margin) > $S){
			push(@{$nreachablegws{$n}}, [$gw, $Prx]);
		}
	}
	if ($sf == 13){
		print "node $n unreachable!\n";
		print "terrain too large?\n";
		exit;
	}
	if ($fixed_packet_size == 0){
		if ($packet_size_distr eq "uniform"){
			$npkt{$n} = int(rand($fpl[$sf-7]));
		}elsif ($packet_size_distr eq "normal"){
			$npkt{$n} = int(random_normal(1, $packet_size, 10));
		}
	}else{
		$npkt{$n} = $packet_size;
	}
	$npkt{$n} = $fpl[$sf-7] if ($npkt{$n} > $fpl[$sf-7]);
	$npkt{$n} += $overhead_u;
	print "# $n can reach a gw with SF$sf\n" if ($debug == 1);
	$sf_distr[$sf-7] += 1;
	return $sf;
}

# a modified version of LoRaSim (https://www.lancaster.ac.uk/scc/sites/lora/lorasim.html)
my %airtime_cache; # let's cache already used calculations
sub airtime{
	my ($sf, $bw, $payload) = @_;
	my $key = "$sf:$bw:$payload";
	return $airtime_cache{$key} if exists $airtime_cache{$key};
	
	my $DE = 0;
	$DE = 1 if (($bw == 125000) && (($sf == 11) || ($sf == 12)));
	my $Tsym = (2**$sf)/$bw;
	my $Tpream = ($preamble + 4.25)*$Tsym;
	my $payloadSymbNB = 8 + max( ceil((8.0*$payload-4.0*$sf+28+16*$CRC-20*$H) /	(4.0*($sf-2*$DE))) * ($cr+4), 0 );
	my $Tpayload = $payloadSymbNB * $Tsym;
	
	my $T = $Tpream + $Tpayload;
	$airtime_cache{$key} = $T;
	return $T;
}


sub bwconv{
	my $bw = shift;
	my $bwi = 0;
	if ($bw == 125000){
		$bwi = 1;
	}elsif ($bw == 250000){
		$bwi = 2;
	}elsif ($bw == 500000){
		$bwi = 3;
	}
	return $bwi;
}

sub read_data{
	open(FH, "<$terrain_file") or die "Error: could not open terrain file $terrain_file\n";
	my @nodes = ();
	my @gateways = ();
	while(<FH>){
		chomp;
		if (/^# stats: (.*)/){
			my $stats_line = $1;
			if ($stats_line =~ /terrain=([0-9]+\.[0-9]+)m\^2/){
				$terrain = $1;
			}
			$norm_x = sqrt($terrain);
			$norm_y = sqrt($terrain);
		} elsif (/^# node coords: (.*)/){
			my $sensor_coord = $1;
			my @coords = split(/\] /, $sensor_coord);
			@nodes = map { /([0-9]+) \[([0-9]+\.[0-9]+) ([0-9]+\.[0-9]+)/; [$1, $2, $3]; } @coords;
		} elsif (/^# gateway coords: (.*)/){
			my $gw_coord = $1;
			my @coords = split(/\] /, $gw_coord);
			@gateways = map { /([A-Z]+) \[([0-9]+\.[0-9]+) ([0-9]+\.[0-9]+)/; [$1, $2, $3]; } @coords;
		}
	}
	close(FH);
	
	my $conf_num = int($confirmed_perc * (scalar @nodes));
	foreach my $node (@nodes){
		my ($n, $x, $y) = @$node;
		$ncoords{$n} = [$x, $y];
		@{$overlaps{$n}} = ();
		$nptx{$n} = (scalar @Ptx_l) - 3; # start with the highest Ptx (14 dBm)
		$nptx{$n} = (scalar @Ptx_l) - 1 if ($fplan eq "US915"); # 20 dBm
		$nresponse{$n} = 0;
		$nnbtrans{$n} = $configured_nbtrans; # initial state; ChirpStack ADR adapts it in 1..3
		$nnbtrans_count{$n} = 1;
		$nretransmissions{$n} = 0;
		if ($conf_num > 0){
			$nconfirmed{$n} = 1;
			$conf_num -= 1;
			$ntotretr{$n} = 0;
		}else{
			$nconfirmed{$n} = 0;
		}
		$nacked{$n} = 0;
		$ndeliv{$n} = 0;
		$appacked{$n} = 0;
		$appsuccess{$n} = 0;
		$nogwavail{$n} = 0;
		if ($fixed_packet_rate == 0){
			my @per = random_exponential(scalar keys @nodes, 2*$period); # other distributions may be used
			foreach my $n (sort {$a <=> $b} keys %ncoords){
				$nperiod{$n} = pop(@per);
			}
		}else{
			$nperiod{$n} = $period;
		}
		foreach my $bnd (@bands){
			next if ($fplan eq "US915");
			$ndc{$n}{$bnd} = -1;
		}
		@{$powers{$n}} = ();
		@{$nuplink_fcnt_history{$n}} = ();
	}
	@gateways = sort { $a->[0] cmp $b->[0] } @gateways;
	my $last_gw = $gateways[-1][0];
	my @dublicates = ();
	if ($double_gws == 1){
		foreach my $gw (@gateways){
			my ($g, $x, $y) = @$gw;
			my $gdb = ++$last_gw;
			push (@dublicates, [$gdb, $x, $y]);
			$gdublicate{$gdb} = 1;
		}
	}
	@gateways = (@gateways, @dublicates);
	foreach my $gw (@gateways){
		my ($g, $x, $y) = @$gw;
		$gcoords{$g} = [$x, $y];
		@{$gunavailability_d{$g}} = ();
		foreach my $ch (@channels){
			next if ($fplan eq "US915");
			$gdc{$g}{$band{$ch}} = 0;
			$gunavailability_u{$g}{$ch} = [-1, -1, 0];
		}
		$gunavailability_u{$g}{$rx2ch} = [-1, -1, 0];
		foreach my $bnd (@bands){
			$gtime{$g}{$bnd} = 0;
		}
		if ($fplan eq "US915"){
			foreach my $ch (@channels_d){
				$gtime{$g}{$ch} = 0;
			}
		}
		$gtime{$g}{$rx2ch} = 0;
		$gdc{$g}{"54"} = 0 if ($fplan ne "US915");
		@{$overlaps{$g}} = ();
		$gresponses{$g} = 0;
	}
	for (my $i=7; $i<13; $i++){
		$sf_retrans{$i} = 0;
	}
	
	# precompute nodes-gws distances/path-losses
	foreach my $n (sort {$a <=> $b} keys %ncoords){
		my ($nx, $ny) = @{$ncoords{$n}};
		foreach my $g (keys %gcoords){
			my ($gx, $gy) = @{$gcoords{$g}};
			my $dx = $gx - $nx;
			my $dy = $gy - $ny;
			my $d  = max(0.2, sqrt($dx*$dx + $dy*$dy));
			$dist_ng{$n}{$g} = $d;
			$pl_ng{$n}{$g} = $Lpld0 + 10 * $gamma * log($d/$dref) / log(10);
		}
	}
	# let's use this one for gw iterations (instead of %gcoords)
	@gw_ids = sort { $a cmp $b } keys %gcoords;
}

sub distance {
	my ($x1, $x2, $y1, $y2) = @_;
	return sqrt( (($x1-$x2)*($x1-$x2))+(($y1-$y2)*($y1-$y2)) );
}

sub generate_picture{
	my $prr = shift;
	my ($display_x, $display_y) = (800, 800); # 800x800 pixel display pane
	my $im = new GD::SVG::Image($display_x, $display_y);
	my $blue = $im->colorAllocate(0,0,255);
	my $black = $im->colorAllocate(0,0,0);
	my $red = $im->colorAllocate(255,0,0);
	
	my $max_ndeliv = max(1, values %ndeliv);
	foreach my $n (sort {$a <=> $b} keys %ncoords){
		my ($x, $y) = @{$ncoords{$n}};
		($x, $y) = (int(($x * $display_x)/$norm_x), int(($y * $display_y)/$norm_y));
		my $color = $im->colorAllocate(255*$nconsumption{$n}/$max_cons,0,0);
		$color = $im->colorAllocate(255-127*$ndeliv{$n}/$max_ndeliv,255*$ndeliv{$n}/$max_ndeliv,255-255*$ndeliv{$n}/$max_ndeliv) if ($prr == 1);
		$im->filledArc($x,$y,10,10,0,360,$color);
	}
	
	foreach my $g (@gw_ids){
		next if (exists $gdublicate{$g});
		my ($x, $y) = @{$gcoords{$g}};
		($x, $y) = (int(($x * $display_x)/$norm_x), int(($y * $display_y)/$norm_y));
		$im->rectangle($x-5, $y-5, $x+5, $y+5, $red);
		$im->string(gdGiantFont,$x-2,$y-20,$g,$blue);
	}
	my $output_file = $terrain_file."-img.svg";
	open(FILEOUT, ">$output_file") or die "could not open file $output_file for writing!";
	binmode FILEOUT;
	print FILEOUT $im->svg;
	close FILEOUT;
}
