#!/usr/bin/perl -w

###################################################################################
#          Event-based simulator for (un)confirmed LoRaWAN transmissions          #
#                                  v2025.11.30                                    #
#                                                                                 #
# Features:                                                                       #
# -- EU868 or US915 spectrum
# -- Multiple half-duplex gateways                                                #
# -- 1% radio duty cycle per band for the nodes                                   #
# -- 1 or 10% radio duty cycle for the gateways                                   #
# -- Acks with two receive windows (RX1, RX2)                                     #
# -- Non-orthogonal SF transmissions                                              #
# -- Periodic or non-periodic (exponential) transmission rate                     #
# -- Percentage of nodes required confirmed transmissions                         #
# -- Capture effect                                                               #
# -- Path-loss signal attenuation model (uplink+downlink)                         #
# -- Multiple channels                                                            #
# -- Collision handling for both uplink and downlink transmissions                #
# -- Energy consumption calculation (uplink+downlink)                             #
# -- ADR support                                                                  #
# -- Network server policies (downlink packet & gw selection)                     #
#                                                                                 #
# author: Dr. Dimitrios Zorbas                                                    #
# email: dimzorbas@ieee.org                                                       #
# distributed under GNUv2 General Public Licence                                  #
###################################################################################

use strict;
use POSIX;
use List::Util qw(min max sum);
use Time::HiRes qw(time);
use Math::Random qw(random_uniform random_exponential random_normal);
use GD::SVG;
use Statistics::Basic qw(:all);

die "usage: $0 <packets_per_hour> <simulation_time_(hours)> <terrain_file!>\n" unless (scalar @ARGV == 3);

die "Packet rate must be higher than or equal to 1pkt per hour\n" if ($ARGV[0] < 1);
die "Simulation time must be longer than or equal to 1h\n" if ($ARGV[1] < 1);

# node attributes
my %ncoords = (); # node coordinates
my %nconsumption = (); # consumption
my %nretransmissions = (); # retransmissions per node (per packet)
my %surpressed = ();
my %nreachablegws = (); # reachable gws
my %nptx = (); # transmit power index
my %nresponse = (); # 0/1 (1 = ADR response will be sent)
my %nconfirmed = (); # confirmed transmissions or not
my %nunique = (); # unique transmissions per node (equivalent to FCntUp)
my %nacked = (); # unique acked packets (for confirmed transmissions)
my %ndeliv = (); # unique delivered packets (for non-confirmed transmissions)
my %nperiod = (); 
my %ndc = (); # indicates when a node can transmit again per band (according to dc)
my %npkt = (); # packet size per node
my %ntotretr = (); # number of retransmissions per node (total)
my %nlast_ch = (); # last transmission time
my %ndeliv_seq = (); # 1 = unique packet seq exists (for confirmed uplinks)

# gw attributes
my %gcoords = (); # gw coordinates
my @gw_ids = (); # just for iterations
my %gunavailability_d = (); # unavailable gw time due to downlinks
my %gunavailability_u = (); # unavailable gw time due to uplinks
my %gdc = (); # gw duty cycle (1% uplink channel is used for RX1, 10% downlink channel is used for RX2)
my %gresponses = (); # acks carried out per gw
my %gdest = (); # contains downlink information [node, sf, RX1/2, channel, power index]
my %gdublicate = (); # exists if it's a double gw
my %gtime = (); # gateway downlink time per band or channel

# LoRa PHY and LoRaWAN parameters
my @sensis = ([7,-124,-122,-116], [8,-127,-125,-119], [9,-130,-128,-122], [10,-133,-130,-125], [11,-135,-132,-128], [12,-137,-135,-129]); # sensitivities per SF/BW
my @thresholds = ([1,-8,-9,-9,-9,-9], [-11,1,-11,-12,-13,-13], [-15,-13,1,-13,-14,-15], [-19,-18,-17,1,-17,-18], [-22,-22,-21,-20,1,-20], [-25,-25,-25,-24,-23,1]); # capture effect power thresholds per SF[SF] for non-orthogonal transmissions
my @snrs = (-7.5, -10, -12.5, -15, -17.5, -20);
my $margin = 5;
my $var = 3.57; # variance
my ($dref, $Lpld0, $gamma) = (40, 110, 2.08); # attenuation model parameters
my $noise = -90; # noise level for snr calculation
my $bw125 = 125000; # channel bandwidth
my $cr = 1; # Coding Rate
my $volt = 3.3; # avg voltage
my $Ptx_gw = 25; # gateway tx power (dBm)
my @Ptx_l = (2, 5, 8, 11, 14, 17, 20); # dBm
my @Ptx_w = (12*$volt, 20*$volt, 32*$volt, 51*$volt, 76*$volt, 90*$volt, 105*$volt); # Ptx cons. for 2, 5, 8, 11, 14, 17, and 20dBm (mA * V = mW)
my $Prx_w = 46 * $volt;
my $Pidle_w = 30 * $volt; # this is actually the consumption of the microcontroller in idle mode
my $fplan = "EU868"; # EU868 (default) or US915
# ------ EU868 ------- #
my @channels = (868100000, 868300000, 868500000, 867100000, 867300000, 867500000, 867700000, 867900000); # TTN channels
my @bands = ("48", "47");
my %band = (868100000=>"48", 868300000=>"48", 868500000=>"48", 867100000=>"47", 867300000=>"47", 867500000=>"47", 867700000=>"47", 867900000=>"47"); # band name per channel (all of them with 1% duty cycle)
my $rx2sf = 9; # SF used for RX2 (LoRaWAN default = SF12, TTN uses SF9)
my $rx2ch = 869525000; # channel used for RX2 (LoRaWAN default = 869.525MHz, TTN uses the same)
my $dutycycle = 99; # airtime multiplier for low duty cycle uplink transmissions (99 = 1% duty cycle, 9 = 10%)
# ------ US915 ------- #
my %uplink_ch_index = ();
if ($fplan eq "US915"){
	@channels = (903900000, 904100000, 904300000, 904500000, 904700000, 904900000, 905100000, 905300000);
	%uplink_ch_index = (903900000=>0, 904100000=>1, 904300000=>2, 904500000=>3, 904700000=>4, 904900000=>5, 905100000=>6, 905300000=>7); # key=uplink channel, value=index of uplink channel in @channels
	$rx2sf = 12; # SF used for RX2 (500kHz)
	$rx2ch = 923300000; # channel used for RX2
}
my $bw500 = 500000;
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
my $adr = 4; # Fopts option for the ADR (4 Bytes)
my $txdc = 1; # Fopts option for the TX duty cycle (1 Byte)
my $fport_u = 1; # 1B for FPort for uplink
my $fport_d = 0; # 0B for FPort for downlink (commands are included in Fopts, acks have no payload)
my $overhead_u = $mhdr+$mic+$fhdr+$fport_u+$hcrc; # LoRa+LoRaWAN uplink overhead
my $overhead_d = $mhdr+$mic+$fhdr+$fport_d+$hcrc; # LoRa+LoRaWAN downlink overhead
my %overlaps = (); # handles special packet overlaps 

# simulation parameters
my $confirmed_perc = 0; # percentage of nodes that require confirmed transmissions (1=all)
my $full_collision = 1; # take into account non-orthogonal SF transmissions or not
my $max_retr = 1; # max number of retransmissions per packet (default value = 1)
my $period = 3600/$ARGV[0]; # time period between transmissions
my $sim_time = $ARGV[1]*3600; # given simulation time
my $debug = 0; # enable debug mode
my $sim_end = 0;
my ($terrain, $norm_x, $norm_y) = (0, 0, 0); # terrain side, normalised terrain side
my $start_time = time; # just for statistics
my $dropped = 0; # number of dropped packets (for confirmed traffic)
my $dropped_unc = 0; # number of dropped packets (for unconfirmed traffic)
my $total_trans = 0; # number of transm. packets
my $total_retrans = 0; # number of re-transm packets
my $no_rx1 = 0; # no gw was available in RX1
my $no_rx2 = 0; # no gw was available in RX1 or RX2
my $picture = 1; # generate an energy consumption or a PRR map (see line after stats)
my $fixed_packet_rate = 1; # send packets periodically with a fixed rate (=1) or at random (=0)
my $total_down_time = 0; # total downlink time
my $avg_sf = 0;
my @sf_distr = (0, 0, 0, 0, 0, 0);
my $fixed_packet_size = 1; # all nodes have the same packet size defined in @fpl (=1) or a randomly selected (=0)
my $packet_size = 16; # default packet size if fixed_packet_size=1 or avg packet size if fixed_packet_size=0 (Bytes)
my $packet_size_distr = "normal"; # uniform / normal (applicable if fixed_packet_size=0)
my $avg_pkt = 0; # actual average packet size
my %sorted_t = (); # keys = channels, values = list of nodes
my @recents = (); # used in auto_simtime
my $auto_simtime = 0; # 1 = the simulation will automatically stop (useful when sim_time>>10000)
my %sf_retrans = (); # number of retransmissions per SF
my $adr_on = 1; # ADR is used or not (=0)
my $double_gws = 0; # enable 8x2 channel gateways

# application server
my $policy = 1; # gateway selection policy for downlink traffic
die "# The least busy policy cannot be used with US915 frequencies" if (($policy == 3) && ($fplan eq "US915"));
my %appacked = (); # counts the number of acked packets per node
my %appsuccess = (); # counts the number of packets that received from at least one gw per node
my %nogwavail = (); # counts how many time no gw was available (keys = nodes)
my %powers = (); # contains last 10 received powers per node

# precomputations
my %pl_ng;    # path-loss (without shadowing) per node-gw pair
my %dist_ng;  # node-gw distance 

read_data(); # read terrain file

# first transmission
my @init_trans = ();
foreach my $n (keys %ncoords){
	my $start = random_uniform(1, 0, $period);
	my $sf = min_sf($n);
	$avg_sf += $sf;
	$avg_pkt += $npkt{$n};
	my $airt = airtime($sf, $bw125, $npkt{$n});
	my $stop = $start + $airt;
	print "# $n will transmit from $start to $stop (SF $sf)\n" if ($debug == 1);
	$nunique{$n} = 1;
	$ndeliv_seq{$n}{$nunique{$n}} = 1;
	my $ch = $channels[rand @channels];
	push (@init_trans, [$n, $start, $stop, $ch, $sf, $nunique{$n}]);
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
	foreach my $ch (keys %sorted_t){
		delete $sorted_t{$ch} if @{$sorted_t{$ch}} == 0;
	}
	
	# select the channel with earliest transmission among all first transmissions
# 	my $min_ch = (sort {$sorted_t{$a}[0][1] <=> $sorted_t{$b}[0][1]} keys %sorted_t)[0];
	my $min_ch;
	my $min_t;
	while (my ($ch, $list) = each %sorted_t){
		my $t = $list->[0][1];  # start time
		if (!defined $min_ch || $t < $min_t){
			$min_ch = $ch;
			$min_t  = $t;
		}
	}
	last if !defined $min_ch;

	my ($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $sel_seq) = @{shift(@{$sorted_t{$min_ch}})};
	if (exists $ncoords{$sel}){ # just a progress trick
		if ($sel == 1){
			$| = 1;
			printf STDERR "%.2f%%\r", 100*$sel_end/$sim_time;
		}
	}
	last if ($sel_sta > $sim_time);
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
		
		my $gw_rc = node_col($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $sel_seq); # check collisions and return a list of gws that received the uplink pkt
		my $rwindow = 0;
		my $failed = 0;
		$nlast_ch{$sel} = $sel_ch;
		# keep the last 20 max received powers
		my $max_snr = -999;
		foreach my $g (@$gw_rc){
			my $snr = @$g[1] - $noise;
			$max_snr = $snr if ($snr > $max_snr);
		}
		push (@{$powers{$sel}}, $max_snr) if ($max_snr > -999);
		shift @{$powers{$sel}} if (scalar @{$powers{$sel}} > 10);
		if ((scalar @$gw_rc > 0) && ($nconfirmed{$sel} == 1)){ # if at least one gateway received the pkt -> successful transmission
			my ($new_ptx, $new_index) = (undef, -1);
			if ((scalar @{$powers{$sel}} == 10) && ($adr_on == 1)){
				($new_ptx, $new_index) = adr($sel, $sel_sf);
			}
			if (exists $ndeliv_seq{$sel}{$sel_seq}){
				$ndeliv{$sel} += 1;
				delete $ndeliv_seq{$sel}{$sel_seq};
			}
			$appsuccess{$sel} += 1;
			printf "# $sel 's transmission received by %d gateway(s) (channel $sel_ch)\n", scalar @$gw_rc if ($debug == 1);
			# now we have to find which gateway (if any) can transmit an ack in RX1 or RX2
			# check RX1
			my $sel_gw = gs_policy($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $gw_rc, 1);
			if (defined $sel_gw){
				schedule_downlink($sel_gw, $sel, $sel_sf, $sel_ch, $sel_seq, $sel_end, 1, $new_index);
			}else{
				# check RX2
				$no_rx1 += 1;
				$sel_gw = gs_policy($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $gw_rc, 2);
				if (defined $sel_gw){
					schedule_downlink($sel_gw, $sel, $sel_sf, $rx2ch, $sel_seq, $sel_end, 2, $new_index);
				}else{
					$no_rx2 += 1;
					print "# no gateway is available\n" if ($debug == 1);
					$nogwavail{$sel} += 1;
					$failed = 1;
				}
			}
		}elsif ((scalar @$gw_rc > 0) && ($nconfirmed{$sel} == 0)){ # successful transmission but no ack is required
			$ndeliv{$sel} += 1;
			$appsuccess{$sel} += 1;
			printf "# $sel 's transmission received by %d gateway(s) (channel $sel_ch)\n", scalar @$gw_rc if ($debug == 1);
			### ADR for unconfirmed transmissions
			if ((scalar @{$powers{$sel}} == 10) && ($adr_on == 1)){
				my ($new_ptx, $new_index) = adr($sel, $sel_sf);
				if (defined $new_ptx){
					my $sel_gw = gs_policy($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $gw_rc, 1);
					if (defined $sel_gw){
						schedule_downlink($sel_gw, $sel, $sel_sf, $sel_ch, $sel_seq, $sel_end, 1, $new_index);
					}else{
						$sel_gw = gs_policy($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $gw_rc, 2);
						if (defined $sel_gw){
							schedule_downlink($sel_gw, $sel, $sel_sf, $sel_ch, $sel_seq, $sel_end, 2, $new_index);
						}else{
							print "# no downlink commands could be sent to $sel\n" if ($debug == 1);
						}
					}
				}
			}
			###
			$nconsumption{$sel} += (2-($preamble+4.25)*(2**$sel_sf)/$bw125)*$Pidle_w + ($preamble+4.25)*(2**$sel_sf)/$bw125 * ($Prx_w + $Pidle_w);
			$nconsumption{$sel} += ($preamble+4.25)*(2**$rx2sf)/$bw125 * ($Prx_w + $Pidle_w);
			
			my $new_ch = $channels[rand @channels];
			$new_ch = $channels[rand @channels] while ($new_ch == $sel_ch);
			$sel_ch = $new_ch;
			my $at = airtime($sel_sf, $bw125, $npkt{$sel});
			$sel_sta = $sel_end + $nperiod{$sel} + rand(1);
			if ($fplan ne "US915"){
				if ($sel_sta < $ndc{$sel}{$band{$sel_ch}}){
					print "# warning! transmission will be postponed due to duty cycle restrictions!\n" if ($debug == 1);
					$sel_sta = $ndc{$sel}{$band{$sel_ch}};
				}
			}
			$sel_end = $sel_sta + $at;
			# place the new transmission at the correct position
			my $i = 0;
			foreach my $el (@{$sorted_t{$sel_ch}}){
				my ($n, $sta, $end, $ch_, $sf_, $seq) = @$el;
				last if ($sta > $sel_sta);
				$i += 1;
			}
			$nunique{$sel} += 1 if ($sel_sta < $sim_time); # do not count transmissions that exceed the simulation time;
			splice(@{$sorted_t{$sel_ch}}, $i, 0, [$sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $nunique{$sel}]);
			$total_trans += 1 if ($sel_sta < $sim_time);
			print "# $sel, new transmission at $sel_sta -> $sel_end\n" if ($debug == 1);
			$nconsumption{$sel} += $at * $Ptx_w[$nptx{$sel}] + $at * $Pidle_w;
			$ndc{$sel}{$band{$sel_ch}} = $sel_end + $dutycycle*$at if ($fplan ne "US915");
		}else{ # non-successful transmission
			$failed = 1;
		}
		if ($failed == 1){
			my $at = 0;
			my $new_trans = 0;
			my $new_ch = $channels[rand @channels];
			$new_ch = $channels[rand @channels] while ($new_ch == $sel_ch);
			$sel_ch = $new_ch;
			if ($nconfirmed{$sel} == 1){
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
				# the node stays on only for the duration of the preamble for both receive windows + in idle mode between RX windows
				$nconsumption{$sel} += (2-($preamble+4.25)*(2**$sel_sf)/$bw125)*$Pidle_w + ($preamble+4.25)*(2**$sel_sf)/$bw125 * ($Prx_w + $Pidle_w);
				$nconsumption{$sel} += ($preamble+4.25)*(2**$rx2sf)/$bw125 * ($Prx_w + $Pidle_w);
				# plan the next transmission as soon as the duty cycle permits that
				$at = airtime($sel_sf, $bw125, $npkt{$sel});
				if ($new_trans == 0){
					$sel_sta = $sel_end + 2 + 1 + rand(2);
				}else{
					$sel_sta = $sel_end + 2 + $nperiod{$sel} + rand(1);
				}
			}else{
				$dropped_unc += 1;
				$new_trans = 1;
				print "# $sel 's packet lost!\n" if ($debug == 1);
				$at = airtime($sel_sf, $bw125, $npkt{$sel});
				$sel_sta = $sel_end + $nperiod{$sel} + rand(1);
			}
			if ($fplan ne "US915"){
				if ($sel_sta < $ndc{$sel}{$band{$sel_ch}}){
					print "# warning! transmission will be postponed due to duty cycle restrictions!\n" if ($debug == 1);
					$sel_sta = $ndc{$sel}{$band{$sel_ch}};
				}
			}
			$sel_end = $sel_sta+$at;
			# place the new transmission at the correct position
			my $i = 0;
			foreach my $el (@{$sorted_t{$sel_ch}}){
				my ($n, $sta, $end, $ch_, $sf_, $seq) = @$el;
				last if ($sta > $sel_sta);
				$i += 1;
			}
			if (($new_trans == 1) && ($sel_sta < $sim_time)){ # do not count transmissions that exceed the simulation time
				$nunique{$sel} += 1;
				if ($nconfirmed{$sel} == 1){
					$total_retrans += 1;
					$ndeliv_seq{$sel}{$nunique{$sel}} = 1;
				}
			}
			$total_trans += 1 if ($sel_sta < $sim_time);
			splice(@{$sorted_t{$sel_ch}}, $i, 0, [$sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $nunique{$sel}]);
			print "# $sel, new transmission at $sel_sta -> $sel_end\n" if ($debug == 1);
			$nconsumption{$sel} += $at * $Ptx_w[$nptx{$sel}] + $at * $Pidle_w;
			$ndc{$sel}{$band{$sel_ch}} = $sel_end + $dutycycle*$at if ($fplan ne "US915");
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
		my ($dest, $st, $sf, $rwindow, $ch, $pow); # we also need dest, rwindow, and pow (the others should be the same)
		foreach my $tup (@{$gdest{$sel}}){
			my ($dest_, $st_, $sf_, $rwindow_, $ch_, $p_) = @$tup;
			if (($st_ == $sel_sta) && ($ch_ == $sel_ch)){
				($dest, $st, $sf, $rwindow, $ch, $pow) = ($dest_, $st_, $sf_, $rwindow_, $ch_, $p_);
				last;
			}
			$index += 1;
		}
		splice @{$gdest{$sel}}, $index, 1;
		# check if the transmission can reach the node
		my $G = random_normal(1, 0, 1);
		my $d = $dist_ng{$dest}{$sel};
		my $prx = $pl_ng{$dest}{$sel} + $G*$var;
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
				$d = $dist_ng{$dest}{$n};
				$d_ = 0.2 if ($d_ == 0);
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
			if ($rwindow == 2){ # also count the RX1 window
				$nconsumption{$dest} += (2-($preamble+4.25)*(2**$sf)/$cb)*$Pidle_w + ($preamble+4.25)*(2**$sf)/$cb * ($Prx_w + $Pidle_w);
			}
			my $extra_bytes = 0; # if an ADR request is included in the downlink packet
			if ($pow != -1){
				$nptx{$dest} = $pow;
				$extra_bytes = $adr;
				$nresponse{$dest} = 1;
				print "# transmit power of $dest is set to $Ptx_l[$pow]dBm\n" if ($debug == 1);
			}
			$nconsumption{$dest} += airtime($sel_sf, $cb, $overhead_d+$extra_bytes) * ($Prx_w + $Pidle_w);
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
			$nconsumption{$dest} += (2-($preamble+4.25)*(2**$sf)/$cb)*$Pidle_w + ($preamble+4.25)*(2**$sf)/$cb * ($Prx_w + $Pidle_w);
			$nconsumption{$dest} += ($preamble+4.25)*(2**$rx2sf)/$cb * ($Prx_w + $Pidle_w);
		}
		@{$overlaps{$sel}} = ();
		if ($nconfirmed{$dest} == 1){
			# plan next transmission
			do{
				$ch = $channels[rand @channels]; 
			} while ($ch == $nlast_ch{$dest});
			my $extra_bytes = 0;
			if ($nresponse{$dest} == 1){
				$extra_bytes = $adr;
				$nresponse{$dest} = 0;
			}
			my $at = airtime($sf, $bw125, $npkt{$dest}+$extra_bytes);
			my $new_start = $sel_sta - $rwindow + $nperiod{$dest} + rand(1);
			$new_start = $sel_sta - $rwindow + 2 + 1 + rand(2) if ($failed == 1 && $new_trans == 0);
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
			splice(@{$sorted_t{$ch}}, $i, 0, [$dest, $new_start, $new_end, $ch, $sf, $nunique{$dest}]);
			$total_trans += 1 if ($new_start < $sim_time); # do not count transmissions that exceed the simulation time
			$total_retrans += 1 if (($failed == 1) && ($new_start < $sim_time)); 
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
print "Total number of re-transmissions = $total_retrans\n" if ($confirmed_perc > 0);
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
foreach my $n (keys %ncoords){
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
if ($confirmed_perc > 0){
	@fairs = ();
	my $avgretr = 0;
	foreach my $n (keys %ncoords){
		next if ($nconfirmed{$n} == 0);
		$nacked{$n} = 1 if ($nacked{$n} == 0);
		push(@fairs, $nacked{$n}/$nunique{$n});
		$avgretr += $ntotretr{$n}/$nunique{$n};
	}
	printf "Downlink fairness = %.3f\n", stddev(\@fairs) if (scalar @fairs > 0);
	printf "Avg number of retransmissions = %.3f\n", $avgretr/(scalar keys %ntotretr);
	printf "Stdev of retransmissions = %.3f\n", (stddev values %ntotretr);
	print "-----\n";
}
for (my $sf=7; $sf<=12; $sf+=1){
	printf "# of nodes with SF%d: %d, Avg retransmissions: %.2f\n", $sf, $sf_distr[$sf-7], $sf_retrans{$sf}/$sf_distr[$sf-7] if ($sf_distr[$sf-7] > 0);
}
printf "Avg SF = %.3f\n", $avg_sf/(scalar keys %ncoords);
# printf "Avg packet size = %.3f bytes\n", $avg_pkt/(scalar keys %ncoords); # includes overhead
printf "Script execution time = %.4f secs\n", $finish_time - $start_time;
generate_picture(1) if ($picture == 1); # 0=energy consumption map, 1=PRR map


sub schedule_downlink{
	my ($sel_gw, $sel, $sel_sf, $sel_ch, $sel_seq, $sel_end, $rwindow, $new_index) = @_;
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
	if ($new_index != -1){
		print "# it will be suggested that $sel changes tx power to $Ptx_l[$new_index]dBm\n" if ($debug == 1);
		$extra_bytes = $adr;
	}
	my $cb = $bw125;
	$cb = $bw500 if ($fplan eq "US915");
	my $airt = airtime($sel_sf, $cb, $overhead_d+$extra_bytes);
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
	splice(@{$sorted_t{$sel_ch}}, $i, 0, [$new_name, $ack_sta, $ack_end, $sel_ch, $sel_sf, $appacked{$sel}]);
	push (@{$gdest{$sel_gw}}, [$sel, $sel_end+$rwindow, $sel_sf, $rwindow, $sel_ch, $new_index]);
}

sub gs_policy{ # gateway selection policy
	my ($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $gw_rc, $win) = @_;
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
		if ($sel_sf < $rx2sf){
			@$gw_rc = @{$nreachablegws{$sel}};
		}
		$sel_sf = $rx2sf;
	}
	my $cb = $bw125;
	$cb = $bw500 if ($fplan eq "US915");
	my ($ack_sta, $ack_end) = ($sel_end+$win, $sel_end+$win+airtime($sel_sf, $cb, $overhead_d));
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
		my ($usta, $uend, $sf) = @{$gunavailability_u{$gw}{$sel_ch}};
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
		foreach my $n (keys %ncoords){
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

sub adr{ # ADR: the SF is already adjusted in min_sf; here only the transmit power is adjusted
	my ($sel, $sel_sf) = @_;
	my $m_snr = (max @{$powers{$sel}});
	my $mgap = $m_snr - $snrs[$sel_sf-7] - $margin;
	my $nstep = int($mgap/3);
	my $new_ptx = $Ptx_l[$nptx{$sel}];
	my $new_index = $nptx{$sel};
	my $max_ptx = $Ptx_l[-3];
	$max_ptx = $Ptx_l[-1] if ($fplan eq "US915");
	while ($nstep != 0){
		if ($nstep > 0){
			$new_ptx -= 3;
			$new_index -= 1;
			$nstep -= 1;
			if ($new_ptx <= $Ptx_l[0]){
				last;
			}
		}else{
			if ($new_ptx < $max_ptx){
				$new_ptx += 3;
				$nstep += 1;
				$new_index += 1
			}else{
				last;
			}
		}
	}
	$new_ptx = undef if ($new_index == $nptx{$sel});
	return ($new_ptx, $new_index);
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
		if ($prx < $sensis[$sel_sf-7][bwconv($bw125)]){
			$surpressed{$sel}{$gw}{$sel_seq} = 1;
			print "# packet didn't reach gw $gw\n" if ($debug == 1);
			next;
		}
		# check if the gw is available for uplink
		my $is_available = 1;
		my ($usta, $uend, $sf) = @{$gunavailability_u{$gw}{$sel_ch}};
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
			my $S = $sensis[$f-7][$bwi];
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
		my $S = $sensis[$rx2sf-7][$bwi];
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
	$npkt{$n} += (16 - $npkt{$n}%16) if ($npkt{$n}%16 != 0); # make the packet size multiple of 16
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
	my $terrain_file = $ARGV[-1];
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
			foreach my $n (keys %ncoords){
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
	foreach my $n (keys %ncoords){
		my ($nx, $ny) = @{$ncoords{$n}};
		foreach my $g (keys %gcoords){
			my ($gx, $gy) = @{$gcoords{$g}};
			my $dx = $gx - $nx;
			my $dy = $gy - $ny;
			my $d  = sqrt($dx*$dx + $dy*$dy);
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
	
	my $max_ndeliv = (max values %ndeliv);
	foreach my $n (keys %ncoords){
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
	my $output_file = $ARGV[-1]."-img.svg";
	open(FILEOUT, ">$output_file") or die "could not open file $output_file for writing!";
	binmode FILEOUT;
	print FILEOUT $im->svg;
	close FILEOUT;
}
