#!/usr/bin/perl -w

###################################################################################
#           Event-based simulator for confirmed LoRaWAN transmissions             #
#                                 v2021.9.19                                      #
#                                                                                 #
# Features:                                                                       #
# -- Multiple half-duplex gateways                                                #
# -- 1% radio duty cycle for the nodes                                            #
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
# -- Network server policies                                                      #
#                                                                                 #
# author: Dr. Dimitrios Zorbas                                                    #
# email: dimzorbas@ieee.org                                                       #
# distributed under GNUv2 General Public Licence                                  #
###################################################################################

use strict;
use POSIX;
use List::Util qw(min max sum);
use Time::HiRes qw(time);
use Math::Random qw(random_uniform random_exponential);
use Term::ProgressBar 2.00;
use GD::SVG;
use Statistics::Basic qw(:all);

die "usage: ./LoRaWAN.pl <packets_per_hour> <simulation_time(secs)> <gateway_selection_policy(1-3)> <terrain_file!>\n" unless (scalar @ARGV == 4);

# node attributes
my %ncoords = (); # node coordinates
my %nconsumption = (); # consumption
my %nretransmisssions = (); # retransmisssions per node
my %surpressed = ();
my %nreachablegws = (); # reachable gws
my %nptx = (); # transmit power index
my %nresponse = (); # 0/1 (1 = ADR response will be sent)
my %nconfirmed = (); # confirmed transmissions or not
my %nunique = (); # unique transmissions per node (equivalent to FCntUp)
my %nacked = (); # unique acked packets (for confirmed transmissions) or just delivered (for non-confirmed transmissions)
my %nperiod = (); 
my %won = ();
my %ndc = ();

# gw attributes
my %gcoords = (); # gw coordinates
my %gunavailability = (); # unavailable gw time due to downlink or locked to another transmission
my %gdc = (); # gw duty cycle (1% uplink channel is used for RX1, 10% downlink channel is used for RX2)
my %gresponses = (); # acks carried out per gw
my %gdest = (); # contains downlink information [node, sf, RX1/2, channel]

# LoRa PHY and LoRaWAN parameters
my @sensis = ([7,-124,-122,-116], [8,-127,-125,-119], [9,-130,-128,-122], [10,-133,-130,-125], [11,-135,-132,-128], [12,-137,-135,-129]); # sensitivities per SF/BW
my @thresholds = ([6,-16,-18,-19,-19,-20], [-24,6,-20,-22,-22,-22], [-27,-27,6,-23,-25,-25], [-30,-30,-30,6,-26,-28], [-33,-33,-33,-33,6,-29], [-36,-36,-36,-36,-36,6]); # capture effect power thresholds per SF[SF] for non-orthogonal transmissions
my $var = 3.57; # variance
my ($dref, $Lpld0, $gamma) = (40, 110, 2.08); # attenuation model parameters
my $max_retr = 8; # max number of retransmisssions per packet
my $bw = 125000; # channel bandwidth
my $cr = 1; # Coding Rate
my @Ptx_l = (2, 7, 14); # dBm
my @Ptx_w = (12 * 3.3 / 1000, 30 * 3.3 / 1000, 76 * 3.3 / 1000); # Ptx cons. for 2, 7, 14dBm (mW)
my $Prx_w = 46 * 3.3 / 1000;
my $Pidle_w = 30 * 3.3 / 1000; # this is actually the consumption of the microcontroller in idle mode
my @channels = (868100000, 868300000, 868500000, 867100000, 867300000, 867500000, 867700000, 867900000); # TTN channels
my %band = (868100000=>"48", 868300000=>"48", 868500000=>"48", 867100000=>"47", 867300000=>"47", 867500000=>"47", 867700000=>"47", 867900000=>"47"); # band name per channel (all of them with 1% duty cycle)
my $rx2sf = 9; # SF used for RX2 (LoRaWAN default = SF12, TTN uses SF9)
my $rx2ch = 869525000; # channel used for RX2 (LoRaWAN default = 869.525MHz, TTN uses the same)

# packet specific parameters
my @fpl = (51, 51, 51, 51, 51, 51); # uplink frame payload per SF (bytes)
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
my @pl_u = ($fpl[0]+$overhead_u, $fpl[1]+$overhead_u, $fpl[2]+$overhead_u, $fpl[3]+$overhead_u, $fpl[4]+$overhead_u, $fpl[5]+$overhead_u); # uplink packet payload per SF (bytes)
my %overlaps = (); # handles special packet overlaps 

# simulation parameters
my $confirmed_perc = 1; # percentage of nodes that require confirmed transmissions
my $full_collision = 1; # take into account non-orthogonal SF transmissions or not
my $period = 3600/$ARGV[0]; # time period between transmissions
my $sim_time = $ARGV[1]; # given simulation time
my $debug = 0; # enable debug mode
my $sim_end = 0;
my ($terrain, $norm_x, $norm_y) = (0, 0, 0); # terrain side, normalised terrain side
my $start_time = time; # just for statistics
my $successful = 0; # number of delivered packets (not necessarily acked)
my $dropped = 0; # number of dropped packets (for confirmed traffic)
my $dropped_unc = 0; # number of dropped packets (for unconfirmed traffic)
my $total_trans = 0; # number of transm. packets
my $total_retrans = 0; # number of re-transm packets
my $no_rx1 = 0; # no gw was available in RX1
my $no_rx2 = 0; # no gw was available in RX1 or RX2
my $picture = 0; # generate an energy consumption map
my $fixed_packet_rate = 1; # send packets periodically with a fixed rate (=1) or at random (=0)
my $total_down_time = 0; # total downlink time
my $progress_bar = 0; # activate progress bar (slower!)
my $avg_sf = 0;
my @sf_distr = (0, 0, 0, 0, 0, 0);

# application server
my $policy = $ARGV[2]; # gateway selection policy for downlink traffic
my %prev_seq = ();
my %appacked = (); # counts the number of acked packets per node
my %appsuccess = (); # counts the number of packets that received from at least one gw per node

my $progress;
if ($progress_bar == 1){
	$progress = Term::ProgressBar->new({
		count => $sim_time,
		ETA   => 'linear',
		remove => 1
	});
	$progress->minor(0);
	$progress->max_update_rate(1);
}
my $next_update = 0;

read_data(); # read terrain file

# first transmission
my @init_trans = ();
foreach my $n (keys %ncoords){
	my $start = random_uniform(1, 0, $period);
	my $sf = min_sf($n);
	$avg_sf += $sf;
	my $stop = $start + airtime($sf);
	print "# $n will transmit from $start to $stop (SF $sf)\n" if ($debug == 1);
	$nunique{$n} = 1;
	push (@init_trans, [$n, $start, $stop, $channels[rand @channels], $sf, $nunique{$n}]);
	$nconsumption{$n} += airtime($sf) * $Ptx_w[$nptx{$n}] + (airtime($sf)+1) * $Pidle_w; # +1sec for sensing
	$total_trans += 1;
}

# sort transmissions in ascending order
my @sorted_t = sort { $a->[1] <=> $b->[1] } @init_trans;
undef @init_trans;

# main loop
while (1){
	print "-------------------------------\n" if ($debug == 1);
	# grab the earliest transmission
	my ($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $sel_seq) = @{shift(@sorted_t)};
	$next_update = $progress->update($sel_end) if ($progress_bar == 1);
	if ($sel_sta > $sim_time){
		if ($progress_bar == 1){
			$next_update = $progress->update($sim_time);
			$progress->update($sim_time);
		}
		last;
	}
	print "# grabbed $sel, transmission from $sel_sta -> $sel_end\n" if ($debug == 1);
	$sim_end = $sel_end;
	
	if ($sel =~ /^[0-9]/){ # if the packet is an uplink transmission
		
		my $gw_rc = node_col($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf); # check collisions and compute a list of gws that received the uplink pkt
		my $rwindow = 0;
		my $failed = 0;
		if ((scalar @$gw_rc > 0) && ($nconfirmed{$sel} == 1)){ # if at least one gateway received the pkt -> successful transmission
			$successful += 1;
			$appsuccess{$sel} += 1 if ($sel_seq > $prev_seq{$sel});
			printf "# $sel 's transmission received by %d gateway(s) (channel $sel_ch)\n", scalar @$gw_rc if ($debug == 1);
			# now we have to find which gateway (if any) can transmit an ack in RX1 or RX2
			
			# check RX1
			my ($sel_gw, $sel_p) = gs_policy($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $gw_rc, 1);
			if (defined $sel_gw){
				my ($ack_sta, $ack_end) = ($sel_end+1, $sel_end+1+airtime($sel_sf, $overhead_d));
				$total_down_time += airtime($sel_sf, $overhead_d);
				$rwindow = 1;
				print "# gw $sel_gw will transmit an ack to $sel (RX$rwindow) (channel $sel_ch)\n" if ($debug == 1);
				$gresponses{$sel_gw} += 1;
				push (@{$gunavailability{$sel_gw}}, [$ack_sta, $ack_end, $sel_ch, $sel_sf, "d"]);
				$gdc{$sel_gw}{$band{$sel_ch}} = $ack_end+airtime($sel_sf, $overhead_d)*99;
				my $new_name = $sel_gw.$gresponses{$sel_gw}; # e.g. A1
				# place new transmission at the correct position
				my $i = 0;
				foreach my $el (@sorted_t){
					my ($n, $sta, $end, $ch, $sf, $seq) = @$el;
					last if ($sta > $ack_sta);
					$i += 1;
				}
				$appacked{$sel} += 1 if ($sel_seq > $prev_seq{$sel});
				splice(@sorted_t, $i, 0, [$new_name, $ack_sta, $ack_end, $sel_ch, $sel_sf, $appacked{$sel}]);
				push (@{$gdest{$sel_gw}}, [$sel, $sel_end+$rwindow, $sel_sf, $rwindow, $sel_ch, -1]);
			}else{
				# check RX2
				$no_rx1 += 1;
				($sel_gw, $sel_p) = gs_policy($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $gw_rc, 2);
				if (defined $sel_gw){
					my $bnd = "54"; # band: 54 for 869525000
					my ($ack_sta, $ack_end) = ($sel_end+2, $sel_end+2+airtime($rx2sf, $overhead_d));
					$total_down_time += airtime($rx2sf, $overhead_d);
					$rwindow = 2;
					print "# gw $sel_gw will transmit an ack to $sel (RX$rwindow) (channel $rx2ch)\n" if ($debug == 1);
					$gresponses{$sel_gw} += 1;
					push (@{$gunavailability{$sel_gw}}, [$ack_sta, $ack_end, $rx2ch, $rx2sf, "d"]);
					$gdc{$sel_gw}{$bnd} = $ack_end+airtime($rx2sf, $overhead_d)*9;
					my $new_name = $sel_gw.$gresponses{$sel_gw};
					my $i = 0;
					foreach my $el (@sorted_t){
						my ($n, $sta, $end, $ch, $sf, $seq) = @$el;
						last if ($sta > $ack_sta);
						$i += 1;
					}
					$appacked{$sel} += 1 if ($sel_seq > $prev_seq{$sel});
					splice(@sorted_t, $i, 0, [$new_name, $ack_sta, $ack_end, $rx2ch, $rx2sf, $appacked{$sel}]);
					push (@{$gdest{$sel_gw}}, [$sel, $sel_end+$rwindow, $rx2sf, $rwindow, $rx2ch, -1]);
				}else{
					$no_rx2 += 1;
					print "# no gateway is available\n" if ($debug == 1);
					$failed = 1;
				}
			}
			$prev_seq{$sel} = $sel_seq;
			if (defined $sel_gw){
				# ADR: the SF is already adjusted in min_sf; here only the transmit power is adjusted
				my $gap = $sel_p - $sensis[$sel_sf-7][bwconv($bw)];
				my $new_ptx = undef;
				foreach my $p (sort {$a<=>$b} @Ptx_l){
					next if ($p >= $Ptx_l[$nptx{$sel}]); # we can only decrease power for the moment
					if ($gap-$Ptx_l[$nptx{$sel}]+$p >= 10){
						$new_ptx = $p;
						last;
					}
				}
				if (defined $new_ptx){
					my $new_index = 0;
					foreach my $p (@Ptx_l){
						if ($p == $new_ptx){
							last;
						}
						$new_index += 1;
					}
					$gdest{$sel_gw}[-1][5] = $new_index;
					print "# it will be suggested that $sel changes tx power to $Ptx_l[$new_index]\n" if ($debug == 1);
				}
			}
		}elsif ((scalar @$gw_rc > 0) && ($nconfirmed{$sel} == 0)){ # successful transmission but no ack is required
			$successful += 1;
			$nacked{$sel} += 1;
			my $at = airtime($sel_sf, $pl_u[$sel_sf-7]);
			$sel_sta = $sel_end + $nperiod{$sel} + rand(1);
			my $next_allowed = $sel_end + 99*$at;
			if ($sel_sta < $next_allowed){
				print "# warning! transmission will be postponed due to duty cycle restrictions!\n" if ($debug == 1);
				$sel_sta = $next_allowed;
			}
			$sel_end = $sel_sta + $at;
			# place the new transmission at the correct position
			my $i = 0;
			foreach my $el (@sorted_t){
				my ($n, $sta, $end, $ch_, $sf_, $seq) = @$el;
				last if ($sta > $sel_sta);
				$i += 1;
			}
			$nunique{$sel} += 1;
			splice(@sorted_t, $i, 0, [$sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $nunique{$sel}]);
			$total_trans += 1 if ($sel_sta < $sim_time); # do not count transmissions that exceed the simulation time
			$total_retrans += 1 if (($sel_sta < $sim_time) && ($nconfirmed{$sel} == 1));
			print "# $sel, new transmission at $sel_sta -> $sel_end\n" if ($debug == 1);
			$nconsumption{$sel} += $at * $Ptx_w[$nptx{$sel}] + (airtime($sel_sf)+1) * $Pidle_w if ($sel_sta < $sim_time);
		}else{ # non-successful transmission
			$failed = 1;
		}
		if ($failed == 1){
			my $at = 0;
			if ($nconfirmed{$sel} == 1){
				if ($nretransmisssions{$sel} < $max_retr){
					$nretransmisssions{$sel} += 1;
					my $new_ch = $channels[rand @channels];
					$new_ch = $channels[rand @channels] while ($new_ch == $sel_ch);
					$sel_ch = $new_ch;
				}else{
					$dropped += 1;
					$nretransmisssions{$sel} = 0;
					$nunique{$sel} += 1;
					print "# $sel 's packet lost!\n" if ($debug == 1);
				}
				# the node stays on only for the duration of the preamble for both receive windows
				$nconsumption{$sel} += $preamble*(2**$sel_sf)/$bw * ($Prx_w + $Pidle_w);
				$nconsumption{$sel} += $preamble*(2**$rx2sf)/$bw * ($Prx_w + $Pidle_w);
				# plan the next transmission as soon as the duty cycle permits that
				$at = airtime($sel_sf);
				$sel_sta = $sel_end + 2 + rand(3);
				$sel_sta = $sel_end + 99*$at + rand(1) if ($sel_sta < ($ndc{$sel}{$sel_ch} + 99*$at));
			}else{
				$dropped_unc += 1;
				$prev_seq{$sel} = $sel_seq;
				$nunique{$sel} += 1;
				print "# $sel 's packet lost!\n" if ($debug == 1);
				$at = airtime($sel_sf, $pl_u[$sel_sf-7]);
				$sel_sta = $sel_end + $nperiod{$sel} + rand(1);
				my $next_allowed = $sel_end + 99*$at;
				if ($sel_sta < $next_allowed){
					print "# warning! transmission will be postponed due to duty cycle restrictions!\n" if ($debug == 1);
					$sel_sta = $next_allowed;
				}
			}
			$sel_end = $sel_sta+$at;
			$ndc{$sel}{$sel_ch} = $sel_end;
			# place the new transmission at the correct position
			my $i = 0;
			foreach my $el (@sorted_t){
				my ($n, $sta, $end, $ch_, $sf_, $seq) = @$el;
				last if ($sta > $sel_sta);
				$i += 1;
			}
			splice(@sorted_t, $i, 0, [$sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $nunique{$sel}]);
			$total_trans += 1;# if ($sel_sta < $sim_time); # do not count transmissions that exceed the simulation time
			$total_retrans += 1 if ($nconfirmed{$sel} == 1);# if (($sel_sta < $sim_time) && ($nconfirmed{$sel} == 1));
			print "# $sel, new transmission at $sel_sta -> $sel_end\n" if ($debug == 1);
			$nconsumption{$sel} += $at * $Ptx_w[$nptx{$sel}] + (airtime($sel_sf)+1) * $Pidle_w;# if ($sel_sta < $sim_time);
		}
		foreach my $g (keys %gcoords){
			$surpressed{$sel}{$g} = 0;
		}
		
		
	}else{ # if the packet is a gw transmission
		
		
		$sel =~ s/[0-9].*//; # keep only the letter(s)
		# remove the examined tuple of gw unavailability
		my @indices = ();
		my $index = 0;
		foreach my $tuple (@{$gunavailability{$sel}}){
			my ($sta, $end, $ch, $sf, $m) = @$tuple;
			push (@indices, $index) if ($end < $sel_sta);
			$index += 1;
		}
		for (sort {$b<=>$a} @indices){
			splice @{$gunavailability{$sel}}, $_, 1;
		}
		
		# look for the examined transmission in gdest, get some info, and then remove it 
		my $failed = 0;
		$index = 0;
		# ($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $sel_seq) information we already have
		my ($dest, $st, $sf, $rwindow, $ch, $pow); # we also need dest, rwindow, and pow (the others should be the same)
		foreach my $tup (@{$gdest{$sel}}){
			my ($dest_, $st_, $sf_, $rwindow_, $ch_, $p_) = @$tup;
			if (($st_ == $sel_sta) && ($sf_ == $sel_sf) && ($ch_ == $sel_ch)){
				($dest, $st, $sf, $rwindow, $ch, $pow) = ($dest_, $st_, $sf_, $rwindow_, $ch_, $p_);
				last;
			}
			$index += 1;
		}
		splice @{$gdest{$sel}}, $index, 1;
		# check if the transmission can reach the node
		my $G = rand(1);
		my $d = distance($gcoords{$sel}[0], $ncoords{$dest}[0], $gcoords{$sel}[1], $ncoords{$dest}[1]);
		my $prx = 14 - ($Lpld0 + 10*$gamma * log10($d/$dref) + $G*$var);
		if ($prx < $sensis[$sel_sf-7][bwconv($bw)]){
			print "# ack didn't reach node $dest\n" if ($debug == 1);
			$failed = 1;
		}
		# check if transmission time overlaps with other transmissions
		foreach my $tr (@sorted_t){
			my ($n, $sta, $end, $ch_, $sf_, $seq) = @$tr;
			last if ($sta > $sel_end);
			$n =~ s/[0-9].*// if ($n =~ /^[A-Z]/);
			next if (($n eq $sel) || ($end < $sel_sta) || ($ch_ != $ch)); # skip non-overlapping transmissions or different channels
			
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
			if ($n =~ /^[0-9]/){
				$d_ = distance($ncoords{$dest}[0], $ncoords{$n}[0], $ncoords{$dest}[1], $ncoords{$n}[1]);
				$p = $Ptx_l[$nptx{$n}];
			}else{
				$d_ = distance($ncoords{$dest}[0], $gcoords{$n}[0], $ncoords{$dest}[1], $gcoords{$n}[1]);
				$p = 14;
			}
			my $prx_ = $p - ($Lpld0 + 10*$gamma * log10($d_/$dref) + $G_*$var);
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
		if ($failed == 0){
			print "# ack successfully received, $dest 's transmission has been acked\n" if ($debug == 1);
			$nretransmisssions{$dest} = 0;
			$nacked{$dest} += 1;
			$nunique{$dest} += 1;
			if ($rwindow == 2){ # also count the RX1 window
				$nconsumption{$dest} += $preamble*(2**$sf)/$bw * ($Prx_w + $Pidle_w);
			}
			my $extra_bytes = 0; # if an ADR request is included in the downlink packet
			if ($pow != -1){
				$nptx{$dest} = $pow;
				$extra_bytes = $adr;
				$nresponse{$dest} = 1;
				print "# transmit power of $dest is set to $Ptx_l[$pow]\n" if ($debug == 1);
			}
			$nconsumption{$dest} += airtime($sel_sf, $overhead_d+$extra_bytes) * ($Prx_w + $Pidle_w);
			delete $won{$dest} if (exists $won{$dest});
		}else{ # ack was not received
			if ($nretransmisssions{$dest} < $max_retr){
				$nretransmisssions{$dest} += 1;
			}else{
				$dropped += 1;
				$nretransmisssions{$dest} = 0;
				$nunique{$dest} += 1;
				print "# $dest 's packet lost (no ack received)!\n" if ($debug == 1);
			}
			# $ch = $channels[rand @channels];
			$nconsumption{$dest} += $preamble*(2**$sf)/$bw * ($Prx_w + $Pidle_w);
			$nconsumption{$dest} += $preamble*(2**$rx2sf)/$bw * ($Prx_w + $Pidle_w);
		}
		@{$overlaps{$sel}} = ();
		# plan next transmission
		$ch = $channels[rand @channels];
		my $extra_bytes = 0;
		if ($nresponse{$dest} == 1){
			$extra_bytes = $adr;
			$nresponse{$dest} = 0;
		}
		my $at = airtime($sf, $pl_u[$sf-7]+$extra_bytes);
		my $new_start = $sel_sta - $rwindow + $nperiod{$dest} + rand(1);
		$new_start = $sel_sta - $rwindow + rand(3) if ($failed == 1);
		my $next_allowed = $ndc{$dest}{$ch} + 99*$at;
		if ($new_start < $next_allowed){
			print "# warning! transmission will be postponed due to duty cycle restrictions!\n" if ($debug == 1);
			$new_start = $next_allowed;
		}
		my $new_end = $new_start + $at;
		$ndc{$dest}{$ch} = $new_end;
		my $i = 0;
		foreach my $el (@sorted_t){
			my ($n, $sta, $end, $ch_, $sf_, $seq) = @$el;
			last if ($sta > $new_start);
			$i += 1;
		}
		splice(@sorted_t, $i, 0, [$dest, $new_start, $new_end, $ch, $sf, $nunique{$dest}]);
		$total_trans += 1;# if ($new_start < $sim_time); # do not count transmissions that exceed the simulation time
		$total_retrans += 1 if ($failed == 1);# && ($new_start < $sim_time)); 
		print "# $dest, new transmission at $new_start -> $new_end\n" if ($debug == 1);
		$nconsumption{$dest} += $at * $Ptx_w[$nptx{$dest}] + (airtime($sf)+1) * $Pidle_w;# if ($new_start < $sim_time);
	}
}
# print "---------------------\n";

my $avg_cons = (sum values %nconsumption)/(scalar keys %nconsumption);
my $min_cons = min values %nconsumption;
my $max_cons = max values %nconsumption;
my $finish_time = time;
printf "Simulation time = %.3f secs\n", $sim_end;
printf "Avg node consumption = %.5f mJ\n", $avg_cons;
printf "Min node consumption = %.5f mJ\n", $min_cons;
printf "Max node consumption = %.5f mJ\n", $max_cons;
print "Total number of transmissions = $total_trans\n";
print "Total number of re-transmissions = $total_retrans\n";
printf "Total number of unique transmissions = %d\n", (sum values %nunique);
print "Total packets delivered = $successful\n";
printf "Total packets acknowledged = %d\n", (sum values %nacked);
print "Total confirmed dropped = $dropped\n";
print "Total unconfirmed packets dropped = $dropped_unc\n";
printf "Packet Delivery Ratio = %.5f\n", (sum values %nacked)/(sum values %nunique); # unique packets delivered / unique packets transmitted
printf "Packet Reception Ratio = %.5f\n", $successful/$total_trans; # Global PRR
print "No GW available in RX1 = $no_rx1 times\n";
print "No GW available in RX1 or RX2 = $no_rx2 times\n";
print "Total downlink time = $total_down_time sec\n";
printf "Script execution time = %.4f secs\n", $finish_time - $start_time;
print "-----\n";
foreach my $g (sort keys %gcoords){
	print "GW $g sent out $gresponses{$g} acks\n";
}
my @fairs = ();
foreach my $n (keys %ncoords){
# 	push(@fairs, $nacked{$n}/$nunique{$n});
	$appsuccess{$n} = 1 if ($appsuccess{$n} == 0);
	push(@fairs, $appacked{$n}/$appsuccess{$n});
}
printf "Mean downlink fairness = %.3f\n", mean(\@fairs);
printf "Stdv of downlink fairness = %.3f\n", stddev(\@fairs);
print "-----\n";
for (my $sf=7; $sf<=12; $sf+=1){
	printf "# of nodes with SF%d: %d\n", $sf, $sf_distr[$sf-7];
}
printf "Avg SF = %.3f\n", $avg_sf/(scalar keys %ncoords);
generate_picture() if ($picture == 1);


sub gs_policy{ # gateway selection policy
	my ($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf, $gw_rc, $win) = @_;
	my $sel_gw = undef;
	my $bnd = $band{$sel_ch};
	if ($win == 2){
		$bnd = "54";
		$sel_ch = $rx2ch;
		if ($sel_sf < $rx2sf){
			@$gw_rc = @{$nreachablegws{$sel}};
		}
		$sel_sf = $rx2sf;
	}
	my ($ack_sta, $ack_end) = ($sel_end+$win, $sel_end+$win+airtime($sel_sf, $overhead_d));
	my ($min_resp, $sel_p, $min_dc) = (1, -9999999999999, 9999999999999);
	foreach my $g (@$gw_rc){
		my ($gw, $p) = @$g;
		next if ($gdc{$gw}{$bnd} > ($sel_end+$win));
		my $is_available = 1;
		foreach my $gu (@{$gunavailability{$gw}}){
			my ($sta, $end, $ch, $sf, $m) = @$gu;
			if ( (($ack_sta >= $sta) && ($ack_sta <= $end)) || (($ack_end <= $end) && ($ack_end >= $sta)) ){
				$is_available = 0;
				last;
			}
		}
		next if ($is_available == 0);
		if ($policy == 1){ # FCFS
			my $resp = rand(2)/10;
			if ($resp < $min_resp){
				$min_resp = $resp;
				$sel_gw = $gw;
				$sel_p = $p;
			}
		}elsif (($policy == 2) || ($policy == 4)){ # RSSI
			if ($p > $sel_p){
				$sel_gw = $gw;
				$sel_p = $p;
			}
		}elsif ($policy == 3){ # less busy gw
			if ($gdc{$gw}{$bnd} < $min_dc){
				$min_dc = $gdc{$gw}{$bnd};
				$sel_gw = $gw;
				$sel_p = $p;
			}
		}
	}
	return ($sel_gw, $sel_p);
}

sub node_col{ # handle node collisions
	my ($sel, $sel_sta, $sel_end, $sel_ch, $sel_sf) = @_;
	# check for collisions with other transmissions (time, SF, power) per gw
	my @gw_rc = ();
	foreach my $gw (keys %gcoords){
		next if ($surpressed{$sel}{$gw} == 1);
		my $d = distance($gcoords{$gw}[0], $ncoords{$sel}[0], $gcoords{$gw}[1], $ncoords{$sel}[1]);
		my $G = rand(1);
		my $prx = $Ptx_l[$nptx{$sel}] - ($Lpld0 + 10*$gamma * log10($d/$dref) + $G*$var);
		if ($prx < $sensis[$sel_sf-7][bwconv($bw)]){
			$surpressed{$sel}{$gw} = 1;
			print "# packet didn't reach gw $gw\n" if ($debug == 1);
			next;
		}
		# check if the gw is available for uplink
		my $is_available = 1;
		foreach my $gu (@{$gunavailability{$gw}}){
			my ($sta, $end, $ch, $sf, $m) = @$gu;
			if ( (($sel_sta >= $sta) && ($sel_sta <= $end)) || (($sel_end <= $end) && ($sel_end >= $sta)) || (($sel_sta == $sta) && ($sel_end == $end))){
				# the gw has either locked to another transmission with the same ch/sf OR is being used for downlink
				if ( ($m eq "d") || (($m eq "u") && ($sel_ch == $ch) && ($sel_sf == $sf)) ){
					$is_available = 0;
					last;
				}
			}
		}
		if ($is_available == 0){
			$surpressed{$sel}{$gw} = 1;
			print "# gw not available for uplink (channel $sel_ch, SF $sel_sf)\n" if ($debug == 1);
			next;
		}
		foreach my $tr (@sorted_t){
			my ($n, $sta, $end, $ch, $sf, $seq) = @$tr;
			last if ($sta > $sel_end);
			if ($n =~ /^[0-9]/){ # node transmission
				next if (($n == $sel) || ($sta > $sel_end) || ($end < $sel_sta) || ($ch != $sel_ch));
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
				my $d_ = distance($gcoords{$gw}[0], $ncoords{$n}[0], $gcoords{$gw}[1], $ncoords{$n}[1]);
				my $prx_ = $Ptx_l[$nptx{$n}] - ($Lpld0 + 10*$gamma * log10($d_/$dref) + rand(1)*$var);
				if ($overlap == 3){
					if ((abs($prx - $prx_) <= $thresholds[$sel_sf-7][$sf-7]) ){ # both collide
						$surpressed{$sel}{$gw} = 1;
						$surpressed{$n}{$gw} = 1;
						print "# $sel collided together with $n at gateway $gw\n" if ($debug == 1);
					}
					if (($prx_ - $prx) > $thresholds[$sel_sf-7][$sf-7]){ # n suppressed sel
						$surpressed{$sel}{$gw} = 1;
						print "# $sel surpressed by $n at gateway $gw\n" if ($debug == 1);
					}
					if (($prx - $prx_) > $thresholds[$sf-7][$sel_sf-7]){ # sel suppressed n
						$surpressed{$n}{$gw} = 1;
						print "# $n surpressed by $sel at gateway $gw\n" if ($debug == 1);
					}
				}
				if (($overlap == 1) && ($full_collision == 1)){ # non-orthogonality
					if (($prx - $prx_) > $thresholds[$sel_sf-7][$sf-7]){
						if (($prx_ - $prx) <= $thresholds[$sf-7][$sel_sf-7]){
							$surpressed{$n}{$gw} = 1;
							print "# $n surpressed by $sel\n" if ($debug == 1);
						}
					}else{
						if (($prx_ - $prx) > $thresholds[$sf-7][$sel_sf-7]){
							$surpressed{$sel}{$gw} = 1;
							print "# $sel surpressed by $n\n" if ($debug == 1);
						}else{
							$surpressed{$sel}{$gw} = 1;
							$surpressed{$n}{$gw} = 1;
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
					my $G_ = rand(1);
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
					my $d_ = distance($gcoords{$gw}[0], $gcoords{$n}[0], $gcoords{$gw}[1], $gcoords{$n}[1]);
					my $prx_ = 14 - ($Lpld0 + 10*$gamma * log10($d_/$dref) + $G_*$var);
					if ($overlap == 3){
						if ((abs($prx - $prx_) <= $thresholds[$sel_sf-7][$sf_-7]) ){ # both collide
							$surpressed{$sel}{$gw} = 1;
							print "# $sel collided together with $n at gateway $gw\n" if ($debug == 1);
						}
						if (($prx_ - $prx) > $thresholds[$sel_sf-7][$sf_-7]){ # n suppressed sel
							$surpressed{$sel}{$gw} = 1;
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
								$surpressed{$sel}{$gw} = 1;
								print "# $sel surpressed by $n\n" if ($debug == 1);
							}else{
								$surpressed{$sel}{$gw} = 1;
								print "# $sel collided together with $n\n" if ($debug == 1);
							}
						}
					}
				}
			}
		}
		if ($surpressed{$sel}{$gw} == 0){
			push (@gw_rc, [$gw, $prx]);
			# set the gw unavailable (exclude preamble) and locked to the specific Ch/SF
			my $pr_time = 2**$sel_sf/$bw;
			push(@{$gunavailability{$gw}}, [$sel_sta+$pr_time, $sel_end, $sel_ch, $sel_sf, "u"]);
		}
	}
	@{$overlaps{$sel}} = ();
	return (\@gw_rc);
}

sub min_sf{
	my $n = shift;
	my $G = rand(1);
	my $Xs = $var*$G;
	my $sf = 0;
	my $bwi = bwconv($bw);
	foreach my $gw (keys %gcoords){
		my $d0 = distance($gcoords{$gw}[0], $ncoords{$n}[0], $gcoords{$gw}[1], $ncoords{$n}[1]);
		for (my $f=7; $f<=12; $f+=1){
			my $S = $sensis[$f-7][$bwi];
			my $Prx = $Ptx_l[$nptx{$n}] - ($Lpld0 + 10*$gamma * log10($d0/$dref) + $Xs);
			if (($Prx - 10) > $S){ # 10dBm tolerance
				$sf = $f;
				$f = 13;
				last;
			}
		}
	}
	# check which gateways can be reached with rx2sf
	foreach my $gw (keys %gcoords){
		my $d0 = distance($gcoords{$gw}[0], $ncoords{$n}[0], $gcoords{$gw}[1], $ncoords{$n}[1]);
		my $S = $sensis[$rx2sf-7][$bwi];
		my $Prx = $Ptx_l[$nptx{$n}] - ($Lpld0 + 10*$gamma * log10($d0/$dref) + $Xs);
		if (($Prx - 10) > $S){ # 10dBm tolerance
			push(@{$nreachablegws{$n}}, [$gw, $Prx]);
		}
	}
	if ($sf == 0){
		print "node $n unreachable!\n";
		print "terrain too large?\n";
		exit;
	}
	# print "# $n can reach a gw with SF$sf\n" if ($debug == 1);
	$sf_distr[$sf-7] += 1;
	return $sf;
}

# a modified version of LoRaSim (https://www.lancaster.ac.uk/scc/sites/lora/lorasim.html)
sub airtime{
	my $sf = shift;
	my $DE = 0;      # low data rate optimization enabled (=1) or not (=0)
	my $payload = shift;
	$payload = $pl_u[$sf-7] if (!defined $payload);
	if (($bw == 125000) && (($sf == 11) || ($sf == 12))){
		# low data rate optimization mandated for BW125 with SF11 and SF12
		$DE = 1;
	}
	my $Tsym = (2**$sf)/$bw;
	my $Tpream = ($preamble + 4.25)*$Tsym;
	my $payloadSymbNB = 8 + max( ceil((8.0*$payload-4.0*$sf+28+16*$CRC-20*$H)/(4.0*($sf-2*$DE)))*($cr+4), 0 );
	my $Tpayload = $payloadSymbNB * $Tsym;
	return ($Tpream + $Tpayload);
}

sub bwconv{
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
	my $terrain_file = $ARGV[3];
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
		$nptx{$n} = scalar @Ptx_l - 1; # start with the highest Ptx
		$nresponse{$n} = 0;
		$nretransmisssions{$n} = 0;
		if ($conf_num > 0){
			$nconfirmed{$n} = 1;
			$conf_num -= 1;
		}else{
			$nconfirmed{$n} = 0;
		}
		$appacked{$n} = 0;
		$appsuccess{$n} = 0;
		$nacked{$n} = 0;
		$prev_seq{$n} = 0;
		if ($fixed_packet_rate == 0){
			my @per = random_exponential(scalar keys @nodes, 2*$period);
			foreach my $n (keys %ncoords){
				$nperiod{$n} = pop(@per);
			}
		}else{
			$nperiod{$n} = $period;
		}
		foreach my $ch (@channels){
			$ndc{$n}{$ch} = -9999999999999;
		}
	}
	foreach my $gw (@gateways){
		my ($g, $x, $y) = @$gw;
		$gcoords{$g} = [$x, $y];
		@{$gunavailability{$g}} = ();
		foreach my $ch (@channels){
			$gdc{$g}{$band{$ch}} = 0;
		}
		$gdc{$g}{"54"} = 0;
		foreach my $n (keys %ncoords){
			$surpressed{$n}{$g} = 0;
		}
		@{$overlaps{$g}} = ();
		$gresponses{$g} = 0;
	}
}

sub distance {
	my ($x1, $x2, $y1, $y2) = @_;
	return sqrt( (($x1-$x2)*($x1-$x2))+(($y1-$y2)*($y1-$y2)) );
}

sub distance3d {
	my ($x1, $x2, $y1, $y2, $z1, $z2) = @_;
	return sqrt( (($x1-$x2)*($x1-$x2))+(($y1-$y2)*($y1-$y2))+(($z1-$z2)*($z1-$z2)) );
}

sub generate_picture{
	my ($display_x, $display_y) = (800, 800); # 800x800 pixel display pane
	my $im = new GD::SVG::Image($display_x, $display_y);
	my $blue = $im->colorAllocate(0,0,255);
	my $black = $im->colorAllocate(0,0,0);
	my $red = $im->colorAllocate(255,0,0);
	
	foreach my $n (keys %ncoords){
		my ($x, $y) = @{$ncoords{$n}};
		($x, $y) = (int(($x * $display_x)/$norm_x), int(($y * $display_y)/$norm_y));
# 		my $color = $im->colorAllocate(255*$nconsumption{$n}/$max_cons,0,0);
		my $color = $im->colorAllocate(255*(min_sf($n)-7)/5,0,0);
		$im->filledArc($x,$y,20,20,0,360,$color);
	}
	
	foreach my $g (keys %gcoords){
		my ($x, $y) = @{$gcoords{$g}};
		($x, $y) = (int(($x * $display_x)/$norm_x), int(($y * $display_y)/$norm_y));
		$im->rectangle($x-5, $y-5, $x+5, $y+5, $red);
		$im->string(gdGiantFont,$x-2,$y-20,$g,$blue);
	}
	my $output_file = $ARGV[3]."-img.svg";
	open(FILEOUT, ">$output_file") or die "could not open file $output_file for writing!";
	binmode FILEOUT;
	print FILEOUT $im->svg;
	close FILEOUT;
}
