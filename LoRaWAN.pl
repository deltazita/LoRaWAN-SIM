#!/usr/bin/perl -w

###################################################################################
#          Event-based simulator for confirmable LoRaWAN transmissions            #
#                                 v2020.5.11                                      #
#                                                                                 #
# Features:                                                                       #
# -- Multiple half-duplex gateways                                                #
# -- 1% radio duty cycle for the nodes                                            #
# -- 10% radio duty cycle for the gateways                                        #
# -- Non-orthogonal SF transmissions                                              #
# -- Capture effect                                                               #
# -- Acks with two receive windows (RX1, RX2)                                     #
# -- Path-loss signal attenuation model                                           #
#                                                                                 #
# Assumptions (or work in progress):                                              #
# -- All uplink transmissions are performed over the same channel                 #
# -- Acks do not collide to each other                                            #
#                                                                                 #
# author: Dr. Dimitrios Zorbas                                                    #
# email: dimzorbas@ieee.org                                                       #
# distributed under GNUv2 General Public Licence                                  #
###################################################################################

use strict;
use POSIX;
use List::Util qw(min max);
use Time::HiRes qw(time);
use Math::Random qw(random_uniform random_exponential);

die "usage: ./LoRaWAN.pl full_collision_check(0/1) packets_per_hour simulation_time(secs) terrain_file!\n" unless (scalar @ARGV == 4);

# node attributes
my %ncoords = (); # node coordinates
my %consumption = (); # consumption
my %SF = (); # Spreading Factor
my %transmissions = (); # current transmission
my %retransmisssions = (); # retransmisssions per node
my %surpressed = ();
my $period = 3600/$ARGV[1]; # packets per hour

# gw attributes
my %gcoords = (); # gw coordinates
my %gunavailability = (); # unavailable gw time due to downlink
my %gdc = (); # gw duty cycle 

# simulation and propagation parameters
my $max_retr = 8; # max number of retransmisssions per packet
my @sensis = ([7,-124,-122,-116], [8,-127,-125,-119], [9,-130,-128,-122], [10,-133,-130,-125], [11,-135,-132,-128], [12,-137,-135,-129]); # sensitivities per SF/BW
my $bw = 125; # bandwidth
my $var = 3.57; # variance
my ($dref, $Ptx, $Lpld0, $gamma) = (40, 14, 95, 2.08); # attenuation model parameters
my $Ptx_w = 76 * 3.5 / 1000; # mW
my $Prx_w = 46 * 3.5 / 1000;
my $Pidle_w = 30 / 1000;
my @thresholds = ([6,-16,-18,-19,-19,-20], [-24,6,-20,-22,-22,-22], [-27,-27,6,-23,-25,-25], [-30,-30,-30,6,-26,-28], [-33,-33,-33,-33,6,-29], [-36,-36,-36,-36,-36,6]); # capture effect power thresholds per SF[SF] for non-orthogonal transmissions
my $full_collision = $ARGV[0]; # take into account non-orthogonal SF transmissions or not
my $sim_time = $ARGV[2];

my @pl = (50, 50, 50, 50, 50, 50); # payload size per SF (bytes)
my ($terrain, $norm_x, $norm_y) = (0, 0, 0); # terrain side (not currenly being used)
my $start_time = time; # just for statistics
my $dropped = 0; # number of dropped packets
my $total_trans = 0; # number of transm. packets
my $total_retrans = 0; # number of re-transm packets
my $acked = 0; # number of acknowledged packets
my $no_rx1 = 0; # number of times none of gw was not available in RX1
my $no_rx2 = 0; # number of times none of gw was not available in RX1 or RX2

read_data(); # read terrain file

foreach my $n (keys %ncoords){
	$SF{$n} = min_sf($n);
	$retransmisssions{$n} = 0;
}

my @examined = ();

# initial transmission
foreach my $n (keys %ncoords){
	my $start = random_uniform(1, 0, $period);
	my $stop = $start + airtime($SF{$n});
	print "# $n will transmit from $start to $stop\n";
	$transmissions{$n} = [$start, $stop];
	$consumption{$n} += airtime($SF{$n}) * $Ptx_w + (airtime($SF{$n})+1) * $Pidle_w;
	$total_trans += 1;
}

# main loop
while (1){
	print "-------------------------------\n";
	printf "# %d transmissions available \n", scalar keys %transmissions;
	# grab the earliest transmission
	my $sel_sta = 9999999999999;
	my $sel_end = 0;
	my $sel = undef;
	foreach my $n (keys %transmissions){
		my ($sta, $end) = @{$transmissions{$n}};
		if ($sta < $sel_sta){
			$sel_sta = $sta;
			$sel_end = $end;
			$sel = $n;
		}
	}
	last if (!defined $sel);
	print "# grabbed $sel, transmission at $sel_sta -> $sel_end\n";

	# check for collisions with other transmissions (time, SF, power) per gw
	my @gw_rc = ();
	my $rwindow = 0;
	my $to_retrans = 0;
	foreach my $gw (keys %gcoords){
		# print "# checking with gw $gw\n";
		next if ($surpressed{$sel}{$gw} == 1);
		my $d = distance($gcoords{$gw}[0], $ncoords{$sel}[0], $gcoords{$gw}[1], $ncoords{$sel}[1]);
		my $G = rand(1);
		my $prx = $Ptx - ($Lpld0 + 10*$gamma * log10($d/$dref) + $G*$var);
		if ($prx < $sensis[$SF{$sel}-7][bwconv($bw)]){
			$surpressed{$sel}{$gw} = 1;
			print "# packet didn't reach gw $gw\n";
			next;
		}
		foreach my $n (keys %transmissions){
			my ($sta, $end) = @{$transmissions{$n}};
			next if (($n == $sel) || ($sta > $sel_end) || ($end < $sel_sta));
			my $overlap = 0;
			# time overlap
			if ( (($sel_sta >= $sta) && ($sel_sta <= $end)) || (($sel_end <= $end) && ($sel_end >= $sta)) || (($sel_sta == $sta) && ($sel_end == $end)) ){
				$overlap += 1;
			}
			# SF
			if ($SF{$n} == $SF{$sel}){
				$overlap += 2;
			}
			# power 
			my $d_ = distance($gcoords{$gw}[0], $ncoords{$n}[0], $gcoords{$gw}[1], $ncoords{$n}[1]);
			my $prx_ = $Ptx - ($Lpld0 + 10*$gamma * log10($d_/$dref) + rand(1)*$var);
			if ($overlap == 3){
				if ((abs($prx - $prx_) <= $thresholds[$SF{$sel}-7][$SF{$n}-7]) ){ # both collide
					$surpressed{$sel}{$gw} = 1;
					$surpressed{$n}{$gw} = 1;
					print "# $sel collided together with $n at gateway $gw\n";
				}
				if (($prx_ - $prx) > $thresholds[$SF{$sel}-7][$SF{$n}-7]){ # n suppressed sel
					$surpressed{$sel}{$gw} = 1;
					print "# $sel surpressed by $n at gateway $gw\n";
				}
				if (($prx - $prx_) > $thresholds[$SF{$n}-7][$SF{$sel}-7]){ # sel suppressed n
					$surpressed{$n}{$gw} = 1;
					print "# $n surpressed by $sel at gateway $gw\n";
				}
			}
			if (($overlap == 1) && ($full_collision == 1)){ # non-orthogonality
				if (($prx - $prx_) > $thresholds[$SF{$sel}-7][$SF{$n}-7]){
					if (($prx_ - $prx) <= $thresholds[$SF{$n}-7][$SF{$sel}-7]){
						$surpressed{$n}{$gw} = 1;
						print "# $n surpressed by $sel\n";
					}
				}else{
					if (($prx_ - $prx) > $thresholds[$SF{$n}-7][$SF{$sel}-7]){
						$surpressed{$sel}{$gw} = 1;
						print "# $sel surpressed by $n\n";
					}else{
						$surpressed{$sel}{$gw} = 1;
						$surpressed{$n}{$gw} = 1;
						print "# $sel collided together with $n\n";
					}
				}
			}
		}
		if ($surpressed{$sel}{$gw} == 0){
			push (@gw_rc, [$gw, $prx]);
		}
	}
	if (scalar @gw_rc > 0){ # successful transmission
		# remove old transmission
		delete $transmissions{$sel};
		print "# $sel transmitted successfully!\n";
		# check which gw can send an ack (RX1)
		my $max_p = -9999999999999;
		my $sel_gw = undef;
		my ($ack_sta, $ack_end) = ($sel_end+1, $sel_end+1+airtime($SF{$sel}, 8));
		foreach my $g (@gw_rc){
			my ($gw, $p) = @$g;
			my ($sta, $end) = @{$gunavailability{$gw}};
			next if ( (($ack_sta >= $sta) && ($ack_sta <= $end)) || (($ack_end <= $end) && ($ack_end >= $sta)) || (($ack_sta == $sta) && ($ack_end == $end)) );
			next if ($gdc{$gw} > ($sel_end+1));
			if ($p > $max_p){
				$sel_gw = $gw;
				$max_p = $p;
			}
		}
		if (defined $sel_gw){
			print "# gw $sel_gw will transmit an ack to $sel\n";
			$gunavailability{$sel_gw} = [$ack_sta, $ack_end];
			$gdc{$sel_gw} = $ack_end+airtime($SF{$sel}, 8)*10;
			$retransmisssions{$sel} = 0;
			$acked += 1;
			$rwindow += 1;
			$consumption{$sel} += airtime($SF{$sel}, 8) * $Prx_w + (airtime($SF{$sel}, 8)+1) * $Pidle_w;
		}else{
			# check RX2
			$no_rx1 += 1;
			my ($ack_sta, $ack_end) = ($sel_end+2, $sel_end+2+airtime(12, 8));
			$max_p = -9999999999999;
			foreach my $g (@gw_rc){
				my ($gw, $p) = @$g;
				my ($sta, $end) = @{$gunavailability{$gw}};
				next if ( (($ack_sta >= $sta) && ($ack_sta <= $end)) || (($ack_end <= $end) && ($ack_end >= $sta)) || (($ack_sta == $sta) && ($ack_end == $end)) );
				next if ($gdc{$gw} > ($sel_end+2));
				if ($p > $max_p){
					$sel_gw = $gw;
					$max_p = $p;
				}
			}
			if (defined $sel_gw){
				print "# gw $sel_gw will transmit an ack to $sel\n";
				$gunavailability{$sel_gw} = [$ack_sta, $ack_end];
				$gdc{$sel_gw} = $ack_end+airtime(12, 8)*10;
				$retransmisssions{$sel} = 0;
				$acked += 1;
				$rwindow += 2;
				$consumption{$sel} += airtime(12, 8) * $Prx_w + (airtime(12, 8)+1) * $Pidle_w;
			}else{
				$no_rx2 += 1;
				print "# no gateway is available\n";
				if ($retransmisssions{$sel} < $max_retr){
					$retransmisssions{$sel} += 1;
					$to_retrans = 1;
				}else{
					$dropped += 1;
					$retransmisssions{$sel} = 0;
					print "# $sel 's packet lost!\n";
				}
			}
		}
	}else{ # delete collided transmission and assign a new one
		delete $transmissions{$sel};
		if ($retransmisssions{$sel} < $max_retr){
			$retransmisssions{$sel} += 1;
			$to_retrans = 1;
		}else{
			$dropped += 1;
			$retransmisssions{$sel} = 0;
			print "# $sel 's packet lost!\n";
		}
		$consumption{$sel} += airtime($SF{$sel}, 8) * $Prx_w + (airtime($SF{$sel}, 8)+1) * $Pidle_w;
		$consumption{$sel} += airtime(12, 8) * $Prx_w + (airtime(12, 8)+1) * $Pidle_w;
	}
	foreach my $g (keys %gcoords){
		$surpressed{$sel}{$g} = 0;
	}
	my $at = airtime($SF{$sel});
	$sel_sta = $sel_end + $rwindow + $period + rand(1);
	if ($sel_sta < $sim_time){
		$sel_end = $sel_sta+$at;
		$transmissions{$sel} = [$sel_sta, $sel_end];
		$total_trans += 1;
		$total_retrans += 1 if ($to_retrans == 1);
		print "# $sel, new transmission at $sel_sta -> $sel_end\n";
		$consumption{$sel} += $at * $Ptx_w + (airtime($SF{$sel})+1) * $Pidle_w;
	}
}
print "---------------------\n";

my $avg_cons = 0;
foreach my $n (keys %SF){
	$avg_cons += $consumption{$n};
}
my $finish_time = time;
print "Simulation time = $sim_time sec\n";
printf "Avg node consumption = %.5f J\n", $avg_cons/(scalar keys %SF);
print "Total number of transmissions = $total_trans\n";
print "Total number of re-transmissions = $total_retrans\n";
printf "Total number of unique transmissions = %d\n", $total_trans-$total_retrans;
print "Total packets dropped = $dropped\n";
printf "Packet Delivery Ratio = %.9f\n", $acked/($total_trans-$total_retrans);
print "No GW available in RX1 = $no_rx1 times\n";
print "No GW available in RX1 or RX2 = $no_rx2 times\n";
printf "Script execution time = %.4f secs\n", $finish_time - $start_time;

sub min_sf{
	my $n = shift;
	my $G = rand(1);
	my ($dref, $Ptx, $Lpld0, $Xs, $gamma) = (40, 7, 95, $var*$G, 2.08);
	my $sf = 0;
	my $bwi = bwconv($bw);
	foreach my $gw (keys %gcoords){
		my $d0 = distance($gcoords{$gw}[0], $ncoords{$n}[0], $gcoords{$gw}[1], $ncoords{$n}[1]);
		my $succ = 0;
		for (my $f=7; $f<=12; $f+=1){
			my $S = $sensis[$f-7][$bwi];
			my $Prx = $Ptx - ($Lpld0 + 10*$gamma * log10($d0/$dref) + $Xs);
			if (($Prx - 20) > $S){
				$sf = $f;
				$f = 13;
				last;
			}
		}
	}
	if ($sf == 0){
		print "node $n unreachable!\n";
		print "terrain too large?\n";
		exit;
	}
	print "# $n can reach a gw with SF$sf\n";
	return $sf;
}

sub airtime{
	my $sf = shift;
	my $cr = 1;
	my $H = 0;       # implicit header disabled (H=0) or not (H=1)
	my $DE = 0;      # low data rate optimization enabled (=1) or not (=0)
	my $Npream = 8;  # number of preamble symbol (12.25  from Utz paper)
	my $payload = shift;
	$payload = $pl[$sf-7] if (!defined $payload);
	
	if (($bw == 125) && (($sf == 11) || ($sf == 12))){
		# low data rate optimization mandated for BW125 with SF11 and SF12
		$DE = 1;
	}
	
	if ($sf == 6){
		# can only have implicit header with SF6
		$H = 1;
	}
	
	my $Tsym = (2**$sf)/$bw;
	my $Tpream = ($Npream + 4.25)*$Tsym;
	my $payloadSymbNB = 8 + max( ceil((8.0*$payload-4.0*$sf+28+16-20*$H)/(4.0*($sf-2*$DE)))*($cr+4), 0 );
	my $Tpayload = $payloadSymbNB * $Tsym;
	return ($Tpream + $Tpayload)/1000;
}

sub bwconv{
	my $bwi = 0;
	if ($bw == 125){
		$bwi = 1;
	}elsif ($bw == 250){
		$bwi = 2;
	}elsif ($bw == 500){
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
		} elsif (/^# node coords: (.*)/){
			my $sensor_coord = $1;
			my @coords = split(/\] /, $sensor_coord);
			@nodes = map { /([0-9]+) \[([0-9]+\.[0-9]+) ([0-9]+\.[0-9]+)/; [$1, $2, $3]; } @coords;
		} elsif (/^# gateway coords: (.*)/){
			my $gw_coord = $1;
			my @coords = split(/\] /, $gw_coord);
			@gateways = map { /([0-9]+) \[([0-9]+\.[0-9]+) ([0-9]+\.[0-9]+)/; [$1, $2, $3]; } @coords;
		}
	}
	close(FH);
	
	foreach my $node (@nodes){
		my ($n, $x, $y) = @$node;
		$ncoords{$n} = [$x, $y];
	}
	foreach my $gw (@gateways){
		my ($g, $x, $y) = @$gw;
		$gcoords{$g} = [$x, $y];
		$gunavailability{$g} = [0, 0];
		$gdc{$g} = 0;
		foreach my $n (keys %ncoords){
			$surpressed{$n}{$g} = 0;
		}
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
