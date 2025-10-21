#!/usr/bin/perl -w
#
# Script to create a 2D terrain of nodes
#
# modified copy of https://github.com/rainbow-src/sensors/tree/master/terrain%20generators
#
# WARNING! make sure the signal attenuation values (see lines 24-30) are the same as in LoRaWAN.pl

use strict;
use Math::Random;

(@ARGV==3) || die "usage: $0 <terrain_side_size_(m)> <num_of_nodes> <num_of_gateways>\ne.g. $0 2000 500 2\n";

my $tx = $ARGV[0];
my $nodes = $ARGV[1];
my $gws = $ARGV[2];

($tx < 1) && die "terrain side must be at least 1 meter!\n";
($nodes < 1) && die "number of nodes must be higher than 0!\n";
($gws < 1) && die "number of gateways must be higher than 0!\n";


### check max range ###
my $dref = 40;
my $Ptx = 14;
my $Prx = -137; # sensitivity for SF12BW125
my $Lpld0 = 110;
my $Xs = 0;
my $margin = 5;
my $gamma = 2.08;

my $dmax = $dref * 10 ** (($Ptx - $Prx - $Lpld0 - $Xs - $margin) / (10 * $gamma));
print "# max range = $dmax m\n";
#if ($tx*sqrt(2) > $dmax){
#	$tx = $dmax/sqrt(2);
#	print "# terrain side decreased to $tx m\n";
#}
###

my @sensors;
my @gws;
my %coords;

for(my $i=1; $i<=$gws; $i++){
	my ($x, $y) = (int(rand($tx*10)), int(rand($tx*10)));
	($x, $y) = ($x/10, $y/10);
	while (exists $coords{$x}{$y}){
		($x, $y) = (int(rand($tx*10)), int(rand($tx*10)));
		($x, $y) = ($x/10, $y/10);
	}
	$coords{$x}{$y} = 1;
	push(@gws, [$x, $y]);
}
for(my $i=1; $i<=$nodes; $i++){
	my ($x, $y) = (int(rand($tx*10)), int(rand($tx*10)));
	($x, $y) = ($x/10, $y/10);
	while (exists $coords{$x}{$y}){
		($x, $y) = (int(rand($tx*10)), int(rand($tx*10)));
		($x, $y) = ($x/10, $y/10);
	}
	my $conn_check = -1;
	my $loops = 0;
	while ($conn_check == -1){
		$conn_check = 0;
		foreach my $g (@gws){
			my ($gx, $gy) = @$g;
			if (distance($x, $gx, $y, $gy) > $dmax){
				$conn_check += 1;
			}	
		}
		if ($conn_check == (scalar @gws)){
			$conn_check = -1;
			$loops += 1;
			die "You have probably given a too large terrain side! Try with up to 3-4 times the max range\n" if ($loops == 50); # give up
			($x, $y) = (int(rand($tx*10)), int(rand($tx*10)));
			($x, $y) = ($x/10, $y/10);
		}
	}
	$coords{$x}{$y} = 1;
	push(@sensors, [$x, $y]);
}


printf "# terrain map [%i x %i]\n", $tx, $tx;
print "# node coords:";
my $n = 1;
foreach my $s (@sensors){
	my ($x, $y) = @$s;
	printf " %s [%.1f %.1f]", $n, $x, $y;
	$n++;
}
print "\n";
print "# gateway coords:";
my $l = "A";
foreach my $g (@gws){
	my ($x, $y) = @$g;
	printf " %s [%.1f %.1f]", $l, $x, $y;
	$l++;
}
print "\n";

print  "# generated with: $0 ",join(" ",@ARGV),"\n";
printf "# stats: nodes=%i gateways=%i terrain=%.1fm^2 node_sz=%.2fm^2\n", scalar @sensors, scalar @gws, $tx*$tx, 0.1 * 0.1;

sub distance {
	my ($x1, $x2, $y1, $y2) = @_;
        return sqrt( (($x1-$x2)*($x1-$x2))+(($y1-$y2)*($y1-$y2)) );
}
