#!/usr/bin/perl -w
#
# Script to create a 2D terrain of nodes using the minimum possible random gateway positions
#
# modified copy of https://github.com/rainbow-src/sensors/tree/master/terrain%20generators
#
# prerequisites: https://metacpan.org/pod/Math::Random, 
#                https://metacpan.org/pod/Algorithm::SetCovering

use strict;
use Math::Random;
use POSIX;
use Algorithm::SetCovering;

(@ARGV==2) || die "usage: $0 <terrain_side_size_(m)> <num_of_nodes> \ne.g. $0 2000 500\n";

my $tx = $ARGV[0];
my $nodes = $ARGV[1];
my $gws = ceil($tx*$tx/10000);

($tx < 1) && die "grid side must be higher than 1 meters!\n";
($nodes < 1) && die "number of nodes must be higher than 1!\n";

my %coords;
my %ncoords = ();
my %gcoords = ();

for(my $i=1; $i<=$nodes; $i++){
	my ($x, $y) = (int(rand($tx*10)), int(rand($tx*10)));
	($x, $y) = ($x/10, $y/10);
	while (exists $coords{$x}{$y}){
		($x, $y) = (int(rand($tx*10)), int(rand($tx*10)));
		($x, $y) = ($x/10, $y/10);
	}
	$coords{$x}{$y} = 1;
	$ncoords{$i} = [$x, $y];
}
my $gl = 'A';
for(my $i=1; $i<=$gws; $i++){
	my ($x, $y) = (int(rand($tx*10)), int(rand($tx*10)));
	($x, $y) = ($x/10, $y/10);
	while (exists $coords{$x}{$y}){
		($x, $y) = (int(rand($tx*10)), int(rand($tx*10)));
		($x, $y) = ($x/10, $y/10);
	}
	$coords{$x}{$y} = 1;
	$gcoords{$gl} = [$x, $y];
	$gl++;
}

# set cover
my @sensis = ([7,-124,-122,-116], [8,-127,-125,-119], [9,-130,-128,-122], [10,-133,-130,-125], [11,-135,-132,-128], [12,-137,-135,-129]); # sensitivities per SF/BW
my $var = 3.57; # variance
my ($dref, $Lpld0, $gamma) = (40, 110, 2.08); # attenuation model parameters
my $bw = 125000; # channel bandwidth
my %gs = ();
my $G = 0; # assume that variance is 0
my $Xs = $var*$G;
my $bwi = bwconv($bw);

foreach my $g (keys %gcoords){
	my ($gx, $gy) = @{$gcoords{$g}};
	foreach my $s (sort keys %ncoords){
		my ($x, $y) = @{$ncoords{$s}};
		my $d = distance($x, $gx, $y, $gy);
		my $f = 12;
		my $S = $sensis[$f-7][$bwi];
		my $Prx = 14 - ($Lpld0 + 10*$gamma * log10($d/$dref) + $Xs);
		if (($Prx - 2) > $S){ # 10dBm tolerance
			push(@{$gs{$g}}, 1);
		}else{
			push(@{$gs{$g}}, 0);
		}
	}
}

my $cols = scalar keys %ncoords;
my $mode = "greedy";
my $alg = Algorithm::SetCovering->new(columns => $cols, mode => $mode);
my @selected = ();
foreach my $g (sort keys %gs){
	$alg->add_row(@{$gs{$g}});
	push (@selected,$g);
}
my @to_solve = ();
foreach my $s (keys %ncoords){
	push(@to_solve, 1);
}
my @idx_set = $alg->min_row_set(@to_solve);
die "Not enough gateways!\n" unless (scalar @idx_set > 0);

printf "# terrain map [%i x %i]\n", $tx, $tx;
print "# node coords:";
foreach my $s (keys %ncoords){
	my ($x, $y) = @{$ncoords{$s}};
	printf " %s [%.1f %.1f]", $s, $x, $y;
}
print "\n";
print "# gateway coords:";
foreach my $ind (@idx_set){
	my ($x, $y) = @{$gcoords{$selected[$ind]}};
	printf " %s [%.1f %.1f]", $selected[$ind], $x, $y;
	#delete(%gcoords{$selected[$ind]});
	#print " $selected[$ind]";
}
print "\n";
#print "# Inactive gateway coords:";
#foreach my $g (keys %gcoords){
#	my ($x, $y) = @{$gcoords{$g}};
#	printf " %s [%.1f %.1f]", $g, $x, $y;
#}
#print "\n";

print  "# generated with: $0 ",join(" ",@ARGV),"\n";
printf "# stats: nodes=%i gateways=%i terrain=%.1fm^2 node_sz=%.2fm^2\n", scalar keys %ncoords, scalar @idx_set, $tx*$tx, 0.1 * 0.1;

sub distance {
	my ($x1, $x2, $y1, $y2) = @_;
	return sqrt( (($x1-$x2)*($x1-$x2))+(($y1-$y2)*($y1-$y2)) );
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

