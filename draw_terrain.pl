#!/usr/bin/perl -w
#
# Script to create svg file from 2d terrain data
#
# copied from https://github.com/rainbow-src/sensors/tree/master/terrain%20generators

use GD::SVG;
use strict;

my ($display_x, $display_y) = (800, 800); # 800x800 pixel display pane

my ($terrain, $norm_x, $norm_y) = (0, 0, 0);

my @sensors = ();
my @gws = ();

die "usage: $0 <terrain_file.txt> <output.svg>\n" 
	unless (@ARGV == 2);

my $terrain_file = $ARGV[0];
my $output_file = $ARGV[1];


# COLLECT INFO FROM INPUT FILE

open(FH, "<$terrain_file") or 
	die "Error: could not open terrain file $terrain_file\n";

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
		@sensors = map { /([0-9]+) \[([0-9]+\.[0-9]+) ([0-9]+\.[0-9]+)/; [$1, $2, $3]; } @coords;
	} elsif (/^# gateway coords: (.*)/){
		my $sensor_coord = $1;
		my @coords = split(/\] /, $sensor_coord);
		@gws = map { /([0-9]+) \[([0-9]+\.[0-9]+) ([0-9]+\.[0-9]+)/; [$1, $2, $3]; } @coords;
	}
}
close(FH);


### GENERATE SVG IMAGE OF TERRAIN ###

my $im = new GD::SVG::Image($display_x, $display_y);
my $blue = $im->colorAllocate(0,0,255);
my $black = $im->colorAllocate(0,0,0);
my $red = $im->colorAllocate(255,0,0);
	
foreach my $po (@sensors){
	my ($s, $x, $y) = @$po;
	($x, $y) = (int(($x * $display_x)/$norm_x), int(($y * $display_y)/$norm_y));
	$im->rectangle($x-2, $y-2, $x+2, $y+2, $black);
# 	$im->string(gdSmallFont,$x-2,$y-20,$s,$blue); 
}
#$im->filledRectangle(3-3, $display_y/2-3, 3+3, $display_y/2+3, $black);
foreach my $po (@gws){
	my ($s, $x, $y) = @$po;
	($x, $y) = (int(($x * $display_x)/$norm_x), int(($y * $display_y)/$norm_y));
	$im->rectangle($x-5, $y-5, $x+5, $y+5, $red);
	$im->string(gdGiantFont,$x-2,$y-20,$s,$blue);
}


open(FILEOUT, ">$output_file") or 
	die "could not open file $output_file for writing!";
binmode FILEOUT;
print FILEOUT $im->svg;
close FILEOUT;
