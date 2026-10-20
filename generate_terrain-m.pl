#!/usr/bin/perl -w
# Greedy selection from random gateway candidates, covering every ED.
# It may take some time...
use strict;
use FindBin qw($Bin);
use POSIX qw(ceil);
require "$Bin/generate_terrain.pl";

my $command = join(' ', $0, @ARGV);
my ($positional, $radio) = terrain_arguments(\@ARGV, 2);
my ($side, $node_count) = @$positional;
my $candidate_count = ceil($side*$side/10000);
die "More devices than distinct coordinates in the terrain\n" if $node_count+$candidate_count > ($side*10)**2;
my (%used, @nodes, @gateways, @covers, %covered);
for my $id (1 .. $node_count) {
    my $point = terrain_position($side, \%used);
    $used{join(',', @$point)} = 1;
    push @nodes, [$id, @$point];
}

sub add_candidate {
    my ($point) = @_;
    my @row;
    for my $node (@nodes) {
        if (terrain_covers($radio, [@$node[1,2]], $point)) {
            push @row, $node->[0];
            $covered{$node->[0]} = 1;
        }
    }
    push @gateways, $point;
    push @covers, \@row;
    $used{join(',', @$point)} = 1;
}

add_candidate(terrain_position($side, \%used)) for 1 .. $candidate_count;
# Supplement random candidates near uncovered EDs before running set cover.
for my $node (@nodes) {
    next if $covered{$node->[0]};
    my $radius = $radio->{range_m}/2;
    $radius = $side if $radius > $side;
    my $point;
    for (1 .. 10000) {
        my ($x, $y) = ($node->[1]+(2*rand()-1)*$radius, $node->[2]+(2*rand()-1)*$radius);
        next if $x < 0 || $y < 0 || $x >= $side || $y >= $side;
        ($x, $y) = (int($x*10)/10, int($y*10)/10);
        next if $used{"$x,$y"} || !terrain_covers($radio, [@$node[1,2]], [$x,$y]);
        $point = [$x,$y]; last;
    }
    die "Cannot place a covering gateway for ED $node->[0]\n" unless $point;
    add_candidate($point);
}

my %remaining = map { $_->[0] => 1 } @nodes;
my (@selected, %selected);
while (keys %remaining) {
    my ($best, $best_count) = (undef, 0);
    for my $index (0 .. $#gateways) {
        next if $selected{$index};
        my $count = 0;
        for my $id (@{$covers[$index]}) { $count++ if $remaining{$id}; }
        if ($count > $best_count) { ($best, $best_count) = ($index, $count); }
    }
    die "Uncovered EDs: " . join(', ', sort {$a <=> $b} keys %remaining) . "\n" unless defined $best;
    $selected{$best} = 1;
    push @selected, $best;
    delete @remaining{@{$covers[$best]}};
}
my $label = 'A';
my @output_gateways = map { [$label++, @{$gateways[$_]}] } @selected;
for my $node (@nodes) {
    die "Internal coverage error for ED $node->[0]\n"
        unless grep { terrain_covers($radio, [@$node[1,2]], [@$_[1,2]]) } @output_gateways;
}
terrain_print($side, \@nodes, \@output_gateways, $radio, $command);

