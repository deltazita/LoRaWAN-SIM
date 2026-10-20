#!/usr/bin/perl -w
# Random gateways and EDs, with nominal coverage for the selected uplink PHY.
# The terrain_* functions are also used by the simulator and greedy terrain generator.
use strict;
use FindBin qw($Bin);
use lib "$Bin/lib";
use Getopt::Long qw(GetOptionsFromArray);
use POSIX qw(isfinite);

sub propagation_parameters {
    my ($config) = @_;
    my $environment = lc($config->{environment} // 'legacy');
    # [reference distance (m), median loss (dB), exponent, shadowing stddev (dB)].
    # Oulu: Petajajarvi et al., ITST 2015, on-ground/car measurements, Table IV.
    # https://cc.oulu.fi/~kmikhayl/site-assets/pdfs/2015_ITST.pdf
    # Bonn: Rademacher et al., 2021, measured urban log-distance fit.
    # https://arxiv.org/abs/2109.07768
    my %profiles = (
        legacy => [40, 110, 2.08, 3.57],
        oulu => [1000, 128.95, 2.32, 7.8],
        bonn => [1000, 132.41, 1.58, 9.9],
    );
    die "environment must be legacy, oulu or bonn\n" unless exists $profiles{$environment};
    my ($dref, $loss, $exponent, $shadowing) = @{$profiles{$environment}};
    return {environment => $environment, dref => $dref, loss_db => $loss, gamma => $exponent, shadowing_db => $shadowing};
}

sub terrain_radio_parameters {
    my ($config) = @_;
    my $region = uc($config->{frequency_plan} // 'EU868');
    die "frequency_plan must be EU868 or US915\n" unless $region eq 'EU868' || $region eq 'US915';
    my $mode = $config->{uplink_modulation} // 'LoRa';
    $mode =~ s/[-_]//g;
    die "uplink_modulation must be LoRa or LRFHSS\n" unless $mode eq 'LoRa' || $mode eq 'LRFHSS';
    my $margin = $config->{coverage_margin_db} // 10; # ChirpStack installation margin (dB).
    die "coverage_margin_db must be finite and nonnegative\n" unless $margin =~ /^(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$/ && isfinite($margin) && $margin >= 0;
    my $power = $region eq 'US915' ? 20 : 14;
    my $sensitivity = $region eq 'US915' ? -135.5 : -141;
    if ($mode eq 'LRFHSS') {
        require LoRaWAN::LRFHSS;
        my $profile = LoRaWAN::LRFHSS::profile($region, $config->{lrfhss_dr} // ($region eq 'EU868' ? 8 : 5));
        $sensitivity = $config->{lrfhss_sensitivity_dbm} // ($profile->{cr_num} == 2 ? -134 : -137);
    } elsif (defined $config->{lrfhss_dr} || defined $config->{lrfhss_sensitivity_dbm}) {
        die "lrfhss_* settings require uplink_modulation LRFHSS\n";
    }
    die "Receiver sensitivity must be finite and numeric\n"
        unless $sensitivity =~ /^-?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$/ && isfinite($sensitivity);
    my $radio = {power_dbm => $power, sensitivity_dbm => 0+$sensitivity,
                 margin_db => 0+$margin, %{propagation_parameters($config)}};
    $radio->{range_m} = $radio->{dref} * 10 **
        (($power-$sensitivity-$radio->{loss_db}-$margin)/(10*$radio->{gamma}));
    die "Selected PHY and margin cannot cover a distinct ED/GW pair at the minimum model distance\n"
        unless isfinite($radio->{range_m}) && $radio->{range_m} > 0.2;
    return $radio;
}

sub terrain_covers {
    my ($radio, $node, $gateway) = @_;
    my $distance_sq = ($node->[0]-$gateway->[0])**2 + ($node->[1]-$gateway->[1])**2;
    return $distance_sq < $radio->{range_m}**2;
}

sub terrain_arguments {
    my ($args, $required) = @_;
    my @positional;
    push @positional, shift @$args while @$args && $args->[0] !~ /^--/;
    die "usage: $0 <side_m> <nodes> " . ($required == 3 ? '<gateways> ' : '') .
        "[seed] [--frequency-plan EU868|US915] [--fhss] [--environment legacy|oulu|bonn] " .
        "[--lrfhss-dr N] [--lrfhss-sensitivity-dbm N] [--coverage-margin-db N]\n"
        unless @positional == $required || @positional == $required+1;
    my %options;
    GetOptionsFromArray($args, 'frequency-plan=s' => \$options{frequency_plan},
        'environment=s' => \$options{environment},
        'fhss' => sub { $options{uplink_modulation} = 'LRFHSS'; }, 'lrfhss-dr=i' => \$options{lrfhss_dr},
        'lrfhss-sensitivity-dbm=f' => \$options{lrfhss_sensitivity_dbm},
        'coverage-margin-db=f' => \$options{coverage_margin_db}) or die "Invalid terrain options\n";
    die "Unexpected terrain arguments: @$args\n" if @$args;
    for my $i (0 .. $required-1) {
        die "Terrain sizes and counts must be positive integers\n" unless $positional[$i] =~ /^\d+$/ && $positional[$i] > 0;
    }
    if (@positional > $required) {
        die "seed must be an integer\n" unless $positional[-1] =~ /^-?\d+$/;
        srand($positional[-1]);
    }
    return (\@positional, terrain_radio_parameters(\%options));
}

sub terrain_position {
    my ($side, $used) = @_;
    for (1 .. 10000) {
        my ($x, $y) = (int(rand($side*10))/10, int(rand($side*10))/10);
        return [$x, $y] unless $used->{"$x,$y"};
    }
    die "Cannot place another distinct coordinate; terrain is too crowded\n";
}

sub terrain_print {
    my ($side, $nodes, $gateways, $radio, $command) = @_;
    printf "# max range = %.12g m\n", $radio->{range_m};
    printf "# coverage: tx=%.1fdBm sensitivity=%.1fdBm margin=%.1fdB (nominal, before random fading)\n",
        @{$radio}{qw(power_dbm sensitivity_dbm margin_db)};
    printf "# propagation: environment=%s dref=%.1fm loss=%.2fdB exponent=%.2f shadowing_stddev=%.2fdB\n",
        @{$radio}{qw(environment dref loss_db gamma shadowing_db)} if $radio->{environment} ne 'legacy';
    printf "# terrain map [%i x %i]\n", $side, $side;
    print '# node coords:';
    printf " %s [%.1f %.1f]", $_->[0], $_->[1], $_->[2] for @$nodes;
    print "\n# gateway coords:";
    printf " %s [%.1f %.1f]", $_->[0], $_->[1], $_->[2] for @$gateways;
    print "\n# generated with: $command\n";
    printf "# stats: nodes=%i gateways=%i terrain=%.1fm^2 node_sz=0.01m^2\n", scalar @$nodes, scalar @$gateways, $side*$side;
}

sub generate_terrain_main {
    my (@args) = @_;
    my $command = join(' ', $0, @args);
    my ($positional, $radio) = terrain_arguments(\@args, 3);
    my ($side, $node_count, $gw_count) = @$positional;
    die "More devices than distinct coordinates in the terrain\n" if $node_count+$gw_count > ($side*10)**2;
    my (%used, @gateways, @nodes);
    my $label = 'A';
    for (1 .. $gw_count) {
        my $point = terrain_position($side, \%used);
        $used{join(',', @$point)} = 1;
        push @gateways, [$label++, @$point];
    }
    for my $id (1 .. $node_count) {
        my $point;
        for (1 .. 10000) {
            my $candidate = terrain_position($side, \%used);
            for my $gateway (@gateways) {
                if (terrain_covers($radio, $candidate, [@$gateway[1,2]])) { $point = $candidate; last; }
            }
            last if $point;
        }
        die "Cannot place covered ED $id; reduce terrain size or add gateways\n" unless $point;
        $used{join(',', @$point)} = 1;
        push @nodes, [$id, @$point];
    }
    terrain_print($side, \@nodes, \@gateways, $radio, $command);
    return 0;
}

exit generate_terrain_main(@ARGV) unless caller;
1;
