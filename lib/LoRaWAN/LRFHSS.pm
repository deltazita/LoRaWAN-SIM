package LoRaWAN::LRFHSS;

use strict;
use warnings;
use List::Util qw(min max);

# Regional profiles and sample-based airtime: 
# see LoRa Alliance RP002-1.0.5, Sections 3.4, 3.5 and 5.3. 
sub profile {
    my ($region, $dr) = @_;
    my %profiles = (
        EU868 => {
            8  => [1, 137000, 35, 8, 50, 11],
            9  => [2, 137000, 35, 8, 115, 10],
            10 => [1, 336000, 86, 8, 50, 11],
            11 => [2, 336000, 86, 8, 115, 10],
        },
        US915 => {
            5 => [1, 1523000, 60, 52, 50, 10],
            6 => [2, 1523000, 60, 52, 125, 9],
        },
    );
    die "Unsupported LR-FHSS region '$region'\n" unless exists $profiles{$region};
    die "Unsupported LR-FHSS data rate DR$dr for $region\n"
        unless defined($dr) && exists $profiles{$region}{$dr};
    my ($num, $bw, $slots, $grids, $max_payload, $rx1sf) = @{$profiles{$region}{$dr}};
    return {
        region => $region, dr => $dr, cr_num => $num, cr_den => 3,
        bandwidth => $bw, slots => $slots, grids => $grids,
        headers => ($num == 1 ? 3 : 2), max_payload => $max_payload,
        rx1sf => $rx1sf, rx1bw => ($region eq 'EU868' ? 125000 : 500000),
    };
}

sub payload_layout {
    my ($profile, $bytes) = @_;
    die "LR-FHSS PHY payload length must be an integer from 1 to 255\n"
        unless defined($bytes) && $bytes =~ /^\d+$/ && $bytes >= 1 && $bytes <= 255;
    # CRC (16 bits), trellis termination (6 bits), then convolutional coding.
    my $bits = (8 * ($bytes + 2) + 6) * 3 / $profile->{cr_num};
    my @layout;
    while ($bits > 0) {
        my $coded = min(48, $bits);
        # Two preamble samples on every payload hop; 488.28125 samples/s.
        push @layout, { duration => ($coded + 2) * 0.002048, coded_bits => $coded };
        $bits -= $coded;
    }
    return \@layout;
}

sub airtime {
    my ($profile, $bytes) = @_;
    my $time = $profile->{headers} * 0.233472;
    $time += $_->{duration} for @{payload_layout($profile, $bytes)};
    return $time;
}

# TODO: check different hop formats
sub hops {
    my ($profile, $bytes, $start, $center, $rng) = @_;
    $rng ||= sub { rand($_[0]) };
    my $grid = int($rng->($profile->{grids}));
    my (@pool, @hops);
    my $last = -1;
    my @parts = (
        (map { { kind => 'header', duration => 0.233472, coded_bits => 0 } }
            1 .. $profile->{headers}),
        (map { { %$_, kind => 'payload' } } @{payload_layout($profile, $bytes)}),
    );
    for my $part (@parts) {
        unless (@pool) {
            @pool = (0 .. $profile->{slots} - 1);
            for (my $i = $#pool; $i > 0; --$i) {
                my $j = int($rng->($i + 1));
                @pool[$i, $j] = @pool[$j, $i];
            }
            @pool[0, -1] = @pool[-1, 0] if $pool[-1] == $last;
        }
        my $slot = pop @pool;
        $last = $slot;
        my $index = $slot * $profile->{grids} + $grid;
        my $offset = ($index - ($profile->{slots} * $profile->{grids} - 1) / 2) * 488.28125;
        push @hops, { %$part, start => $start, end => $start + $part->{duration},
                      frequency => $center + $offset, bandwidth => 488,
                      grid => $grid, slot => $slot };
        $start += $part->{duration};
    }
    return \@hops;
}

# Ideal erasure decoder: one intact header and at least CR of the coded payload bits
sub decode {
    my ($profile, $hops, $lost) = @_;
    my ($headers, $good, $total) = (0, 0, 0);
    for my $i (0 .. $#$hops) {
        my $hop = $hops->[$i];
        if ($hop->{kind} eq 'header') {
            ++$headers unless $lost->{$i};
        } else {
            $total += $hop->{coded_bits};
            $good += $hop->{coded_bits} unless $lost->{$i};
        }
    }
    return $headers > 0 && $good * 3 >= $total * $profile->{cr_num};
}

sub new {
    my ($class, %args) = @_;
    die "LR-FHSS receiver needs a received-power callback\n" unless ref($args{power}) eq 'CODE';
    $args{capture} ||= sub { 6 };
    return bless { %args, active => [], now => -1, stats => {
        headers => 0, fragments => 0, lost_headers => 0, lost_fragments => 0,
        gateway_packets => 0,
    } }, $class;
}

sub next_end {
    my ($self) = @_;
    return undef unless @{$self->{active}};
    return $self->{active}[0]{end};
}

sub receiver_busy {
    my ($self, $receiver, $start, $end) = @_;
    return 0 unless defined($start) && defined($end) && $start < $end;

    for my $frame (@{$self->{active}}) {
        # Downlinks list their destination as the receiver, whereas LR-FHSS
        # uplinks list gateways.  This keeps gateway TX/RX occupancy local to
        # the gateway without treating an unrelated downlink as reception.
        next unless grep { $_ eq $receiver } @{$frame->{receivers}};
        return 1 if max($start, $frame->{start})
                 < min($end, $frame->{end}) - 1e-9;
    }
    return 0;
}

sub start_frame {
    my ($self, $frame) = @_;
    die "Non-chronological radio event\n" if $frame->{start} < $self->{now} - 1e-9;
    $self->{now} = $frame->{start};
    $frame->{lost} = {};
    $frame->{powers} = {};
    for my $receiver (@{$frame->{receivers}}) {
        my $p = $self->{power}->($frame, $receiver);
        $frame->{powers}{$receiver} = $p;
        if ($p < $frame->{sensitivity}) {
            $frame->{lost}{$receiver}{$_} = 1 for 0 .. $#{$frame->{hops}};
        }
    }
    # Only frames that really started enter this model. Future cancelled NbTrans
    # copies cannot cause interference. Pairwise results are saved on both frames.
    for my $other (@{$self->{active}}) {
        $self->_interfere($frame, $other);
    }
    my $i = 0;
    ++$i while $i < @{$self->{active}} && $self->{active}[$i]{end} <= $frame->{end};
    splice @{$self->{active}}, $i, 0, $frame;
}

sub _power {
    my ($self, $frame, $receiver) = @_;
    $frame->{powers}{$receiver} = $self->{power}->($frame, $receiver)
        unless exists $frame->{powers}{$receiver};
    return $frame->{powers}{$receiver};
}

sub _damage {
    my ($self, $victim, $vi, $interferer, $ii, $overlap_bw) = @_;
    my $ih = $interferer->{hops}[$ii];
    for my $receiver (@{$victim->{receivers}}) {
        # A transmitting gateway/node is deaf across the entire RF band.
        if ($receiver eq $interferer->{sender}) {
            $victim->{lost}{$receiver}{$vi} = 1;
            next;
        }
        next unless $overlap_bw > 0;
        my $wanted = $self->_power($victim, $receiver);
        # Rectangular spectra: only the interferer's power inside the overlap.
        my $unwanted = $self->_power($interferer, $receiver)
            + 10 * log($overlap_bw / $ih->{bandwidth}) / log(10);
        my $threshold = $self->{capture}->($victim, $interferer);
        $victim->{lost}{$receiver}{$vi} = 1 if $wanted - $unwanted <= $threshold;
    }
}

sub _interfere {
    my ($self, $a, $b) = @_;
    my ($i, $j) = (0, 0);
    # Both hop lists are chronological; linear sweep includes full containment
    # and excludes touching endpoints (half-open transmission intervals).
    while ($i < @{$a->{hops}} && $j < @{$b->{hops}}) {
        my ($ah, $bh) = ($a->{hops}[$i], $b->{hops}[$j]);
        if (max($ah->{start}, $bh->{start}) < min($ah->{end}, $bh->{end}) - 1e-9) {
            my $overlap = min($ah->{frequency} + $ah->{bandwidth}/2,
                              $bh->{frequency} + $bh->{bandwidth}/2)
                        - max($ah->{frequency} - $ah->{bandwidth}/2,
                              $bh->{frequency} - $bh->{bandwidth}/2);
            $self->_damage($a, $i, $b, $j, $overlap);
            $self->_damage($b, $j, $a, $i, $overlap);
        }
        if ($ah->{end} <= $bh->{end}) { ++$i } else { ++$j }
    }
}

sub finish_frame {
    my ($self) = @_;
    my $frame = shift @{$self->{active}};
    return undef unless $frame;
    $self->{now} = $frame->{end};
    my @received;
    for my $receiver (@{$frame->{receivers}}) {
        my $lost = $frame->{lost}{$receiver} || {};
        my $ok = $frame->{profile}
            ? decode($frame->{profile}, $frame->{hops}, $lost) : !$lost->{0};
        push @received, [$receiver, $frame->{powers}{$receiver}] if $ok;
        if ($frame->{profile}) {
            ++$self->{stats}{gateway_packets};
            for my $i (0 .. $#{$frame->{hops}}) {
                my $key = $frame->{hops}[$i]{kind} eq 'header' ? 'headers' : 'fragments';
                ++$self->{stats}{$key};
                ++$self->{stats}{"lost_$key"} if $lost->{$i};
            }
        }
    }
    $frame->{received} = \@received;
    return $frame;
}

sub stats { return { %{$_[0]{stats}} } }

1;
