#!/usr/bin/perl -w
#
# Script to create a connected 2D terrain of robots
#
# Author: Dimitrios Zorbas (jim/at/students/cs/unipi/gr)
# based on the terrain generator of D. Glynos
# Distributed under the GPLv3 (see LICENSE file)

use Graph;
use Math::Random;
use strict;

# We use decimeter precision (1/10th of a meter). No two robots shall occupy
# the same square decimeter.

my $robot_comm_radius = 50 * 10; 	# in deci-meters
my %nodes = ();
my %hop = ();
my @sensing_nodes = ();
my @relay_nodes = ();
my ($terrain_x, $terrain_y) = (1000 * 10, 1000 * 10); 	# 1km by 1km terrain


sub distance {
	my ($x1, $x2, $y1, $y2) = @_;
	return sqrt( (($x1-$x2)*($x1-$x2)) + (($y1-$y2)*($y1-$y2)) );
}

sub random_int {
	my $low = shift;
	my $high = shift;
	return Math::Random::random_uniform_integer(1, $low, $high);
}

sub gaussian_rand {
	my $mean = shift; # x or y location of the sensor
	my $sdev = 30/2;
	
	my $u1;  # uniformly distributed random number
	my $w;   # variance               
	my $g1;  # gaussian-distributed number
	
	do {
		$u1 = 2 * random_uniform() - 1;
		$w = $u1*$u1;
	} while ( $w >= 1 );
	
	$w = sqrt( (-2 * log($w)) / $w );
	
	$g1 = $u1 * $w * $sdev + $mean;
	
	return $g1;
}

(@ARGV==3) || die "usage: $0 <relay_nodes> <sensing_nodes> <density%>\n";

my $relay_nodes = $ARGV[0];
my $sensing_nodes = $ARGV[1];

if ($sensing_nodes == 0){
	$sensing_nodes = 1;
}

my $density = $ARGV[2]/100;
my $norm_x = int($terrain_x * $density);	# normalised terrain_x
my $norm_y = int($terrain_y * $density); 	# normalised terrain_y

my $base_x = 1;		# x location of base node
my $base_y = int($norm_y / 2);	# y location of base node


### GENERATING SENSORS ###

my %nodes_temp = ();
my $s = 1;
for(my $i=1; $i<=$relay_nodes; $i++){
	my ($x,$y) = (random_int(1, $norm_x), random_int(1, $norm_y));
	
	while (exists $nodes_temp{$x}{$y}){
		($x, $y) = (random_int(1, $norm_x), random_int(1, $norm_y));
	}
	$nodes_temp{$x}{$y} = 1;
	$nodes{$i} = [$x, $y];
	$s++;
}

my $num_points = 5;
my $sensors_per_point = $sensing_nodes/$num_points;
my %points_temp = ();

for(my $i=1; $i<=$num_points; $i++){
	my ($x, $y) = (random_int(1, $norm_x), random_int(1, $norm_y));
	
	while (exists $points_temp{$x}{$y}){
		($x, $y) = (random_int(1, $norm_x), random_int(1, $norm_y));
	}
	$points_temp{$x}{$y} = 1;
	
	for (my $j=1; $j<=$sensors_per_point; $j++){
		my ($nx, $ny) = (gaussian_rand($x), gaussian_rand($y));
		while (($nx < 0) || ($nx > $norm_x) || ($ny < 0) || ($ny > $norm_y)){
			($nx, $ny) = (gaussian_rand($x), gaussian_rand($y));
		}
		$nodes_temp{$nx}{$ny} = 1;
		$nodes{$s} = [$nx, $ny];
		push (@sensing_nodes, $s);
		$s++;
	}
}


### COMPUTE SPT GRAPH ###

my $graph = Graph::Undirected->new;
foreach my $n (keys %nodes){
	my ($x, $y) = @{$nodes{$n}};
	
# 	my $min_dist = 9999999;
# 	my $selected = undef;
# 	foreach my $n_ (keys %nodes){
# 		next if (grep {$_ eq $n_} @sensing_nodes);
# 		next if ($n_ == $n);
# 		my ($x_, $y_) = @{$nodes{$n_}};
# 		my $d = distance($x, $x_, $y, $y_);
# 		if ($d < $min_dist){
# 			$d = $min_dist;
# 			$selected = $n_;
# 		}
# 	}
	
	if (distance($x, $base_x, $y, $base_y) <= $robot_comm_radius){
		$graph->add_weighted_edge($n, "0", distance($x, $base_x, $y, $base_y));
	}
	foreach my $n_ (keys %nodes){
		next if ($n_ eq $n);
		my ($x_, $y_) = @{$nodes{$n_}};
		if (distance($x, $x_, $y, $y_) <= $robot_comm_radius){
			$graph->add_weighted_edge($n, $n_, distance($x, $x_, $y, $y_)) if (!$graph->has_edge($n, $n_));
		}
	}
}
my $sptg = $graph->SPT_Dijkstra("0");


### DELETE UNUSED NODES ###

foreach my $n (keys %nodes){
	if (!$graph->same_connected_components($n, "0")){
		delete $nodes{$n};
		$graph->delete_vertex($n);
	}
	if (!$sptg->has_vertex($n)){
		delete $nodes{$n};
	}
	if (($sptg->degree($n) == 1) && (!grep {$_ eq $n} @sensing_nodes)){
		$sptg->delete_vertex($n);
	}
}
foreach my $n (@sensing_nodes){
	my @path = $sptg->SP_Dijkstra($n, "0");
	$hop{$n} = (scalar @path) - 1;
	foreach my $r (@path){
		next if (($r eq $n) || ($r eq "0"));
		if (!grep {$_ eq $r} @relay_nodes){
			push (@relay_nodes, $r);
		}
	}
}
foreach my $n (@relay_nodes){
	my @path = $sptg->SP_Dijkstra($n, "0");
	$hop{$n} = (scalar @path) - 1;
}
foreach my $n (keys %nodes){
	if ((!grep {$_ eq $n} @relay_nodes) && (!grep {$_ eq $n} @sensing_nodes)){
		$sptg->delete_vertex($n);
		delete $nodes{$n};
	}
}

### PRINT OUTPUT ###

printf "# terrain map [%i x %i]\n", $norm_x, $norm_y;

print "# robot coords:";
$nodes{"0"} = [$base_x, $base_y];
foreach my $n (keys %nodes){
	my ($x, $y) = @{$nodes{$n}};
	printf " %i [%i %i]", $n, $x, $y;
}
print "\n";

print "# cds nodes:";
foreach my $n (@relay_nodes){
	print " $n [$hop{$n}]";
}
print "\n";

print "# sensing nodes:";
foreach my $n (@sensing_nodes){
	print " $n [$hop{$n}]";
}
print "\n";

printf "# base station coords: [%i %i]\n", $base_x, $base_y;
print "# generated with: $0 ",join(" ",@ARGV),"\n";
printf "# stats: robots=%i sensing_robots=%d terrain=%.1fm^2 robot_sz=%.2fm^2 r_c=%.2fm\n", (scalar keys %nodes) - 1, $sensing_nodes, ($norm_x * $norm_y) / 100, 0.1 * 0.1, $robot_comm_radius / 10;
printf "# Graph: %s\n", $sptg;
printf "# %s\n", '$Id: generate_sp-gauss-tree.pl 78 2013-12-13 14:48:27Z jim $';
exit 0;
