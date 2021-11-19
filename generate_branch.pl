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

my %m = ();				# if the value is 1, the key node is cds node
my %nb = ();
my %nodes = ();
my %hop = ();
my @sensing_nodes = ();
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

(@ARGV==2) || die "usage: $0 <num_of_leaves> <dist>\n";

my $num_nodes = $ARGV[0] + 1;
my $robot_comm_radius = $ARGV[1] * 10; 	# in deci-meters

my $density = 0.1;
my $norm_x = int($terrain_x * $density);	# normalised terrain_x
my $norm_y = int($terrain_y * $density); 	# normalised terrain_y

my $base_x = 1;		# x location of base node
my $base_y = int($norm_y / 2);	# y location of base node


### GENERATING SENSORS ###

$nodes{"0"} = [$base_x, $base_y];
$nodes{"1"} = [$base_x+$robot_comm_radius, $base_y];

my %nodes_temp = ();
for(my $i=2; $i<=$num_nodes; $i++){
	# we generate numbers that are parts of the semi-circle with center $node{"1"} coordinates and range $robot_comm_radius
	my ($x,$y) = (random_int($base_x+$robot_comm_radius, $base_x+2*$robot_comm_radius), random_int($base_y-$robot_comm_radius, $base_y+$robot_comm_radius)); 
	my $d = distance($x, $base_x+$robot_comm_radius, $y, $base_y);
	while ((exists $nodes_temp{$x}{$y}) || ($d > $robot_comm_radius) || ($d < ($robot_comm_radius-0.05))){
		($x,$y) = (random_int($base_x+$robot_comm_radius, $base_x+2*$robot_comm_radius), random_int($base_y-$robot_comm_radius, $base_y+$robot_comm_radius)); 
		$d = distance($x, $base_x+$robot_comm_radius, $y, $base_y);
	}
	$nodes_temp{$x}{$y} = 1;
	$nodes{$i} = [$x, $y];
}


### COMPUTE GRAPH ###

my $graph = Graph::Undirected->new;
foreach my $n (keys %nodes){
	next if ($n > 1);
	my ($x, $y) = @{$nodes{$n}};
	foreach my $n_ (keys %nodes){
		next if ($n_ eq $n);
		my ($x_, $y_) = @{$nodes{$n_}};
		if (distance($x, $x_, $y, $y_) <= $robot_comm_radius){
			$graph->add_edge($n, $n_) if (!$graph->has_edge($n, $n_));
		}
	}
}

my $sptg = $graph->SPT_Dijkstra("0");

### PRINT OUTPUT ###

printf "# terrain map [%i x %i]\n", $norm_x, $norm_y;

print "# robot coords:";
foreach my $n (keys %nodes){
	my ($x, $y) = @{$nodes{$n}};
	printf " %i [%i %i]", $n, $x, $y;
}
print "\n";

print "# cds nodes:";
print " 0 [0] 1 [1]";
print "\n";

print "# sensing nodes:";
foreach my $n (keys %nodes){
	next if ($n < 2);
	print " $n [2]";
}
print "\n";

printf "# base station coords: [%i %i]\n", $base_x, $base_y;
print "# generated with: $0 ",join(" ",@ARGV),"\n";
printf "# stats: robots=%i sensing_robots=%d terrain=%.1fm^2 robot_sz=%.2fm^2 r_c=%.2fm\n", (scalar keys %nodes) - 1, $num_nodes - 1, ($norm_x * $norm_y) / 100, 0.1 * 0.1, $robot_comm_radius / 10;
printf "# Graph: %s\n", $sptg;
printf "# %s\n", '$Id: generate_branch.pl 39 2013-06-20 14:47:04Z jim $';
exit 0;
