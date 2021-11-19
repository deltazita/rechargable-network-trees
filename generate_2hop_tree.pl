#!/usr/bin/perl -w
#
# Script to create a connected 2D terrain of robots
#
# Author: Dimitrios Zorbas (jim/at/students/cs/unipi/gr)
# based on the terrain generator of D. Glynos
# Distributed under the GPLv3 (see LICENSE file)

use Graph;
use Math::Random;
use POSIX;
use strict;

# We use decimeter precision (1/10th of a meter). No two robots shall occupy
# the same square decimeter.

my $robot_comm_radius = 50 * 10; 	# in deci-meters
my %nodes = ();
my %hop = ();
my @sensing_nodes = ();
my @relay_nodes = ();
my ($terrain_x, $terrain_y) = (100 * 10, 100 * 10); 	# 100m by 100m terrain


sub distance {
	my ($x1, $x2, $y1, $y2) = @_;
	return sqrt( (($x1-$x2)*($x1-$x2)) + (($y1-$y2)*($y1-$y2)) );
}

sub random_int {
	my $low = shift;
	my $high = shift;
	return Math::Random::random_uniform_integer(1, $low, $high);
}

(@ARGV==2) || die "usage: $0 <num_of_relays> <num_of_sensing_nodes>\n";

my $relay_nodes = $ARGV[0];
my $sensing_nodes = $ARGV[1];

my $base_x = 1;		# x location of base node
my $base_y = int($terrain_y / 2);	# y location of base node
my $graph = Graph::Undirected->new;

### GENERATING RELAYS & SENSORS ###

my %nodes_temp = ();
for(my $i=1; $i<=$relay_nodes; $i++){
	my ($x,$y) = (random_int(1, $terrain_x), random_int(1, $terrain_y));
	my $d = distance($x, $base_x, $y, $base_y);
	
	while ((exists $nodes_temp{$x}{$y}) || ($d > $robot_comm_radius)){
		($x, $y) = (random_int(1, $terrain_x), random_int(1, $terrain_y));
		$d = distance($x, $base_x, $y, $base_y);
	}
	$nodes_temp{$x}{$y} = 1;
	$nodes{$i} = [$x, $y];
	push (@relay_nodes, $i);
	$graph->add_weighted_edge($i, "0", distance($x, $base_x, $y, $base_y));
}

my $n = $relay_nodes + 1;
my $k = floor($sensing_nodes/$relay_nodes);
my $l = $k + 1;
foreach my $r (@relay_nodes){
	my $rem = ($sensing_nodes - scalar @sensing_nodes);
	my $check = floor($rem/$l) - $rem/$l;
	if ($check == 0){
		$k = $l;
	}
	print "$k $l\n";

	for (my $i=1; $i<=$k; $i++){
		my $conn_check = 0;
		my ($x, $y) = (random_int(1, $terrain_x), random_int(1, $terrain_y));
		my ($xr, $yr) = ($nodes{$r}[0], $nodes{$r}[1]);
		my $d = distance($x, $xr, $y, $yr);
		my $db = distance($x, $base_x, $y, $base_y);
		if (($d <= $robot_comm_radius) && ($db > $robot_comm_radius)){
			$conn_check = 1;
		}

		while ((exists $nodes_temp{$x}{$y}) || ($conn_check == 0)){
			($x, $y) = (random_int(1, $terrain_x), random_int(1, $terrain_y));
			$d = distance($x, $xr, $y, $yr);
			$db = distance($x, $base_x, $y, $base_y);
			if (($d <= $robot_comm_radius) && ($db > $robot_comm_radius)){
				$conn_check = 1;
			}
		}
		$nodes_temp{$x}{$y} = 1;
		$nodes{$n} = [$x, $y];
		push (@sensing_nodes, $n);
		$graph->add_weighted_edge($n, $r, distance($x, $xr, $y, $yr));
		$n++;
	}
}

### COMPUTE SPT GRAPH AND HOPS ###

my $sptg = $graph->SPT_Dijkstra("0");

foreach my $n (@sensing_nodes){
	my @path = $sptg->SP_Dijkstra($n, "0");
	$hop{$n} = (scalar @path) - 1;
}
foreach my $n (@relay_nodes){
	my @path = $sptg->SP_Dijkstra($n, "0");
	$hop{$n} = (scalar @path) - 1;
}

### PRINT OUTPUT ###

printf "# terrain map [%i x %i]\n", $terrain_x, $terrain_y;

print "# robot coords:";
$nodes{"0"} = [$base_x, $base_y];
foreach my $n (keys %nodes){
	my ($x, $y) = @{$nodes{$n}};
	printf " %i [%i %i]", $n, $x, $y;
}
print "\n";

print "# relay nodes:";
foreach my $n (@relay_nodes){
	next if (!$sptg->has_vertex($n));
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
printf "# stats: relay_robots=%i sensing_robots=%d terrain=%.1fm^2 robot_sz=%.2fm^2 r_c=%.2fm\n", $relay_nodes, $sensing_nodes, ($terrain_x * $terrain_y) / 100, 0.1 * 0.1, $robot_comm_radius / 10;
printf "# Graph: %s\n", $sptg;
printf "# %s\n", '$Id: generate_2hop_tree.pl 74 2013-12-05 18:05:14Z jim $';
exit 0;
