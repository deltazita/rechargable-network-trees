#!/usr/bin/perl -w
#
# Script to create a 2D grid of sensors

use strict;

(@ARGV==2) || die "usage: $0 <grid_side_size> <grid_tics>\ne.g. $0 100 10\n";

my $tx = $ARGV[0]*10;
my $d = $ARGV[1]*10;

($tx < 1) && die "grid side must be higher than 1 meters!\n";
($d < 0.5) && die "grid tics must be higher than 0.5 meters!\n";

my @sensors;
my $base_x = $d;			# x location of base node
my $base_y = int($tx / 2);	# y location of base node
my $robot_comm_radius = 5*$d;

my $r = 1;
push (@sensors, ["0", $base_x, $base_y]);
for (my $i=$d; $i<=$tx; $i+=$d){
	for (my $j=$d; $j<=$tx; $j+=$d){
		push(@sensors, [$r, $i, $j]);
		$r++;
	}
}

printf "# terrain map [%i x %i]\n", $tx, $tx;
print "# robot coords:";
foreach my $s (@sensors){
	my ($n, $x, $y) = @$s;
	printf " %s [%i %i]", $n, $x, $y;
}
print "\n";

print "# sensing nodes:";
foreach my $s (@sensors){
	my ($n, $x, $y) = @$s;
	next if ($n eq "0");
	print " $n [0]";
}
print "\n";

printf "# base station coords: [%i %i]\n", $base_x, $base_y;
print  "# generated with: $0 ",join(" ",@ARGV),"\n";
printf "# stats: robots=%i sensing_robots=%d terrain=%.1fm^2 tics=%.2fm robot_sz=%.2fm^2 r_c=%.2fm\n", scalar @sensors, scalar @sensors, $tx*$tx/100, $d/10, 0.1 * 0.1, $robot_comm_radius/10;
printf "# %s\n", '$Id: generate_grid.pl ??? jim $';
