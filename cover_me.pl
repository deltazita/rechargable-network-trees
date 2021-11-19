#!/usr/bin/perl -w
#
# Localised algorithm for replacing and recharging mobile robots
#
# by Dimitrios Zorbas (dimitrios.zormpas(at)inria.fr)
#
# Distributed under the GPLv3 (see LICENSE file)

use strict;
use GD;
use Graph;
use POSIX qw/floor/;

die "$0 <init_energy> <terrain_file>\n" unless(@ARGV == 2);

my $t = 0;
my $hybrid = 1; # rerouting is used when no replacement is found
my $sensors = 1;
my $score = 0;
my $generate_figures = 0; # 1 = generate images
my @robots = ();
my @sensing_robots = ();
my @cds_nodes = ();
my $terrain = 0;
my %my_location = ();
my %init_location = ();
my $g;
my ($comm_radius, $base_x, $base_y, $norm_x, $norm_y, $one_pt_sz) = (0, 0, 0, 0, 0, 1);
my $speed = 0.9;
my $init_energy = shift @ARGV;
my $sensing_ener = 2.5;
my $tran_ener = 5;
my $rec_ener = 2.5;
my $step_ener = 25;
my $idle_ener = 8;
my $recharge_energy = 0; # choose a value >0 if recharging is applied e.g 27
my %lifetime = ();
my %consumption = ();
my %is_recharging = ();
my %threshold = ();
my %critical_threshold = ();
my %is_relay = ();
my %is_sensing = ();
my %destination = ();
my %is_moving = ();
my %future_location = ();
my %parent = ();
my %will_be_replaced = ();
my %will_replace = ();
my %is_standby = ();
my %standby_time = ();
my $theor_max = 0;
my $messages = 0;

#============================================================================================================================#

read_data();

while ($sensors > 0){
	$t++;
	print "############# time $t #############\n";
	foreach my $r (keys %my_location){
		next if ($r eq "0");
		if ($is_moving{$r} == 1){
			my ($x, $y) = (0, 0);
			if (distance($my_location{$r}[0], $destination{$r}[0], $my_location{$r}[1], $destination{$r}[1]) <= $speed){
				$x = $destination{$r}[0];
				$y = $destination{$r}[1];
				$lifetime{$r} -= distance($my_location{$r}[0], $destination{$r}[0], $my_location{$r}[1], $destination{$r}[1]) * $step_ener;
				# has reached the base station
				if (($destination{$r}[0] == $base_x) && ($destination{$r}[1] == $base_y)){
					$is_recharging{$r} = 1;
					$is_moving{$r} = 0;
					delete $destination{$r};
				# has reached the relay node
				}else{
					if ($will_replace{$r} > 0){ # if it will become relay robot
						my $old = $will_replace{$r};
						my @N = $g->neighbours($old);
						$g->delete_vertex($old);
						$messages += 1;
						foreach my $n (@N){
							next if ($n == $r);
							$messages += 1;
							$g->add_edge($n, $r);
							next if ($n eq "0");
							if ($parent{$n} == $old){
								$parent{$n} = $r;
							}
						}
						$parent{$r} = $parent{$old};
						$g->SPT_Dijkstra_clear_cache;
						$g = $g->SPT_Dijkstra("0");
						$is_relay{$r} = 1;
						$is_sensing{$r} = 0;
						$is_moving{$r} = 0;
						$will_be_replaced{$r} = -2;
						delete $destination{$r};
						$will_replace{$r} = -1;
						$is_relay{$old} = 0;
						$is_sensing{$old} = 1;
						$is_moving{$old} = 1;
						$destination{$old} = [$base_x, $base_y];
						$init_location{$old} = $my_location{$r};
						$will_be_replaced{$old} = -1;
						$messages += 1;
					}else{ # if it will become sensing robot (return after recharging)
						print "# not implemented yet\n";
					}
				}
			}else{
				($x, $y) = compute_position($destination{$r}[0], $destination{$r}[1], $my_location{$r}[0], $my_location{$r}[1]);
				$lifetime{$r} -= $step_ener;
			}
			#print "# $r moved from [$my_location{$r}[0] $my_location{$r}[1]] to [$x $y]\n";
			$my_location{$r} = [$x, $y];
		}else{
			update_energy($r);
			if ($is_recharging{$r} == 0){
				compute_threshold($r);
				
				if ($will_be_replaced{$r} == -2){ # if a will_be_replaced node has not been selected yet
					select_replacer($r);
				}elsif ($will_be_replaced{$r} > 0){ # periodically check if will_be_replaced exists (just to count the sent messages)
					$messages += 2 if ($t%10 == 0);
				}
				
				cover_me($r);
				check_isolation($r);
				reroute($r) if ($hybrid == 1);
			}
		}
	}
	print "# $g\n";
	draw_terrain($t) if ($generate_figures == 1);

	$sensors = 0;
	foreach my $r (keys %is_relay){
		next if (($is_sensing{$r} == 0) || ($is_recharging{$r} == 1) || ($is_moving{$r} == 1) || ($is_standby{$r} == 1));
		$sensors += 1;
	}
	$score += $sensors/(scalar @sensing_robots);
}

# statistics here
printf "# Normalised network lifetime: %.2f\n", $score;
printf "# Network lifetime: %d\n", $t-1;
print "# Theoretical maximum: $theor_max\n";
print "# Number of messages: $messages\n";
printf "# %s\n", '$Id: cover_me.pl 101 2014-05-02 19:39:00Z jim $';


sub cover_me{
	my $r = shift;
	my ($x, $y) = @{$my_location{$r}};
	if (($lifetime{$r} - $consumption{$r}) < $threshold{$r}){
		if ($will_be_replaced{$r} != -1){
			$is_moving{$will_be_replaced{$r}} = 1;
			$destination{$will_be_replaced{$r}} = [$x, $y];
			$init_location{$will_be_replaced{$r}} = $my_location{$will_be_replaced{$r}};
			my @N = $g->neighbours($will_be_replaced{$r});
			$messages += 1;
			foreach my $n (@N){
				next if ($n eq "0");
				if ($parent{$n} == $will_be_replaced{$r}){
					$parent{$n} = -1;
				}
			}
			$g->delete_vertex($will_be_replaced{$r});
			$g = $g->SPT_Dijkstra("0");
			$is_recharging{$will_be_replaced{$r}} = 0;
			print "# $will_be_replaced{$r} is moving to replace $r\n";
			$messages += 2;
		}else{
			$is_moving{$r} = 1;
			$destination{$r} = [$base_x, $base_y];
			$init_location{$r} = $my_location{$r};
			my @N = $g->neighbours($r);
			$messages += 1;
			foreach my $n (@N){
				next if ($n eq "0");
				if ($parent{$n} == $r){
					$parent{$n} = -1;
				}
			}
			$g->delete_vertex($r);
			$g = $g->SPT_Dijkstra("0");
			print "# $r is going to recharge (no replacer)\n";
		}
	}
}

sub compute_threshold{
	my $r = shift;
	my ($x, $y) = @{$my_location{$r}};
	if ($will_be_replaced{$r} > 0){ # if a will_be_replaced node has already been selected
		$critical_threshold{$r} = (distance($x, $base_x, $y, $base_y)/$speed) * $step_ener;
		$threshold{$r} = (distance($x, $my_location{$will_be_replaced{$r}}[0], $y, $my_location{$will_be_replaced{$r}}[1])/$speed) * $consumption{$r} + (distance($x, $base_x, $y, $base_y)/$speed) * $step_ener;
	}else{
		$critical_threshold{$r} = (distance($x, $base_x, $y, $base_y)/$speed) * $step_ener;
		$threshold{$r} = (distance($x, $base_x, $y, $base_y)/$speed) * $step_ener;
	}
}

sub update_energy{
	my $r = shift;
	$consumption{$r} = 0;
	if ($is_recharging{$r} == 1){
		$lifetime{$r} += $recharge_energy;
		if ($lifetime{$r} >= $init_energy){
			$is_recharging{$r} = 0;
			$lifetime{$r} = $init_energy;
			$is_moving{$r} = 1;
			$destination{$r} = $init_location{$r};
		}
	}else{
		if (exists $critical_threshold{$r}){
			if (($lifetime{$r} - $consumption{$r}) < $critical_threshold{$r}){
				$is_moving{$r} = 1;
				$destination{$r} = [$base_x, $base_y];
				$init_location{$r} = $my_location{$r};
				my @N = $g->neighbours($r);
				$messages += 1;
				foreach my $n (@N){
					next if ($n eq "0");
					if ($parent{$n} == $r){
						$parent{$n} = -1;
					}
				}
				$g->delete_vertex($r);
				$g = $g->SPT_Dijkstra("0");
				print "# $r is going to recharge (no replacer)\n";
			}
		}
		my $u = $g->get_vertex_attribute($r, 'p');
		my $leaves = 0;
		if ($is_sensing{$r} == 1){
			$consumption{$r} = $sensing_ener + $tran_ener;
		}
		my $g_tmp = $g->copy_graph;
		$g_tmp->delete_vertex($u);
		my @reachable = $g_tmp->all_neighbours($r);
		foreach my $r_ (@reachable){
			if ($is_sensing{$r_} == 1){
				$leaves += 1;
			}
		}
		$consumption{$r} += $rec_ener * $leaves  +  $tran_ener * $leaves + $idle_ener;
		$lifetime{$r} -= $consumption{$r};
		print "# $r spent $consumption{$r}J ($leaves)\n";
		if (($is_relay{$r} == 1) && ($leaves == 0) && ($is_sensing{$r} == 0)){
			is_isolated($r);
		}
	}
}

sub select_replacer{
	my $r = shift;
	my $selected = undef;
	my $selected_r = undef;
	my $selected_s = undef;
	my $min_d = $comm_radius + 1;
	my $max_e = 0;
	my $min_cons = $consumption{$r};
	my $selected_rel = undef;
	$messages += 1;
	my $someone_is_recharging = 0;
	foreach my $r_ (keys %my_location){
		next if (($r_ == $r) || ($is_moving{$r_} == 1) || ($r_ eq "0"));
 		next if (($is_relay{$r_} == 1) && ($is_recharging{$r_} == 0));
		if ($will_replace{$r_} > 0){
			$messages += 1;
			next;
		}
		my $d = distance($my_location{$r}[0], $my_location{$r_}[0], $my_location{$r}[1], $my_location{$r_}[1]);
# 		if (($is_relay{$r_} == 1) && ($is_recharging{$r_} == 0)){
# 			if (($d <= $comm_radius) && ($consumption{$r_} < $min_cons) && (($lifetime{$r_}-$d*$step_ener) > $critical_threshold{$r})){
# 				$min_cons = $consumption{$r_};
# 				$selected_rel = $r_;
# 				$messages += 1;
# 			}
# 		}else{
			if ($is_recharging{$r_} == 1){
				if (($d <= $comm_radius) && ($lifetime{$r_} > $max_e) && (($lifetime{$r_}-$d*$step_ener) > $critical_threshold{$r})){
					$messages += 1;
					$selected_r = $r_;
					$max_e = $lifetime{$r_};
				}
			}else{
				if (($d <= $comm_radius) && ($d < $min_d) && (($lifetime{$r_}-$d*$step_ener) > $critical_threshold{$r})){
					$messages += 1;
					$selected_s = $r_;
					$min_d = $d;
				}
			}
# 		}
	}
	if (defined $selected_r){ # prefer selecting recharging nodes
		$selected = $selected_r;
	}elsif ((!defined $selected_r) && (defined $selected_s)){
		$selected = $selected_s;
	}elsif ((!defined $selected_r) && (!defined $selected_s) && (defined $selected_rel)){
		$selected = $selected_rel;
	}
	
	if (defined $selected){
		$will_be_replaced{$r} = $selected;
		$will_replace{$selected} = $r;
		print "# $r selected $selected as replacer\n";
		$messages += 2;
	}else{
		$will_be_replaced{$r} = -1;
		print "# $r did not find any replacer\n";
	}
}

sub check_isolation{
	my $r = shift;
	if ($parent{$r} == -1){
		if ($hybrid == 1){
			if ($is_sensing{$r} == 1){
				# find another route in a while
				if ($is_standby{$r} == 0){
					$is_standby{$r} = 1;
					$standby_time{$r} = $t;
				}
			}else{
				is_isolated($r);
			}
		}else{
			is_isolated($r);
		}
	}
}

sub is_isolated {
	my $r = shift;
	$is_moving{$r} = 1;
	$is_standby{$r} = 0;
	$destination{$r} = [$base_x, $base_y];
	$init_location{$r} = $my_location{$r};
	my @N = $g->neighbours($r);
	$messages += 1;
	foreach my $n (@N){
		next if ($n eq "0");
		if ($parent{$n} == $r){
			$parent{$n} = -1;
		}
	}
	if ($will_replace{$r} > 0){
		$will_be_replaced{$will_replace{$r}} = -2;
	}
	$g->delete_vertex($r);
	$g = $g->SPT_Dijkstra("0");
	print "# $r is going to recharge (isolated)\n";
}

sub reroute {
	my $r = shift;
	if ($is_standby{$r} == 1){
		if (($t - $standby_time{$r}) > 5){
			my ($x, $y) = @{$my_location{$r}};
			my $min_dist = 999999;
			my $selected = undef;
			$messages += 1;
			foreach my $n (keys %is_relay){
				next if ($is_standby{$n} == 1);
				next if ($is_moving{$n} == 1);
				next if ($is_recharging{$n} == 1);
				
				my ($nx, $ny) = @{$my_location{$n}};
				my $d = distance($x, $nx, $y, $ny);
				my $dn = distance($base_x, $nx, $base_y, $ny);
				if ($d <= $comm_radius){
					$messages += 1;
				}
				if (($dn < $min_dist) && ($d <= $comm_radius)){
					$selected = $n;
					$min_dist = $dn;
				}
			}
			if (defined $selected){
				$g->delete_vertex($r);
				$g->add_edge($r, $selected);
				$g = $g->SPT_Dijkstra("0");
				$is_relay{$selected} = 1;
				$is_standby{$r} = 0;
				$parent{$r} = $selected;
				$messages += 1;
				print "# $r connected to $selected\n";
			}else{
				is_isolated($r);
			}
		}
	}
}

sub read_data{
	my $temp_graph;
	while(<>){
		chomp;
		if (/^# stats: (.*)/){
			my $stats_line = $1;

			if ($stats_line =~ /robot_sz=([0-9]+\.[0-9]+)m\^2/){
				$one_pt_sz = sqrt($1); 
			}
			if ($stats_line =~ /r_c=([0-9]+\.[0-9]+)m/){
				$comm_radius = $1;
			}
		} elsif (/^# base station coords: \[([0-9]+) ([0-9]+)\]/){
			($base_x, $base_y) = ($1, $2);	
		} elsif (/^# terrain map \[([0-9]+) x ([0-9]+)\]/){
			($norm_x, $norm_y) = ($1, $2);
		} elsif (/^# robot coords: (.*)/){
			my $sens_coord = $1;
			my @coords = split(/\] /, $sens_coord);
			@robots = map { /([0-9]+) \[([0-9]+) ([0-9]+)/; [$1, $2, $3]; } @coords;
		} elsif ((/^# relay nodes: (.*)/) || (/^# cds nodes: (.*)/)){
			my $elem = $1;
			my @no = split(/\] /, $elem);
			@cds_nodes = map { /([0-9]+)/; $1; } @no;
		} elsif (/^# sensing nodes: (.*)/){
			my $elem = $1;
			my @no = split(/\] /, $elem);
			@sensing_robots = map { /([0-9]+)/; $1; } @no;
		} elsif (/^# Graph: (.*)/){
			$temp_graph = $1;
		}
	}
	
	$g = Graph::Undirected->new;
	my @edges = split(/,/, $temp_graph);
	foreach my $edge (@edges){
		chomp ($edge);
		my ($v1, $v2) = split(/=/,$edge);
		$g->add_edge($v1, $v2);
	}
	$g = $g->SPT_Dijkstra("0");
	
	foreach my $rob (@robots){
		my ($r, $x, $y) = @$rob;
		if ($r eq "0"){
			$my_location{$r} = [$x, $y];
			$is_relay{$r} = 1;
			$is_sensing{$r} = 0;
			$is_recharging{$r} = 0;
			$is_moving{$r} = 0;
		}else{
			$my_location{$r} = [$x, $y];
			$lifetime{$r} = $init_energy;
			$is_sensing{$r} = 0;
			$is_relay{$r} = 0;
			$will_be_replaced{$r} = -1; # no will_be_replaced node selected yet
			if (grep {$_ == $r} @cds_nodes){
				$is_relay{$r} = 1;
				$will_be_replaced{$r} = -2; # a will_be_replaced node must be selected
			}
			if (grep {$_ == $r} @sensing_robots){
				$is_sensing{$r} = 1;
			}
			$is_recharging{$r} = 0;
			$is_moving{$r} = 0;
			my $u = $g->get_vertex_attribute($r, 'p');
			$parent{$r} = $u;
			$threshold{$r} = 0;
			$consumption{$r} = 0;
			$will_replace{$r} = -1;
			$is_standby{$r} = 0;
		}
	}
	my $min_dist = 99999999;
	foreach my $r (@sensing_robots){
		my $d = distance($my_location{$r}[0], $base_x, $my_location{$r}[1], $base_y);
		if ($d < $min_dist){
			$min_dist = $d;
		}
	}
	$theor_max = floor($init_energy/($sensing_ener+$tran_ener+$idle_ener) - $min_dist/$speed);
}

sub draw_terrain {
	my $t = shift;
	my ($display_x, $display_y) = (800, 800);
	my $im = new GD::Image($display_x, $display_y);
	my $white = $im->colorAllocate(255,255,255);
	my $blue = $im->colorAllocate(0,0,255);
	my $green = $im->colorAllocate(200,255,200);
	my $black = $im->colorAllocate(0,0,0);
	my $red = $im->colorAllocate(255,0,0);
	my $gray = $im->colorAllocate(100,100,100);
	$im->transparent($white);
	$im->interlaced('true');
	
	foreach my $s (keys %my_location){
		my ($x, $y) = @{$my_location{$s}};
		($x, $y) = (int(($x * $display_x)/ $norm_x), int(($y * $display_y)/ $norm_y));
		next if ($s eq "0");
		if ($is_moving{$s} == 1){
			$im->string(gdLargeFont,$x-4,$y-20,$s,$black);
			$im->string(gdLargeFont,$x-10,$y+5,$lifetime{$s},$black);
			$im->filledRectangle($x-4, $y-4, $x+4, $y+4, $black);
			$im->arc($x-3, $y-5, 5, 5, 0, 360, $black);
			$im->arc($x+3, $y-5, 5, 5, 0, 360, $black);
			$im->arc($x-3, $y+5, 5, 5, 0, 360, $black);
			$im->arc($x+3, $y+5, 5, 5, 0, 360, $black);
			#$im->arc($x, $y, 8, 8, 0, 360, $red);
		}else{
			if ($is_relay{$s} == 1){
				next if ($is_moving{$s} == 1);
				if ($s eq "0"){
					$im->string(gdLargeFont,$x+4,$y-22,"BS",$red);
				}else{
					$im->string(gdLargeFont,$x-4,$y-22,$s,$red);
					$im->string(gdMediumBoldFont,$x-2,$y-33,$will_be_replaced{$s},$red);
					$im->string(gdLargeFont,$x-10,$y+5,$lifetime{$s},$red);
					$im->string(gdLargeFont,$x-10,$y+18,int($threshold{$s}),$red);
				}
				$im->filledRectangle($x-4, $y-4, $x+4, $y+4, $red);
				$im->arc($x-3, $y-5, 5, 5, 0, 360, $red);
				$im->arc($x+3, $y-5, 5, 5, 0, 360, $red);
				$im->arc($x-3, $y+5, 5, 5, 0, 360, $red);
				$im->arc($x+3, $y+5, 5, 5, 0, 360, $red);
				#$im->arc($x, $y, 12, 12, 0, 360, $red);
			}else{
				$im->string(gdLargeFont,$x-4,$y-20,$s,$blue);
				$im->string(gdLargeFont,$x-10,$y+5,$lifetime{$s},$blue);
				$im->string(gdLargeFont,$x-10,$y+18,int($threshold{$s}),$blue);
				$im->filledRectangle($x-4, $y-4, $x+4, $y+4, $blue);
				$im->arc($x-3, $y-5, 5, 5, 0, 360, $blue);
				$im->arc($x+3, $y-5, 5, 5, 0, 360, $blue);
				$im->arc($x-3, $y+5, 5, 5, 0, 360, $blue);
				$im->arc($x+3, $y+5, 5, 5, 0, 360, $blue);
				#$im->arc($x, $y, 8, 8, 0, 360, $blue);
			}
		}
	}
	
	my @edges = split(/,/, $g);
	foreach my $e (@edges){
		my ($e1, $e2) = split(/=/, $e);
		my ($x1, $y1) = ($my_location{$e1}[0], $my_location{$e1}[1]);
		($x1, $y1) = (int(($x1 * $display_x)/ $norm_x), int(($y1 * $display_y)/ $norm_y));
		my ($x2, $y2) = ($my_location{$e2}[0], $my_location{$e2}[1]);
		($x2, $y2) = (int(($x2 * $display_x)/ $norm_x), int(($y2 * $display_y)/ $norm_y));
		$im->line($x1, $y1, $x2, $y2, $black);
	}

	$im->filledRectangle( ($base_x * $display_x)/$norm_x-5, ($base_y * $display_y)/$norm_y-5,
		($base_x * $display_x)/$norm_x+5, ($base_y * $display_y)/$norm_y+5, $red);
		
# 	$im->arc(($base_x * $display_x)/$norm_x,
# 		($base_y * $display_y)/$norm_y,
# 		2 * int(($comm_radius/$one_pt_sz * $display_x)/$norm_x),
# 		2 * int(($comm_radius/$one_pt_sz * $display_y)/$norm_y),
# 		0, 360, $black);
	
	$im->string(gdSmallFont, 10, $display_y-20, $t, $black);
	
	my $image_file = undef;
	if ($t < 10){
		$t = join ('', "000", $t);
		$image_file = join('.', "time", $t, "png");
	}elsif (($t >= 10) && ($t < 100)){
		$t = join ('', "00", $t);
		$image_file = join('.', "time", $t, "png");
	}elsif (($t >= 100) && ($t < 1000)){
		$t = join ('', "0", $t);
		$image_file = join('.', "time", $t, "png");
	}else{
		$image_file = join('.', "time", $t, "png");
	}

	open(FILEOUT, ">$image_file") or
		die "could not open file $image_file for writing!";
	binmode FILEOUT;
	print FILEOUT $im->png;
	close FILEOUT;
}

sub distance {
	my ($x1, $x2, $y1, $y2) = @_;
	return sqrt( ($x1-$x2)*($x1-$x2) + ($y1-$y2)*($y1-$y2) ) / 10;
}


sub compute_position{ # it could be also done using cos/sin
	my ($x1, $y1, $x0, $y0) = @_;
	my $x2 = undef;
	my $y2 = undef;
	if (($x0 - $x1) != 0){ ## avoid division by zero
		my $a = ($y0-$y1)/($x0-$x1);
		my $b = $y1 - $a*$x1;
		
		$x2 = (($speed*10)**2 - (distance($x0, $x1, $y0, $y1)*10 - $speed*10)**2 - $x0**2 - $y0**2 + $x1**2 + $y1**2 - 2*$b*$y1 + 2*$b*$y0)/(2*$x1 - 2*$x0 + 2*$a*$y1 - 2*$a*$y0);
		$y2 = $a*$x2 + $b;
	}else{
		$x2 = $x1;
		if ($y1 > $y0){
			$y2 = $y0 + $speed*10;
		}else{
			$y2 = $y0 - $speed*10;
		}
	}
	return ($x2, $y2);
}
