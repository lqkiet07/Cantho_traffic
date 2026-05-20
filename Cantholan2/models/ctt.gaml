/**
* Name: NewModel
* Based on the internal empty template. 
* Author: smth
* Tags: 
*/
model Traffic

/* Insert your model definition here */
global {
	file road_shp <- shape_file("../includes/road.shp");
	file building_shp <- shape_file("../includes/building.shp");
	file signal_shp <- shape_file("../includes/traffic_signals.shp");
	geometry shape <- envelope(road_shp);
	graph road_network;
	float step <- 0.1 #s;
	string volume_scenario <- "High";
	string routing_scenario <- "Bình thường";
	float spawn_rate <- 1.0;
	
	list<intersection> spawn_nodes; // spawn points at the edge of the map
	map<road, float> road_heat;  // smoothed heat value for road density
	int heat_tick <- 0;          // step counter to update heatmap periodically

	// update road_heat using ema for smooth color transitions
	reflex update_road_counts {
		heat_tick <- heat_tick + 1;
		if (heat_tick mod 5 = 0) {
			// instant vehicle count per road
			map<road, int> cur <- map<road,int>([]);
			loop v over: (motobike as list) + (car as list) + (truck as list) {
				if (v.current_road != nil) {
					road rd <- road(v.current_road);
					if (rd != nil) {
						cur[rd] <- (cur contains_key rd) ? cur[rd] + 1 : 1;
					}
				}
			}
			// apply ema smoothing for heat values
			loop r over: road {
				float new_val <- (cur contains_key r) ? min(float(cur[r]) / 40.0, 1.0) : 0.0;
				float old_val <- (road_heat contains_key r) ? road_heat[r] : 0.0;
				road_heat[r] <- 0.7 * old_val + 0.3 * new_val;
			}
		}
	}

	init {
		write "read data";
		
		list<geometry> fixed_road <- clean_network(list<geometry>(road_shp.contents), 15.0, true, true);
		create road from: road_shp;

		create building from: building_shp;

		graph temp_graph <- as_edge_graph(road);
		loop v over: temp_graph.vertices {
			create intersection with: [shape::point(v)] {
				is_traffic_signal <- false;
			}
		}
		road_network <- as_driving_graph(road, intersection);

		// group traffic signals within 70m radius
		list<geometry> free_signals <- list<geometry>(signal_shp.contents);
		loop while: !empty(free_signals) {
			geometry head_sg <- free_signals[0];
			
			// find all signal points in cluster
			list<geometry> cluster_sg <- free_signals where (each distance_to head_sg < 70.0);
			
			if (length(cluster_sg) >= 2) {
				// calculate real center of intersection
				point real_center <- mean(cluster_sg collect each.location);
				
				// find closest intersection node
				intersection target_node <- (intersection) closest_to(real_center);
				
				if (target_node != nil) {
					ask target_node {
						is_traffic_signal <- true;
						// compute crossing using signal positions and real center
						do compute_crossing(sig_pts: cluster_sg collect each.location, center_pt: real_center);
					}
					
					// create visual traffic lights
					loop sg_pt over: cluster_sg {
						create traffic_light_visual {
							location <- sg_pt.location;
							my_parent <- target_node;
							// calculate angle to determine axis
							float ang <- location towards real_center;
							float norm_ang <- ang mod 180;
							if (norm_ang > 45 and norm_ang < 135) {
								axis <- "axis_1";
							} else {
								axis <- "axis_2";
							}
						}
					}
				}
			}
			free_signals <- free_signals - cluster_sg;
		}
		// identify spawn nodes based on graph degree
		spawn_nodes <- intersection where (
			!each.is_traffic_signal and
			!empty(each.roads_out) and
			(length(each.roads_out) + length(each.roads_in) <= 2)
		);
		// fallback if too few spawn nodes found
		if (length(spawn_nodes) < 3) {
			spawn_nodes <- intersection where (!each.is_traffic_signal and !empty(each.roads_out));
		}
		


		int nb_moto <- 0;
		int nb_car <- 0;
		int nb_truck <- 0;
		write "done";

		
		list<intersection> signal_nodes <- intersection where (each.is_traffic_signal);
		loop while: not empty(signal_nodes){
			intersection seed <- signal_nodes[0];
			list<intersection> cluster <- signal_nodes where (each distance_to seed <= 50.0);
			
			create traffic_controller{
				my_nodes <- cluster;
				location <- cluster[0].location;
				ask my_nodes {
					do to_green;
				}
			}
			signal_nodes <- signal_nodes - cluster;
		}

	}

	reflex maintain_population {
		int target_population <- 0;
		switch volume_scenario {
			match "High" { target_population <- 1600; }
			match "Mid"  { target_population <- 800;  }
			match "Low"  { target_population <- 400;  }
		}

		// count actual total vehicles
		int current_population <- length(motobike) + length(car) + length(truck);
		int diff <- target_population - current_population;

		if (diff > 0) {
			int spawn_count <- min(diff, 3); // spawn up to 3 vehicles per step

			loop times: spawn_count {
				intersection end_node <- nil;
				float cx <- shape.location.x;
				float cy <- shape.location.y;
				switch routing_scenario {
					match "Trục dọc kẹt cứng" {
						list<intersection> ns <- spawn_nodes where (each.location.y < cy - 100 or each.location.y > cy + 100);
						list<intersection> ew <- spawn_nodes where (each.location.x < cx - 100 or each.location.x > cx + 100);
						if (flip(0.8) and !empty(ns)) {
							end_node <- one_of(ns);
						} else if (!empty(ew)) {
							end_node <- one_of(ew);
						} else {
							end_node <- one_of(spawn_nodes);
						}
					}
					match "Đổ dồn về phía Đông" {
						list<intersection> east <- spawn_nodes where (each.location.x > cx + 100);
						end_node <- !empty(east) ? one_of(east) : one_of(spawn_nodes);
					}
					match "Bình thường" {
						end_node <- one_of(spawn_nodes);
					}
				}

				if (end_node != nil) {
					intersection start_node <- one_of(spawn_nodes);
					if (start_node != nil and start_node != end_node) {
						// allow spawn if less than 2 vehicles within 8m
						if (length(vehicle overlapping circle(8.0, start_node.location)) < 2) {
							float rnd_type <- rnd(1.0);
							if (rnd_type < 0.6) {
								create motobike number: 1 { location <- start_node.location; final_target <- end_node; }
							} else if (rnd_type < 0.9) {
								create car number: 1 { location <- start_node.location; final_target <- end_node; }
							} else {
								create truck number: 1 { location <- start_node.location; final_target <- end_node; }
							}
						}
					}
				}
			}
		} else if (diff < 0) {
			// remove random vehicles if population exceeds target
			ask abs(diff) among (motobike as list) + (car as list) + (truck as list) { do die; }
		}
	}


species road skills: [road_skill] {
	int lanes <- 3;
	int num_lanes <- 3;
	float width <- 6.0;

	aspect default {
		draw shape + (width / 4) color: #black;
	}

	// base style for heatmap road display
	aspect heatmap_base {
		draw shape + (width / 4) color: rgb(35, 55, 90);
	}

	// 3-state heatmap color based on road density
	aspect heatmap_heat {
		float h <- (road_heat contains_key self) ? road_heat[self] : 0.0;
		// mapped value for max threshold
		if (h >= 0.5) {
			// high density color
			draw shape + (width / 2) color: rgb(220, 30, 30);
		} else if (h >= 0.2) {
			// medium density color
			draw shape + (width / 2) color: rgb(255, 190, 0);
		} else if (h >= 0.06) {
			// low density color
			draw shape + (width / 2) color: rgb(40, 200, 80);
		}
		// do not draw if below threshold
	}

}

species building {

	aspect default {
		draw shape color: #grey;
	}

}

species vehicle skills: [driving] {

	init {
		right_side_driving <- true;
		safety_distance_coeff <- 3.0;
	}
	//find road
	reflex move {
		if (final_target = nil or (location distance_to final_target.location < 5.0)) {
			do die;
		}
		else{
			if (current_path = nil) {
				do compute_path graph: road_network target: final_target;
				if (current_path = nil) {
					// remove if no path available
					//write "vehicle killed due to no path";
					do die;
					return;
				}
			}	
			// stop only if red traffic light is directly ahead within 90 degrees
			traffic_light_visual light_ahead <- traffic_light_visual closest_to self;
			bool should_stop <- false;
			bool should_slow <- false;
			float dist_to_light <- (light_ahead != nil) ? self distance_to light_ahead : #infinity;

			if (light_ahead != nil and light_ahead.state = "red") {
				float angle_to_light <- float(self towards light_ahead);
				float diff_ang <- abs(angle_to_light - heading) mod 360.0;
				if (diff_ang > 180.0) { diff_ang <- 360.0 - diff_ang; }
				if (diff_ang < 90.0) {
					if (dist_to_light < 5.0)  { should_stop <- true; }  // hard stop zone
					else if (dist_to_light < 18.0) { should_slow <- true; } // braking zone
				}
			}

			if (should_stop) {
				// hold position at red light
				speed <- 0.0;
			} else if (should_slow) {
				// gradual braking: reduce speed proportionally to distance
				float brake_ratio <- (dist_to_light - 5.0) / 13.0; // 1.0 far, 0.0 at stop line
				speed <- max_speed * brake_ratio * 0.4;
				do drive;
			} else {
				// restore speed in case it was held at 0 from previous red light
				if (speed = 0.0) { speed <- max_speed * 0.5; }
				do drive;
			}
		}
		
	}

	point compute_position {
		if (current_road != nil) {
			float road_width <- road(current_road).width;
			int n_lanes <- road(current_road).num_lanes;
			float lane_w <- road_width / n_lanes;

			
			float dist_from_left_edge <- (n_lanes - lowest_lane - 0.5) * lane_w;
			float center_offset <- dist_from_left_edge - (road_width / 2);
			float final_dist <- -center_offset;
			point shift_pt <- {cos(heading + 90) * final_dist, sin(heading + 90) * final_dist};
			return location + shift_pt;
		} else {
			return location;
		}

	}

}

species motobike parent: vehicle {

	init {
		vehicle_length <- 2.0;
		max_speed <- rnd(40.0, 60.0) #km / #h;
		speed <- max_speed;
	}

	aspect default {
		point pos <- compute_position();
		draw box(2, 1, 1) color: #green rotate: heading at: {pos.x, pos.y, 0.5};
	}

	// heatmap point display
	aspect heat_dot {
		draw circle(10) color: rgb(255, 60, 0, 80);
	}

}

species car parent: vehicle {

	init {
		vehicle_length <- 4.0;
		max_speed <- rnd(30.0, 50.0) #km / #h;
		speed <- max_speed;
	}

	aspect default {
		point pos <- compute_position();
		draw box(4, 2, 2) color: #red rotate: heading at: {pos.x, pos.y, 1};
	}

	// larger heatmap point for car
	aspect heat_dot {
		draw circle(13) color: rgb(255, 60, 0, 90);
	}

}

species truck parent: vehicle {

	init {
		vehicle_length <- 6.0;
		max_speed <- rnd(20.0, 40.0) #km / #h;
		speed <- max_speed;
	}

	aspect default {
		point pos <- compute_position();
		draw box(6, 3, 3) color: #blue rotate: heading at: {pos.x, pos.y, 1.5};
	}

	// largest heatmap point for truck
	aspect heat_dot {
		draw circle(16) color: rgb(255, 60, 0, 100);
	}

}

species intersection skills: [intersection_skill] {
	bool is_green;
	bool is_traffic_signal;
	
	map<road,int> queue_per_road;
	int queue_ways1 <- 0;
	int queue_ways2 <- 0;
	list<road> ways1 <- [];
	list<road> ways2 <- [];
	rgb color_fire;
	int start_phase <- 1;

	//caculate lane for intersection
	action compute_crossing(list<point> sig_pts, point center_pt) {
		ways1 <- [];
		ways2 <- [];
		if (empty(roads_in) or empty(sig_pts)) { return; }

		list<road> all_roads <- roads_in collect road(each);

		// map signal points to nearest roads
		loop sg_pt over: sig_pts {
			float ang <- sg_pt towards center_pt;
			float normalized_ang <- ang mod 180;

			road best_rd <- nil;
			float min_d <- #infinity;
			loop rd over: all_roads {
				float d <- sg_pt distance_to road(rd).shape;
				if (d < min_d) { min_d <- d; best_rd <- road(rd); }
			}
			if (best_rd != nil) {
				if (normalized_ang > 45 and normalized_ang < 135) {
					if (!(ways1 contains best_rd)) { ways1 <- ways1 + [best_rd]; }
				} else {
					if (!(ways2 contains best_rd)) { ways2 <- ways2 + [best_rd]; }
				}
			}
		}

		// assign unmapped roads to balance groups
		loop rd over: all_roads {
			if (!(ways1 contains rd) and !(ways2 contains rd)) {
				if (length(ways1) <= length(ways2)) {
					ways1 <- ways1 + [rd];
				} else {
					ways2 <- ways2 + [rd];
				}
			}
		}
	}

	action to_green {
		// update visual state for green light
		color_fire <- #green;
		is_green <- true;
		// switch lights axis_1 to green and axis_2 to red
		ask traffic_light_visual where (each.my_parent = self) {
			state <- (axis = "axis_1") ? "green" : "red";
		}
	}

	action to_red {
		// update visual state for red light
		color_fire <- #red;
		is_green <- false;
		// switch lights axis_2 to green and axis_1 to red
		ask traffic_light_visual where (each.my_parent = self) {
			state <- (axis = "axis_2") ? "green" : "red";
		}
	}


	reflex calculate_queue when: is_traffic_signal {
		// reset queue counters
		loop rd over: roads_in {
			queue_per_road[road(rd)] <- 0;
		}
		
		list<vehicle> near_vehicles <- vehicle where (each distance_to self < 30.0);
		int count_slow <- 0;
		
		loop v over: near_vehicles {
			if (v.current_road != nil) {
				road current_rd <- road(v.current_road);
				if (current_rd in roads_in) {
					// in ra tốc độ của xe đang ở gần ngã tư để kiểm tra
					// write name + " - Vehicle near: speed = " + v.speed + " real_speed = " + v.real_speed;
					
					if (v.speed < 5 #km/#h or v.real_speed < 5 #km/#h) {
						queue_per_road[current_rd] <- queue_per_road[current_rd] + 1;
						count_slow <- count_slow + 1;
					}
				}
			}
		}
		
		queue_ways1 <- ways1 sum_of (queue_per_road[each]);
		queue_ways2 <- ways2 sum_of (queue_per_road[each]);
		
		// Luôn in ra để kiểm tra xem hàm có chạy không, và length của ways1, ways2
		// write name + " | roads_in: " + length(roads_in) + " | ways1: " + length(ways1) + " | ways2: " + length(ways2) + " | slow_vehicles: " + count_slow;
		
		if (queue_ways1 > 0 or queue_ways2 > 0) {
			write name + " | Hướng 1: " + queue_ways1 + " xe | Hướng 2: " + queue_ways2 + " xe";
		}
		
		// In ra bắt buộc một chu kỳ (VD: tick mod 100) để theo dõi nếu nó cứ bằng 0
		if (cycle mod 50 = 0 and (length(ways1) > 0 or length(ways2) > 0)) {
			write "DEBUG " + name + " -> roads_in: " + length(roads_in) + " | ways1: " + length(ways1) + " | ways2: " + length(ways2) + " | near_veh: " + length(near_vehicles) + " | slow: " + count_slow;
		}
	}

	aspect default {
		if (is_traffic_signal) {
//			rgb light_color <- is_green ? #green : #red;
//			draw cylinder(0.3, 5) at: location color: #black;
//	        draw sphere(1.5) at: {location.x, location.y, 5} color: light_color;
		}else{
//			draw circle(1) color: color;
		}
	}
}

species traffic_controller{
	list<intersection> my_nodes;
	float time_to_change <- 60 #s;
	float counter <- 0.0;
	bool is_green <- true;
	
	int queue_ways1 <- 0;
	int queue_ways2 <- 0;
	
	reflex run_cycle{
		counter <- counter + step;
		if(counter >= time_to_change){
			counter <- 0.0;
			ask my_nodes {
				if (is_green) { do to_red; } 
                else { do to_green; }
			}
		}
	}
}

species traffic_light_visual {
    intersection my_parent;
    road my_road; // optional variable for compatibility
    string axis;  // axis identifier
    string state <- "red"; // visual state of traffic light

    aspect default {
    	rgb light_color <- (state = "green") ? #green : #red;
        draw cylinder(0.3, 5) color: #black;
        draw sphere(1.2) at: {location.x, location.y, 5} color: light_color;
    }
}

}
experiment test type: gui {
	parameter "Lưu lượng xe:" var: volume_scenario among: ["High", "Mid", "Low"];
	//parameter "Kịch bản di chuyển:" var: routing_scenario among: ["Bình thường", "Trục dọc kẹt cứng", "Đổ dồn về phía Đông"];
	output {
		display main type: 3d background: #lightskyblue axes: false {
			species road refresh: false;
			species motobike;
			species car;
			species truck;
			species intersection;
			species traffic_light_visual;
		}
		
		display heatmap type: 3d background: rgb(8, 12, 25) axes: false {
			// base road layer
			species road aspect: heatmap_base refresh: false;
			// overlay heat dots based on actual vehicle positions
			species motobike aspect: heat_dot;
			species car aspect: heat_dot;
			species truck aspect: heat_dot;
		}
	}
}