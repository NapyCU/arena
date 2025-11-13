extends Node2D

@onready var ship : Ship = get_parent()
@onready var debug_path : Line2D = ship.get_node('../debug_path')

var ticks = 0
var spin = 0
var thrust = false

var overrides : int
const OVERRIDE_FRMAES = 100
const BREAKS = 100
var curr_path : Array[Vector2]
var polygons
var gems
var neighbors
var walls
var update_path_next = true
var kamikaze = false
var kamikaze_target
var break_count : int
var last_ship_polygon
var elapsed = 0
var shoot_on_bounce = false
var shoot = false
# This method is called on every tick to choose an action.  See README.md
# for a detailed description of its arguments and return value.
func action(_walls: Array[PackedVector2Array], _gems: Array[Vector2], 
			_polygons: Array[PackedVector2Array], _neighbors: Array[Array]):
	polygons = _polygons
	gems = _gems
	walls = _walls
	neighbors = _neighbors
	
	var curr_ship_polygon = ship_astar.find_polygon(ship.position, polygons)
#	if(gems.size() <= 1):
#		if(free_path(gems[0], true) == 1):
#			kamikaze = true
#			kamikaze_target = gems[0]
#			shoot_on_bounce = true
#			print("Will shoot when bounces")
	
	if(curr_ship_polygon != last_ship_polygon):
		force_update_path()
	if(update_path_next) : update_path()
	
	if(break_count > 0):
		break_ship()
	else:
		follow_path(curr_path)
	last_ship_polygon = curr_ship_polygon
	return [spin, thrust, shoot]

var wall_hit_count = 0
var wall_hit_pos = null
# Called every time the agent has bounced off a wall.
func bounce():	
	kamikaze = false
	if(wall_hit_pos != null and ship.position.distance_squared_to(wall_hit_pos) < 50):
		wall_hit_count +=1
		wall_hit_pos = ship.position
		if(wall_hit_count > 100):		
			shoot_on_bounce = true
			wall_hit_count = 0
			wall_hit_pos = ship.position
	else:
		wall_hit_count = 0
		wall_hit_pos = ship.position
	if(shoot_on_bounce):
		shoot = true
	else:
		consider_brake()
#	force_update_path()
# Called every time a gem has been collected.
func gem_collected():
	kamikaze = false
	kamikaze_target = null
	last_closest = null
	force_update_path()
	consider_brake()
# Called every time a new level has been reached.
func new_level():
	kamikaze =false
	force_update_path()
	gem_polygon .clear()
	shoot_on_bounce = false
	shoot = false
	
func update_path():
	update_path_next = false
	var ship_pos = ship.position
	var gem = find_closest()  # for now, just pick first gem
	var ship_poly = ship_astar.find_polygon(ship_pos, polygons)
	var gem_poly = ship_astar.find_polygon(gem, polygons)
	
	if(ship_poly != gem_poly and free_path(gem)):
		kamikaze = true
		kamikaze_target = gem
		print("Start_kamiakze")
		return
		
	
	current_waypoint_index = 0
	if ship_poly == -1 or gem_poly == -1:
		print("Failed")
		return [0, false, false]  # fallback
	curr_path = ship_astar.find_path(ship_poly, gem_poly, polygons, neighbors)
	curr_path.append(gem)
	queue_redraw()


var current_waypoint_index = 0
var waypoint_reach_dist = 50  # how close before moving to next point
var facing_threshold = 4  # radians (~23°)
var safety_arrive_margin = 50
var break_speed_tolerace = 5
var debug_target

func follow_path(path: Array[Vector2]):
	if path.is_empty() and not kamikaze:
		spin = 0
		thrust = false
		print("no path to follow")
		force_update_path()
		return
	
	var target 
	if(kamikaze):
		target = kamikaze_target	
		path.clear()
	else:
		target = path[current_waypoint_index]
	queue_redraw()
	# ensure index is valid
	
	
	# if close  -  next waypoint
	if not kamikaze:
		if ship.position.distance_to(target) < waypoint_reach_dist:
			if current_waypoint_index < path.size() - 1:
				current_waypoint_index += 1
				target = path[current_waypoint_index]
				print("reached waypoint")
				consider_brake(2)
				return
		
	# --- SEEK BEHAVIOR ---
	var desired_velocity = (target - ship.position).normalized()

	# desired heading
	var desired_angle = desired_velocity.angle()
	var current_angle = ship.rotation

	# wrap angle difference into [-PI, PI]
	var diff = wrapf(desired_angle - current_angle, -PI, PI)

	# --- STEERING (turning) ---
	if abs(diff) < deg_to_rad(facing_threshold):
		spin = 0
	else:
		spin = int(sign(diff))  # simple left/right turn

	# Thrust
	
	var dist_to_end = ship.position.distance_to(target)
	
	var velocity_target_diff = desired_angle - ship.velocity.angle()
	velocity_target_diff = wrapf(velocity_target_diff, -PI, PI)
	
	if(spin == 0 and thrust == false):
		thrust = true
	
#	if ship.velocity.length() > break_speed_tolerace * 4 and  abs(velocity_target_diff) > deg_to_rad(facing_threshold * 2):
#		thrust = true
		
	# ARRIVE		
	var dist = ship.position.distance_to(target)
	var speed = ship.velocity.length()
	var time_to_stop = speed / ship.ACCEL

	if dist < speed * time_to_stop:
		consider_brake()
		thrust = false

	# Optionally slow down near end of path (ARRIVE behavior)
		
func _draw():
	debug_path.clear_points()
	var ai = global_transform.affine_inverse()
	for point in curr_path:
		debug_path.add_point(point)
	if(kamikaze):
		debug_path.add_point(ship.position)
		debug_path.add_point(kamikaze_target)
		
		

func start_brake():
	break_count = BREAKS
	break_remaining_rotation = 1

var break_confirmed = false
var break_remaining_rotation = 1
func break_ship():
	if(break_confirmed == false and ship.velocity.length() < break_speed_tolerace):
		break_count = 0
		print("No need to break")
		break_confirmed = false
		break_remaining_rotation = 1
		
	if break_count > 0:
		var velocity_dir = ship.velocity.normalized()
		var curr_heading = Vector2.RIGHT.rotated(ship.rotation)
		
		var angle_diff = curr_heading.angle_to(-velocity_dir)
		var brake_threshold = 0.1  
		break_remaining_rotation = angle_diff/PI
		if abs(angle_diff) < brake_threshold:
			thrust = true
			spin = 0
		else:
			spin = sign(angle_diff)
			thrust = false
		
		break_confirmed = true
		break_count -= 1
		if ship.velocity.length_squared() < break_speed_tolerace:
			print("Finsihed breaking")
			break_count = 0
			break_remaining_rotation = 1
			break_confirmed = false
			
func force_update_path():
	update_path_next = true		

var last_closest = null
func find_closest():
	calc_gem_polygons()
	var closest = null
	var ship_polygon = ship_astar.find_polygon(ship.position, polygons)
	var ship_neighbors:Array = neighbors[ship_polygon]
	var min_dist = ship.position.distance_squared_to(gems[0])
	var min_dist_gem = gems[0]
	for gem in gems:
		var curr_gem_poly = gem_polygon[gem]
		if(curr_gem_poly == ship_polygon):
			closest = gem
			break
		if(ship_neighbors.has(curr_gem_poly)):
			closest = gem
		else:
			var d = ship.position.distance_squared_to(gem)
			if(d < min_dist):
				min_dist =d
				min_dist_gem = gem
	if(closest == null):
		closest = min_dist_gem
	
	if(last_closest == null or ship.position.distance_squared_to(closest) * 2 < ship.position.distance_squared_to(last_closest)):
		last_closest = closest
		return closest
	else:
		return last_closest

var gem_polygon : Dictionary
func calc_gem_polygons():
	if(gem_polygon.is_empty() == false) :return
	for gem in gems:
		gem_polygon[gem] = ship_astar.find_polygon(gem, polygons)
		
func consider_brake(tolerance_multiplier = 1):
	if(curr_path.size() > 0 and current_waypoint_index + 1 <curr_path.size()):
		var target=curr_path[current_waypoint_index + 1]
		if(ship.velocity.angle_to(target - ship.position) > deg_to_rad(facing_threshold * tolerance_multiplier)):
			start_brake()
	else:
		start_brake()
func free_path(target: Vector2, count_all_walls: bool = false):
	var pos = ship.position
	var dist_to_target = pos.distance_to(target)

	var counter = 0

	for wall in walls:
		var wall_points = wall
		var wall_point_count = wall_points.size()

		# quick reject: check if wall is far away
		# (distance from ship to wall's bounding box > dist_to_target)
		var wall_min = Vector2(INF, INF)
		var wall_max = Vector2(-INF, -INF)
		for p in wall_points:
			wall_min = wall_min.min(p)
			wall_max = wall_max.max(p)

		var wall_center = (wall_min + wall_max) * 0.5
		if pos.distance_to(wall_center) > dist_to_target + 50.0:
			continue  # skip walls behind or beyond the target

		for i in range(wall_point_count):
			var a = wall_points[i]
			var b = wall_points[(i + 1) % wall_point_count]

			var t = _segments_intersect_t(pos + Vector2.DOWN * ship.RADIUS*2, target, a, b)
			var t2 = _segments_intersect_t(pos + Vector2.UP * ship.RADIUS *2 , target, a, b)

			if (t >= 0.0 and t <= 1.0) or (t2 >= 0.0 and t2 <= 1.0):
				counter += 1
				
				if not count_all_walls:
					return false
				break

	if count_all_walls:
		return counter
	else:
		return counter == 0

# this code was taken from itnernet

func _segments_intersect_t(p1: Vector2, p2: Vector2, q1: Vector2, q2: Vector2) -> float:
	var r = p2 - p1
	var s = q2 - q1
	var rxs = r.cross(s)
	var q_p = q1 - p1
	var q_pxr = q_p.cross(r)

	if abs(rxs) < 0.0001 and abs(q_pxr) < 0.0001:
		# collinear → treat as no valid intersection for a ray
		return -1.0

	if abs(rxs) < 0.0001:
		# parallel
		return -1.0

	var t = q_p.cross(s) / rxs
	var u = q_p.cross(r) / rxs

	if t >= 0.0 and t <= 1.0 and u >= 0.0 and u <= 1.0:
		return t   # intersection at parameter t
	return -1.0
