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
var update_path_next = true

var break_count : int
var last_ship_polygon
# This method is called on every tick to choose an action.  See README.md
# for a detailed description of its arguments and return value.
func action(_walls: Array[PackedVector2Array], _gems: Array[Vector2], 
			_polygons: Array[PackedVector2Array], _neighbors: Array[Array]):
	polygons = _polygons
	gems = _gems
	neighbors = _neighbors
	var curr_ship_polygon = ship_astar.find_polygon(ship.position, polygons)
	if(curr_ship_polygon != last_ship_polygon):
		force_update_path()
	if(update_path_next ) : update_path()
	
	if(break_count > 0):
		break_ship()
	else:
		follow_path(curr_path)
	last_ship_polygon = curr_ship_polygon
	return [spin, thrust, false]

# Called every time the agent has bounced off a wall.
func bounce():	
	consider_break()
	force_update_path()
# Called every time a gem has been collected.
func gem_collected():
	consider_break()
	
	force_update_path()
# Called every time a new level has been reached.
func new_level():
	gem_polygon .clear()
func update_path():
	update_path_next = false
	var ship_pos = ship.position
	var gem = find_closest()  # for now, just pick first gem
	var ship_poly = ship_astar.find_polygon(ship_pos, polygons)
	var gem_poly = ship_astar.find_polygon(gem, polygons)
	current_waypoint_index = 0
	if ship_poly == -1 or gem_poly == -1:
		print("Failed")
		return [0, false, false]  # fallback
	var poly_path = ship_astar.find_path(ship_poly, gem_poly, polygons, neighbors)
	poly_path[poly_path.size() - 1] = -1
	
	curr_path.clear()
	
	for point in poly_path:
		var target
		if(point == -1):
			target = gem
		else:	
			target = ship_astar.polygon_center(polygons[point])
		curr_path.append(target)
	queue_redraw()
	if(curr_path.size() > 1):
		current_waypoint_index = 1
		

var current_waypoint_index = 0
var waypoint_reach_dist = 50.0  # how close before moving to next point
var facing_threshold = 5  # radians (~23°)
var stop_threshold = 0.08       # radians (~5°)
var arrive_seconds = 2
var break_speed_tolerace = 20
var debug_target

func follow_path(path: Array[Vector2]):
	if path.is_empty():
		spin = 0
		thrust = false
		print("no path to follow")
		force_update_path()
		return
	
	# ensure index is valid
	var target = path[current_waypoint_index]
	
	
	# if close  -  next waypoint
	if ship.position.distance_to(target) < waypoint_reach_dist:
		if current_waypoint_index < path.size() - 1:
			current_waypoint_index += 1
			target = path[current_waypoint_index]
			print("reached waypoint")
			consider_break()
			return
		else:
			# reached end of path
			print("reached path_end")
			spin = 0
			thrust = false
			force_update_path()
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

	# --- THRUST (forward acceleration) ---
	# Only thrust when roughly facing the target
	
	var dist_to_end = ship.position.distance_to(target)
	var covered_distance = ship.velocity.length() * arrive_seconds
	
	var velocity_target_diff = desired_angle - ship.velocity.angle()
	velocity_target_diff = wrapf(velocity_target_diff, -PI, PI)
	
	if ship.velocity.length() > break_speed_tolerace and  abs(velocity_target_diff) > deg_to_rad(facing_threshold):
		# arrive do not overshoot
		if(covered_distance > dist_to_end):
			thrust = false
			start_break()
		else:
			thrust = true
	
	if(spin == 0 and thrust == false):
		if(covered_distance < dist_to_end):
			thrust	= true

	# Optionally slow down near end of path (ARRIVE behavior)
		
func _draw():
	debug_path.clear_points()
	for point in curr_path:
		debug_path.add_point(point)
	if(curr_path.size() > 0):
		debug_target = curr_path[current_waypoint_index]
		draw_circle(to_local(debug_target), 15, Color(1, 0, 0))  # red circle radius 15
		
		

func start_break():
	break_count = BREAKS

var break_confirmed = false
func break_ship():
	if(break_confirmed == false and ship.velocity.length() < break_speed_tolerace):
		break_count = 0
		print("No need to break")
		break_confirmed = false
		
	if break_count > 0:
		var velocity_dir = ship.velocity.normalized()
		var curr_heading = Vector2.RIGHT.rotated(ship.rotation)
		
		var angle_diff = curr_heading.angle_to(-velocity_dir)
		var brake_threshold = 0.1  # radians (~6°)
		
		if abs(angle_diff) < brake_threshold:
			thrust = true
			spin = 0
		else:
			spin = sign(angle_diff)
			thrust = false
		
		print("BREAKS")
		break_confirmed = true
		break_count -= 1
		if ship.velocity.length_squared() < break_speed_tolerace:
			print("Finsihed breaking")
			break_count = 0
			break_confirmed = false
			
func force_update_path():
	update_path_next = true		

func find_closest():
	calc_gem_polygons()
	var closest = null
	var ship_polygon = ship_astar.find_polygon(ship.position, polygons)
	var ship_neighbors:Array = neighbors[ship_polygon]
	for gem in gems:
		var curr_gem_poly = gem_polygon[gem]
		if(curr_gem_poly == ship_polygon):
			return gem
		if(ship_neighbors.has(curr_gem_poly)):
			closest = gem
	if(closest == null):
		return gems[0]
	return closest

var gem_polygon : Dictionary
func calc_gem_polygons():
	if(gem_polygon.is_empty() == false) :return
	for gem in gems:
		gem_polygon[gem] = ship_astar.find_polygon(gem, polygons)
		
func consider_break():
	var target=curr_path[current_waypoint_index]
	if(ship.velocity.angle_to(target - ship.position) > deg_to_rad(facing_threshold)):
		start_break()
