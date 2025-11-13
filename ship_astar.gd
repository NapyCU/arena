class_name ship_astar

static func find_path(start_poly: int, goal_poly: int, polygons: Array, neighbors: Array) -> Array:
	var open_set = [start_poly]
	var came_from = {}
	var g_score = {}
	var f_score = {}

	for i in range(polygons.size()):
		g_score[i] = INF
		f_score[i] = INF
	g_score[start_poly] = 0
	f_score[start_poly] = heuristic_cost_estimate(start_poly, goal_poly, polygons)

	while open_set.size() > 0:
		var current = get_lowest_fscore(open_set, f_score)
		if current == goal_poly:
			return reconstruct_path(came_from, current)

		open_set.erase(current)
		for neighbor in neighbors[current]:
			var tentative_g = g_score[current] + polygon_distance(current, neighbor, polygons)
			if tentative_g < g_score[neighbor]:
				came_from[neighbor] = current
				g_score[neighbor] = tentative_g
				f_score[neighbor] = g_score[neighbor] + heuristic_cost_estimate(neighbor, goal_poly, polygons)
				if neighbor not in open_set:
					open_set.append(neighbor)

	return []  # no path found


static func reconstruct_path(came_from: Dictionary, current: int) -> Array:
	var total_path = [current]
	while came_from.has(current):
		current = came_from[current]
		total_path.insert(0, current)
	return total_path


static func heuristic_cost_estimate(a: int, b: int, polygons: Array) -> float:
	return polygon_distance(a, b, polygons)


static func polygon_distance(a: int, b: int, polygons: Array) -> float:
	var center_a = polygon_center(polygons[a])
	var center_b = polygon_center(polygons[b])
	return center_a.distance_to(center_b)


static func polygon_center(poly: PackedVector2Array) -> Vector2:
	var sum = Vector2.ZERO
	for v in poly:
		sum += v
	return sum / poly.size()


static func find_polygon(point: Vector2, polygons: Array) -> int:
	for i in range(polygons.size()):
		if Geometry2D.is_point_in_polygon(point, polygons[i]):
			return i
	return -1
	
static func get_lowest_fscore(open_set: Array, f_score: Dictionary) -> int:
	var lowest = open_set[0]
	var lowest_val = f_score[lowest]
	for node in open_set:
		var val = f_score[node]
		if 	val < lowest_val:
			lowest = node
			lowest_val = val
	return lowest
