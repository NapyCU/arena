class_name ship_astar

static func find_path(start_poly: int, goal_poly: int, polygons: Array, neighbors: Array) -> Array:
	var open_set = [start_poly]
	var came_from = {}
	var g_score = {}
	var cost = {}

	for i in range(polygons.size()):
		g_score[i] = INF
		cost[i] = INF
	g_score[start_poly] = 0
	cost[start_poly] = heuristic(start_poly, goal_poly, polygons)

	while open_set.size() > 0:
		var current = get_lowest_fscore(open_set, cost)
		if current == goal_poly:
			return polygons_to_waypoints(reconstruct_path(came_from, current), polygons, neighbors)

		open_set.erase(current)
		for neighbor in neighbors[current]:
			var tentative_g = g_score[current] + polygon_distance(current, neighbor, polygons)
			if tentative_g < g_score[neighbor]:
				came_from[neighbor] = current
				g_score[neighbor] = tentative_g
				cost[neighbor] = g_score[neighbor] + heuristic(neighbor, goal_poly, polygons)
				if neighbor not in open_set:
					open_set.append(neighbor)

	return []  # no path found

static func polygons_to_waypoints(polygon_indices, polygons, neighbors) -> Array[Vector2]:
	var path : Array[Vector2]
	for i in range(polygon_indices.size() -1):
		var curr_polygon = polygons[polygon_indices[i]]
		var next_polygon = polygons[polygon_indices[i+1]]
		var intersect = get_common_intersect(curr_polygon, next_polygon)
		path.append(intersect)
	return path
		
		
static func get_common_intersect(current, next):
	# find common vertex
	var common_vertex
	var index_other_common = -1
	var index_curret_common = -1
	var common_vertices = Array()
	for c in range(current.size()):
		var vert = current[c]
		for n in range(next.size()):
			var other_vert = next[n]
			if(other_vert == vert):
				common_vertices.append(vert)
	# have single common_vertex
	return (common_vertices[0] + common_vertices[1])/2

static func reconstruct_path(came_from: Dictionary, current: int) -> Array:
	var total_path = [current]
	while came_from.has(current):
		current = came_from[current]
		total_path.insert(0, current)
	return total_path


static func heuristic(a: int, b: int, polygons: Array) -> float:
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
	
static func get_lowest_fscore(open_set: Array, cost: Dictionary) -> int:
	var lowest = open_set[0]
	var lowest_val = cost[lowest]
	for node in open_set:
		var val = cost[node]
		if 	val < lowest_val:
			lowest = node
			lowest_val = val
	return lowest
