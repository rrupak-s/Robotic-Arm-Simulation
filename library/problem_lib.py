max_x_reach = 1.2
max_y_reach = 1.2
min_x_reach = -1.2
min_y_reach = -1.2

problem_count = 0


def vision_problem_check(coordinates_array):
    global max_x_reach, max_y_reach, min_x_reach, min_y_reach, problem_count
    if len(coordinates_array) == 0:
        problem_count = 2
    else:
        for x, y in coordinates_array:
            if x > max_x_reach or x < min_x_reach:
                problem_count = 1
                break
            if y > max_y_reach or y < min_y_reach:
                problem_count = 1
                break
        else:
            problem_count = 0

    return problem_count


def pick_glass(coordinates_array):
    if not coordinates_array:
        return None

    result = coordinates_array[0]

    for coord in coordinates_array:
        # or (coord[0] == result[0] and coord[1] > result[1]):
        if coord[0] > result[0]:
            result = coord

    return result


def block_check(points, region_corners):
    """
    Check if a list of points are inside a rectangular region defined by four corner points.

    Args:
    - points (list of tuples): A list of tuples, each containing the (x, y) coordinates of a point.
    - region_corners (list of tuples): A list of four tuples, each containing the (x, y) coordinates
      of a corner point of the rectangular region in clockwise order.

    Returns:
    - list of bool: A list of booleans indicating whether each point is inside the region.
    """
    x_coords, y_coords = zip(*region_corners)
    min_x, max_x, min_y, max_y = min(x_coords), max(
        x_coords), min(y_coords), max(y_coords)
    results = []

    for point in points:
        x, y = point
        if min_x <= x <= max_x and min_y <= y <= max_y:
            wn = 0
            for i in range(len(region_corners)):
                x1, y1 = region_corners[i]
                x2, y2 = region_corners[(i + 1) % len(region_corners)]
                if y1 <= y:
                    if y2 > y and (x2 - x1) * (y - y1) - (x - x1) * (y2 - y1) > 0:
                        wn += 1
                else:
                    if y2 <= y and (x2 - x1) * (y - y1) - (x - x1) * (y2 - y1) < 0:
                        wn -= 1
            results.append(wn != 0)
        else:
            results.append(False)

    return results
