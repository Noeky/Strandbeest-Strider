"""
Lightweight leg simulation interface.

Parameters order (length 13 list/tuple):
[link1, link2, link3, link4, link5, link6, link7, link8, link9, link10, link11, node4_x, node4_y]

Public API:
- evaluate_params(p, theta_step_deg=2): returns {'ok': bool, 'length': float, 'height': float}

Notes:
"""
from math import cos, sin, hypot, sqrt, radians, isfinite, acos, pi
from typing import List, Tuple, Dict


def evaluate_params(p: List[float], theta_step_deg: int = 2) -> Dict[str, float]:
	"""Evaluate 13-parameter vector and return ok, flat segment length, and step height.

	Args:
		p: list of 13 floats [link1..link11, node4_x, node4_y]
		theta_step_deg: step in degrees for crank rotation sweep (default 10)

	Returns:
		dict with keys {'ok': bool, 'length': float, 'height': float}
	"""
	if p is None or len(p) != 13:
		raise ValueError("Expected 13 parameters: 11 link lengths + node4_x + node4_y")
	if theta_step_deg <= 0 or theta_step_deg > 180:
		raise ValueError("theta_step_deg must be in (0, 180]")

	# Build theta vector (in radians)
	theta_vec = [radians(t) for t in range(0, 360, theta_step_deg) if t <= 350]
	end_traj, ok, diag = simulate_leg(p, theta_vec)
	metrics = compute_metrics(end_traj, diag)
	return {"ok": bool(ok), "length": float(metrics["length"]), "height": float(metrics["step_height"]), "spi_margin": float(metrics["spi_margin"]), "asi_margin": float(metrics["asi_margin"])}


def simulate_leg(p: List[float], theta_vec: List[float]) -> Tuple[List[Tuple[float, float]], bool, Dict]:
	if not theta_vec:
		theta_vec = [radians(t) for t in range(0, 360, 2) if t <= 350]

	link_lengths = p[0:11]
	# nodes[0..7] correspond to node1..node8
	nodes: List[List[float]] = [[float('nan'), float('nan')] for _ in range(8)]
	# node4 is fixed
	nodes[3] = [p[11], p[12]]
	# node1 is pivot at origin
	nodes[0] = [0.0, 0.0]
	prev_pts: List[List[float]] = [[float('nan'), float('nan')] for _ in range(8)]
	end_traj: List[Tuple[float, float]] = []
	ok = True

	# diagnostics
	min_h2 = float('inf')
	n5s: List[Tuple[float, float]] = []
	n6s: List[Tuple[float, float]] = []
	n7s: List[Tuple[float, float]] = []

	# cache link lengths for clarity
	link1, link2, link3 = link_lengths[0], link_lengths[1], link_lengths[2]
	link4, link5, link6 = link_lengths[3], link_lengths[4], link_lengths[5]
	link7, link8, link9 = link_lengths[6], link_lengths[7], link_lengths[8]
	link10, link11 = link_lengths[9], link_lengths[10]
	fixed_node4 = nodes[3]

	for theta in theta_vec:
		pivot_pos = nodes[0]
		# node2 from pivot via link1
		nodes[1] = [pivot_pos[0] + link1 * cos(theta), pivot_pos[1] + link1 * sin(theta)]

		# node3 from node2 & node4 via link2 & link3
		pts, h2 = circle_intersections(nodes[1], link2, fixed_node4, link3)
		if not pts:
			ok = False
			break
		min_h2 = min(min_h2, h2)
		nodes[2] = choose_point(pts, prev_pts[2], 3)

		# node5 from node3 & node4 via link4 & link5
		pts, h2 = circle_intersections(nodes[2], link4, fixed_node4, link5)
		if not pts:
			ok = False
			break
		min_h2 = min(min_h2, h2)
		nodes[4] = choose_point(pts, prev_pts[4], 5)

		# node6 from node2 & node4 via link7 & link6
		pts, h2 = circle_intersections(nodes[1], link7, fixed_node4, link6)
		if not pts:
			ok = False
			break
		min_h2 = min(min_h2, h2)
		nodes[5] = choose_point(pts, prev_pts[5], 6)

		# node7 from node6 & node5 via link8 & link9
		pts, h2 = circle_intersections(nodes[5], link8, nodes[4], link9)
		if not pts:
			ok = False
			break
		min_h2 = min(min_h2, h2)
		nodes[6] = choose_point(pts, prev_pts[6], 7)

		# node8 from node6 & node7 via link10 & link11 (foot)
		pts, h2 = circle_intersections(nodes[5], link10, nodes[6], link11)
		if not pts:
			ok = False
			break
		min_h2 = min(min_h2, h2)
		nodes[7] = choose_point(pts, prev_pts[7], 8)

		# feasibility: foot height must not exceed node6 or node7 heights
		ny8 = nodes[7][1]
		ny6 = nodes[5][1]
		ny7 = nodes[6][1]
		if (isfinite(ny6) and ny8 > ny6) or (isfinite(ny7) and ny8 > ny7):
			ok = False
			break

		prev_pts = [list(pt) for pt in nodes]
		end_traj.append((nodes[7][0], nodes[7][1]))
		n5s.append((nodes[4][0], nodes[4][1]))
		n6s.append((nodes[5][0], nodes[5][1]))
		n7s.append((nodes[6][0], nodes[6][1]))

	# trim trajectory if failed before finishing
	if not ok and end_traj:
		pass

	diag = {
		"spi_min_h2": (min_h2 if isfinite(min_h2) else 0.0),
		"n5s": n5s,
		"n6s": n6s,
		"n7s": n7s,
	}
	return end_traj, ok, diag


def circle_intersections(c1: List[float], r1: float, c2: List[float], r2: float) -> Tuple[List[Tuple[float, float]], float]:
	dx = c2[0] - c1[0]
	dy = c2[1] - c1[1]
	d = hypot(dx, dy)
	tol = 1e-9
	if d < tol:
		return [], 0.0
	if d > (r1 + r2) + tol or d < abs(r1 - r2) - tol:
		return [], 0.0
	a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
	h2 = r1 * r1 - a * a
	invd = 1.0 / d
	px = c1[0] + a * dx * invd
	py = c1[1] + a * dy * invd
	if h2 <= tol:
		p = (px, py)
		return [p, p], h2
	h = sqrt(h2)
	rx = -dy * (h * invd)
	ry = dx * (h * invd)
	return [(px + rx, py + ry), (px - rx, py - ry)], h2


def choose_point(pts: List[Tuple[float, float]], prev: List[float], idx: int) -> List[float]:
	if not pts:
		raise ValueError("No intersection points found for given radii/centers.")
	prev_is_nan = (
		prev is None or
		len(prev) != 2 or
		(not isfinite(prev[0])) or
		(not isfinite(prev[1]))
	)
	if prev_is_nan:
		if idx == 3:
			# prefer higher y
			return list(pts[0]) if pts[0][1] >= pts[1][1] else list(pts[1])
		elif idx == 5:
			# prefer smaller x
			return list(pts[0]) if pts[0][0] <= pts[1][0] else list(pts[1])
		elif idx in (6, 7, 8):
			# prefer lower y
			return list(pts[0]) if pts[0][1] <= pts[1][1] else list(pts[1])
		else:
			# default: prefer higher y
			return list(pts[0]) if pts[0][1] >= pts[1][1] else list(pts[1])
	# otherwise: choose the point closer to previous
	d1sq = (pts[0][0] - prev[0]) ** 2 + (pts[0][1] - prev[1]) ** 2
	d2sq = (pts[1][0] - prev[0]) ** 2 + (pts[1][1] - prev[1]) ** 2
	return list(pts[0]) if d1sq <= d2sq else list(pts[1])


def compute_metrics(end_traj: List[Tuple[float, float]], diag: Dict | None = None) -> Dict[str, float]:
	if not end_traj:
		# Even if empty, include stability fields
		return {
			"step_height": float('nan'),
			"step_length": float('nan'),
			"stable_pairs": 0.0,
			"length": 0.0,
			"mean_y": float('nan'),
			"std_y": float('nan'),
			"spi_margin": 0.0,
			"asi_margin": 0.0,
		}
	xs = [pt[0] for pt in end_traj]
	ys = [pt[1] for pt in end_traj]
	# height and length
	step_height = (max(ys) - min(ys)) if ys else float('nan')
	step_length = (max(xs) - min(xs)) if xs else float('nan')
	# flat segment 'length': longest contiguous segment where
	# (1) adjacent |dy| < thresh and (2) overall range (max_y-min_y) < thresh
	stab_thresh = 0.2
	maxspan = 0.0
	stable_pairs = 0
	n = len(ys)
	if n >= 2:
		s = 0
		while s < n - 1:
			min_y = ys[s]
			max_y = ys[s]
			e = s
			while e < n - 1:
				dy = abs(ys[e + 1] - ys[e])
				if dy < stab_thresh:
					# attempt to extend segment
					new_min = min(min_y, ys[e + 1])
					new_max = max(max_y, ys[e + 1])
					if (new_max - new_min) < stab_thresh:
						e += 1
						min_y = new_min
						max_y = new_max
						stable_pairs += 1
					else:
						break
				else:
					break
			span = abs(xs[e] - xs[s])
			if span > maxspan:
				maxspan = span
			s = e + 1
	mean_y = sum(ys) / len(ys)
	# std of y
	if len(ys) <= 1:
		std_y = 0.0
	else:
		mu = mean_y
		var = sum((y - mu) ** 2 for y in ys) / (len(ys) - 1)
		std_y = sqrt(var)
	# SPI margin: min h2 across intersections (larger is better)
	spi_margin = 0.0
	asi_margin = 0.0
	if diag:
		spi = diag.get("spi_min_h2")
		if spi is not None and isfinite(spi):
			spi_margin = float(spi)
		# ASI: node6 angle safety margin min over sweep
		n5s = diag.get("n5s") or []
		n6s = diag.get("n6s") or []
		n7s = diag.get("n7s") or []
		marg = float('inf')
		for (x5,y5),(x6,y6),(x7,y7) in zip(n5s,n6s,n7s):
			u = (x5 - x6, y5 - y6)
			v = (x7 - x6, y7 - y6)
			nu = hypot(u[0], u[1])
			nv = hypot(v[0], v[1])
			if nu <= 1e-12 or nv <= 1e-12:
				continue
			cosang = max(-1.0, min(1.0, (u[0]*v[0] + u[1]*v[1]) / (nu*nv)))
			ang = acos(cosang)
			# distance to 0 or pi
			dist = min(ang, pi - ang)
			if dist < marg:
				marg = dist
		if isfinite(marg):
			asi_margin = (0.0 if marg == float('inf') else marg)
	return {
		"step_height": step_height,
		"step_length": step_length,
		"stable_pairs": float(stable_pairs),
		"length": maxspan,
		"mean_y": mean_y,
		"std_y": std_y,
		"spi_margin": spi_margin,
		"asi_margin": asi_margin,
	}


if __name__ == "__main__":
	# Example baseline parameters from grid_search.m
	p0 = [15, 50, 41.5, 55.8, 40.1, 39.3, 61.9, 36.7, 39.4, 49, 65.7, -38, -7.8]
	res = evaluate_params(p0, theta_step_deg=2)
	print({k: (round(v, 6) if isinstance(v, float) else v) for k, v in res.items()})
