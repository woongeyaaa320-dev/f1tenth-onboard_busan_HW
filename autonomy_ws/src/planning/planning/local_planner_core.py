"""Geometry helpers for the scan-based F1TENTH local planner."""

import math

import numpy as np


QUINTIC_SMOOTHSTEP_MAX_SECOND_DERIVATIVE = 10.0 * math.sqrt(3.0) / 3.0


def adaptive_map_endpoint_threshold(
        clearances, base_threshold, registration_percentile,
        registration_margin, maximum_extra):
    """
    Estimate a scan-to-map wall rejection threshold from the current scan.

    Track walls normally account for most valid LiDAR endpoints, while real
    track obstacles are a minority.  A robust percentile therefore estimates
    the current registration residual without using map coordinates.  The
    bounded extra allowance prevents a badly localized scan from hiding every
    obstacle indefinitely.
    """
    values = np.asarray(clearances, dtype=float)
    values = values[np.isfinite(values) & (values >= 0.0)]
    base_threshold = max(0.0, float(base_threshold))
    maximum_extra = max(0.0, float(maximum_extra))
    if len(values) == 0:
        return base_threshold
    percentile = float(np.clip(registration_percentile, 0.0, 100.0))
    residual = float(np.percentile(values, percentile))
    adaptive = residual + max(0.0, float(registration_margin))
    return min(
        base_threshold + maximum_extra,
        max(base_threshold, adaptive))


def angle_difference(target, source):
    """Return the wrapped signed angle from source to target."""
    return math.atan2(math.sin(target - source), math.cos(target - source))


def ordered_candidate_offsets(offsets, locked_offset, obstacle_lateral):
    """Prefer a stable avoidance side without excluding a safe fallback side."""
    offsets = list(offsets)
    if locked_offset is not None:
        locked_sign = math.copysign(1.0, locked_offset)
        offsets = [value for value in offsets if abs(value) > 1e-3]
        offsets.sort(key=lambda value: (
            math.copysign(1.0, value) != locked_sign,
            abs(value - locked_offset),
            abs(value),
        ))
    elif obstacle_lateral >= 0.0:
        offsets.sort(key=lambda value: (value > 0.0, abs(value)))
    else:
        offsets.sort(key=lambda value: (value < 0.0, abs(value)))
    return offsets


def adaptive_candidate_offsets(
        obstacle_lateral, required_clearance, spacing, count,
        maximum_offset):
    """
    Generate map-independent Frenet pass candidates around an obstacle.

    The first candidate on either side just clears the measured obstacle and
    subsequent candidates add a small lateral margin.  Map collision checking
    remains responsible for rejecting candidates that leave the drivable
    corridor, so no track coordinates or preferred side are embedded here.
    """
    required_clearance = max(0.0, float(required_clearance))
    spacing = max(1e-3, float(spacing))
    count = max(1, int(count))
    maximum_offset = max(0.0, float(maximum_offset))
    obstacle_lateral = float(obstacle_lateral)

    values = [0.0]
    left_start = obstacle_lateral + required_clearance
    right_start = obstacle_lateral - required_clearance
    for index in range(count):
        left = left_start + index * spacing
        right = right_start - index * spacing
        if abs(left) <= maximum_offset + 1e-9:
            values.append(left)
        if abs(right) <= maximum_offset + 1e-9:
            values.append(right)

    # Preserve deterministic ordering while removing numerically identical
    # candidates (for example when required_clearance is zero).
    unique = []
    for value in values:
        if not any(abs(value - other) < 1e-6 for other in unique):
            unique.append(float(value))
    return unique


def minimum_quintic_transition_length(offset, maximum_curvature):
    """
    Approximate the minimum smoothstep length for a curvature constraint.

    A quintic Frenet shift has a known maximum second derivative.  This bound
    makes high-speed detours longer and smoother without using map-specific
    coordinates.  The caller still checks the resulting path against the map.
    """
    offset = abs(float(offset))
    maximum_curvature = max(1e-3, float(maximum_curvature))
    return math.sqrt(
        QUINTIC_SMOOTHSTEP_MAX_SECOND_DERIVATIVE
        * offset / maximum_curvature)


def speed_dependent_horizon(
        speed, reaction_time, deceleration, margin,
        minimum, maximum):
    """
    Return a bounded perception/planning horizon for the current speed.

    The distance is the reaction distance plus a constant-deceleration
    stopping distance and a geometric planning margin.  It therefore scales
    with vehicle dynamics instead of a particular map or obstacle position.
    """
    speed = max(0.0, float(speed))
    deceleration = max(0.1, float(deceleration))
    distance = (
        speed * max(0.0, float(reaction_time))
        + speed * speed / (2.0 * deceleration)
        + max(0.0, float(margin)))
    return max(float(minimum), min(distance, float(maximum)))


def path_curvature_percentile(points, percentile=90.0):
    """Estimate a robust absolute-curvature percentile of an XY polyline."""
    points = np.asarray(points, dtype=float)
    if len(points) < 4:
        return 0.0
    segments = np.diff(points, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    valid = lengths > 1e-5
    if np.count_nonzero(valid) < 3:
        return 0.0
    headings = np.unwrap(np.arctan2(segments[:, 1], segments[:, 0]))
    heading_delta = np.abs(np.diff(headings))
    arc_span = 0.5 * (lengths[:-1] + lengths[1:])
    valid_curvature = arc_span > 1e-5
    curvature = heading_delta[valid_curvature] / arc_span[valid_curvature]
    if len(curvature) == 0:
        return 0.0
    return float(np.percentile(curvature, np.clip(percentile, 0.0, 100.0)))


class ClosedPathGeometry:
    """Arc-length representation of a non-self-intersecting closed path."""

    def __init__(self, points):
        points = np.asarray(points, dtype=float)
        if len(points) > 1 and np.linalg.norm(points[0] - points[-1]) < 1e-6:
            points = points[:-1]
        if points.ndim != 2 or points.shape[1] != 2 or len(points) < 4:
            raise ValueError('A closed path requires at least four XY points')

        next_points = np.roll(points, -1, axis=0)
        segments = next_points - points
        lengths = np.linalg.norm(segments, axis=1)
        if np.any(lengths < 1e-5):
            raise ValueError('Closed path contains duplicate adjacent points')

        self.points = points
        self.segments = segments
        self.segment_lengths = lengths
        self.cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
        self.length = float(self.cumulative[-1])
        self.yaw = np.arctan2(segments[:, 1], segments[:, 0])
        self.segment_normals = np.column_stack(
            (-np.sin(self.yaw), np.cos(self.yaw)))
        unit_tangents = segments / lengths[:, None]
        vertex_tangents = unit_tangents + np.roll(unit_tangents, 1, axis=0)
        vertex_norms = np.linalg.norm(vertex_tangents, axis=1)
        degenerate = vertex_norms < 1e-6
        vertex_tangents[~degenerate] /= vertex_norms[~degenerate, None]
        vertex_tangents[degenerate] = unit_tangents[degenerate]
        self.normals = np.column_stack(
            (-vertex_tangents[:, 1], vertex_tangents[:, 0]))

    def project(self, point):
        """Project an XY point and return s, signed lateral offset and distance."""
        point = np.asarray(point, dtype=float)
        relative = point - self.points
        fractions = np.sum(relative * self.segments, axis=1) / (
            self.segment_lengths ** 2)
        fractions = np.clip(fractions, 0.0, 1.0)
        projections = self.points + fractions[:, None] * self.segments
        distances = np.linalg.norm(projections - point, axis=1)
        index = int(np.argmin(distances))
        projection = projections[index]
        lateral = float(np.dot(
            point - projection, self.segment_normals[index]))
        s_value = (
            self.cumulative[index]
            + float(fractions[index]) * self.segment_lengths[index])
        return float(s_value), lateral, float(distances[index])

    def circular_delta(self, values, center_s):
        """Return signed shortest arc distance from center_s to each s value."""
        values = np.asarray(values, dtype=float)
        return (values - center_s + 0.5 * self.length) % self.length - (
            0.5 * self.length)

    def offset_bump(self, center_s, offset, before_distance, after_distance):
        """
        Create a minimum-jerk lateral path offset around center_s.

        Quintic smoothstep has zero first and second derivative at both ends,
        avoiding the steering-rate discontinuity of a piecewise linear shift.
        """
        point_s = self.cumulative[:-1]
        delta = self.circular_delta(point_s, center_s)
        weights = np.zeros(len(self.points), dtype=float)

        before = (delta >= -before_distance) & (delta <= 0.0)
        after = (delta > 0.0) & (delta <= after_distance)
        before_progress = np.clip(
            (delta[before] + before_distance) / before_distance, 0.0, 1.0)
        after_progress = np.clip(
            delta[after] / after_distance, 0.0, 1.0)
        weights[before] = (
            10.0 * before_progress ** 3
            - 15.0 * before_progress ** 4
            + 6.0 * before_progress ** 5)
        weights[after] = 1.0 - (
            10.0 * after_progress ** 3
            - 15.0 * after_progress ** 4
            + 6.0 * after_progress ** 5)
        return self.points + self.normals * (offset * weights[:, None])

    def forward_distance(self, start_s, target_s):
        """Return positive wrapped distance from start_s to target_s."""
        return float((target_s - start_s) % self.length)


def sample_closed_path(points, spacing=0.05):
    """Densely sample every segment of a closed XY polyline."""
    points = np.asarray(points, dtype=float)
    samples = []
    for index in range(len(points)):
        start = points[index]
        end = points[(index + 1) % len(points)]
        length = float(np.linalg.norm(end - start))
        count = max(1, int(math.ceil(length / spacing)))
        fractions = np.arange(count, dtype=float) / count
        samples.extend(start + fractions[:, None] * (end - start))
    return np.asarray(samples)


def sample_path_window(points, geometry, start_s, distance, spacing=0.05):
    """Sample a forward arc window using the reference path parameterization."""
    points = np.asarray(points, dtype=float)
    sample_s = start_s + np.arange(
        0.0, distance + 0.5 * spacing, spacing)
    wrapped = np.mod(sample_s, geometry.length)
    closed_x = np.concatenate((points[:, 0], [points[0, 0]]))
    closed_y = np.concatenate((points[:, 1], [points[0, 1]]))
    return np.column_stack((
        np.interp(wrapped, geometry.cumulative, closed_x),
        np.interp(wrapped, geometry.cumulative, closed_y),
    ))


def cluster_ordered_points(indexed_points, max_gap, min_points, max_diameter):
    """Cluster ordered scan endpoints while preserving missing-beam breaks."""
    clusters = []
    current = []
    previous_index = None
    previous_point = None

    def finish_cluster():
        if len(current) < min_points:
            return
        array = np.asarray(current, dtype=float)
        diameter = float(np.max(np.ptp(array, axis=0)))
        if diameter <= max_diameter:
            clusters.append(array)

    for beam_index, x_value, y_value in indexed_points:
        point = np.asarray([x_value, y_value], dtype=float)
        separated = (
            previous_index is None
            or beam_index != previous_index + 1
            or np.linalg.norm(point - previous_point) > max_gap)
        if separated and current:
            finish_cluster()
            current = []
        current.append(point)
        previous_index = beam_index
        previous_point = point

    if current:
        finish_cluster()
    return clusters


def nearest_clustered_corridor_distance(clusters, half_width):
    """
    Return the nearest forward point from a real scan cluster in a corridor.

    AEB must not react to an isolated range sample that was intentionally
    rejected by the obstacle cluster filter.  The caller supplies clusters in
    the vehicle base frame, so this helper remains independent of any map or
    track geometry.
    """
    nearest = float('inf')
    half_width = max(0.0, float(half_width))
    for cluster in clusters:
        points = np.asarray(cluster, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2:
            continue
        in_corridor = (
            (points[:, 0] > 0.0)
            & (np.abs(points[:, 1]) < half_width))
        if np.any(in_corridor):
            nearest = min(nearest, float(np.min(points[in_corridor, 0])))
    return nearest


def update_tracked_obstacles(
        tracks, observations, now_seconds, next_id, match_distance,
        memory_seconds):
    """
    Update obstacle tracks with one-to-one scan associations.

    A new scan cluster starts with one hit. Repeated spatially consistent
    observations increase ``hits``; missed tracks remain only for the short
    configured memory window. This keeps one noisy scan from becoming a
    planned obstacle while preserving real obstacles across sparse scans.
    """
    tracks = [
        track for track in tracks
        if now_seconds - float(track['last_seen']) <= memory_seconds
    ]
    matched_indices = set()
    original_track_count = len(tracks)

    for center, radius, s_value, lateral in observations:
        center = np.asarray(center, dtype=float)
        best_index = None
        best_distance = float(match_distance)
        for index in range(original_track_count):
            if index in matched_indices:
                continue
            distance = float(np.linalg.norm(tracks[index]['center'] - center))
            if distance < best_distance:
                best_index = index
                best_distance = distance

        if best_index is None:
            tracks.append({
                'id': int(next_id),
                'center': center.copy(),
                'radius': float(radius),
                's': float(s_value),
                'lateral': float(lateral),
                'hits': 1,
                'last_seen': float(now_seconds),
            })
            next_id += 1
            continue

        matched_indices.add(best_index)
        track = tracks[best_index]
        # Competition obstacles are static. A running mean prevents the
        # visible LiDAR surface from moving the estimated center and Frenet
        # offset as the car changes viewing angle around the same object.
        previous_hits = max(1, int(track.get('hits', 1)))
        observation_weight = 1.0 / (previous_hits + 1.0)
        track['center'] = (
            (1.0 - observation_weight) * track['center']
            + observation_weight * center)
        track['radius'] = max(float(track['radius']), float(radius))
        track['s'] = (
            (1.0 - observation_weight) * float(track['s'])
            + observation_weight * float(s_value))
        track['lateral'] = (
            (1.0 - observation_weight) * float(track['lateral'])
            + observation_weight * float(lateral))
        track['hits'] = previous_hits + 1
        track['last_seen'] = float(now_seconds)

    return tracks, next_id
