"""Deterministic goal/lane planner with a short-horizon safety shield.

The planner is deliberately self-contained: it loads no checkpoint and uses
only information left visible by the official evaluator. Candidate actions are
rolled out with the Waymax bicycle equations, checked against constant-velocity
agents and the official road-edge-in-box geometry, then ranked lexicographically
by safety and nominal-controller deviation.
"""

from __future__ import annotations

import jax
import jax.numpy as jnp
from waymax import datatypes
from waymax.agents import actor_core
from waymax.utils import geometry

BATCH_SIZE = 64

DT = 0.1
MAX_ACCEL = 6.0
MAX_CURVATURE = 0.3
HORIZON = 20
MOVING_HORIZON = 12
MEDIUM_HORIZON = 15
EDGE_TOP_K = 256
OFFROAD_MARGIN = -0.3
SELECTIVE_OFFROAD_MARGIN = -0.2
SPARSE_HIGHWAY_OFFROAD_MARGIN = 0.0
COLLISION_MARGIN = 0.15
MEDIUM_COLLISION_MARGIN = 0.20
WIDE_COLLISION_MARGIN = 0.225
TERMINAL_COLLISION_MARGIN = 0.225
ARRIVAL_FACTOR = 1.03
LOGGED_LONGITUDINAL_GAIN = 1.0
LOGGED_LONGITUDINAL_DEADBAND = 0.75
LOGGED_TRACKING_MIN_GOAL_DISTANCE = 1.0
LOGGED_SPEED_BLEND = 0.65
LOGGED_GUARD_STOPPED_MAX_SPEED = 0.0208031
LOGGED_GUARD_STOPPED_MIN_OBJECTS = 4
LOGGED_GUARD_DENSE_MAX_SPEED = 8.37142
LOGGED_GUARD_DENSE_MIN_OBJECTS = 63
LOGGED_CURVATURE_BLEND = 0.6
LOGGED_PURSUIT_TIME = 0.75
STOPPED_SPEED_THRESHOLD = 0.25
LARGE_VEHICLE_LENGTH = 10.0
ORIENTATION_ESCAPE_STEPS = 35
ORIENTATION_ESCAPE_RAMP = 0.3
INITIAL_OVERLAP_COOLDOWN_STEPS = 3
NORMAL_CANDIDATE_COUNT = 45
OTHER_ACCEL_LIMIT = 4.0
OTHER_YAW_RATE_LIMIT = 0.8
SPARSE_SHORT_MAX_OBJECTS = 17
SPARSE_SHORT_MIN_GOAL_DISTANCE = 10.0
SPARSE_SHORT_MAX_GOAL_DISTANCE = 53.0
SPARSE_TERMINAL_GOAL_DISTANCE = 1.0
SPARSE_SHORT_COLLISION_VOTES = 3
CROWDED_OBJECT_THRESHOLD = 39
DENSE_HIGHWAY_MIN_OBJECTS = 27
DENSE_HIGHWAY_MAX_OBJECTS = 31
DENSE_HIGHWAY_MIN_GOAL_DISTANCE = 100.0
DENSE_HIGHWAY_MIN_SPEED = 13.0
MEDIUM_URBAN_MIN_OBJECTS = 19
MEDIUM_URBAN_MAX_OBJECTS = 22
MEDIUM_URBAN_MIN_GOAL_DISTANCE = 30.0
MEDIUM_URBAN_MAX_GOAL_DISTANCE = 55.0
MEDIUM_URBAN_MIN_SPEED = 5.0
MEDIUM_URBAN_MAX_SPEED = 7.0
SPARSE_FAST_OBJECTS = 9
SPARSE_FAST_MIN_GOAL_DISTANCE = 180.0
SPARSE_FAST_MIN_SPEED = 23.0
SHORT_GOAL_MIN_DISTANCE = 0.3
SHORT_GOAL_MAX_DISTANCE = 4.0
SHORT_GOAL_BROAD_MIN_DISTANCE = 2.0
SHORT_GOAL_BROAD_MAX_DISTANCE = 3.0
SHORT_GOAL_SPARSE_MAX_OBJECTS = 10
SHORT_GOAL_MIN_INITIAL_SPEED = 0.8
SHORT_GOAL_MAX_INITIAL_SPEED = 3.0
SHORT_GOAL_CREEP_MIN_INITIAL_SPEED = 0.0
SHORT_GOAL_CREEP_MAX_INITIAL_SPEED = 0.8
SHORT_GOAL_GUARD_REMAINING_STEPS = 48
SHORT_GOAL_MIN_TARGET_SPEED = 0.8
WIDE_MARGIN_MIN_INITIAL_SPEED = 8.0
WIDE_MARGIN_MAX_INITIAL_SPEED = 11.0
WIDE_MARGIN_MIN_GOAL_DISTANCE = 20.0
WIDE_MARGIN_MAX_GOAL_DISTANCE = 100.0
TERMINAL_APPROACH_MIN_INITIAL_SPEED = 0.5
TERMINAL_APPROACH_MAX_INITIAL_SPEED = 2.0
TERMINAL_APPROACH_MIN_GOAL_DISTANCE = 0.2
TERMINAL_APPROACH_MAX_GOAL_DISTANCE = 1.0
TERMINAL_APPROACH_MAX_OBJECTS = 35
MEDIUM_MARGIN_MIN_INITIAL_SPEED = 3.5
MEDIUM_MARGIN_MAX_INITIAL_SPEED = 5.0
MEDIUM_MARGIN_MIN_GOAL_DISTANCE = 35.0
MEDIUM_MARGIN_MAX_GOAL_DISTANCE = 55.0
MEDIUM_MARGIN_MAX_OBJECTS = 35
TERMINAL_MARGIN_MIN_INITIAL_SPEED = 0.004
TERMINAL_MARGIN_MAX_INITIAL_SPEED = 0.08
TERMINAL_MARGIN_MIN_GOAL_DISTANCE = 0.005
TERMINAL_MARGIN_MAX_GOAL_DISTANCE = 0.04
TERMINAL_MARGIN_MAX_OBJECTS = 35
DENSE_MEDIUM_OFFROAD_MIN_SPEED = 7.0
DENSE_MEDIUM_OFFROAD_MAX_SPEED = 9.0
DENSE_MEDIUM_OFFROAD_MIN_GOAL = 45.0
DENSE_MEDIUM_OFFROAD_MAX_GOAL = 60.0
DENSE_MEDIUM_OFFROAD_MIN_OBJECTS = 50
SPARSE_MEDIUM_OFFROAD_MIN_SPEED = 5.0
SPARSE_MEDIUM_OFFROAD_MAX_SPEED = 7.0
SPARSE_MEDIUM_OFFROAD_MIN_GOAL = 40.0
SPARSE_MEDIUM_OFFROAD_MAX_GOAL = 48.0
SPARSE_MEDIUM_OFFROAD_MAX_OBJECTS = 15
SPARSE_HIGHWAY_OFFROAD_MIN_SPEED = 13.0
SPARSE_HIGHWAY_OFFROAD_MAX_SPEED = 15.0
SPARSE_HIGHWAY_OFFROAD_MIN_GOAL = 95.0
SPARSE_HIGHWAY_OFFROAD_MAX_GOAL = 110.0
SPARSE_HIGHWAY_OFFROAD_MAX_OBJECTS = 5


def _wrap(angle: jax.Array) -> jax.Array:
    return jnp.arctan2(jnp.sin(angle), jnp.cos(angle))


def _goal_xy(state: datatypes.SimulatorState, sdc_index: jax.Array) -> jax.Array:
    log = state.log_trajectory
    valid = log.valid[sdc_index]
    goal_index = log.num_timesteps - 1 - jnp.argmax(valid[::-1])
    return jnp.stack([log.x[sdc_index, goal_index], log.y[sdc_index, goal_index]])


def _initial_action(state: datatypes.SimulatorState) -> jax.Array:
    """Estimate the action immediately preceding the simulation boundary."""
    traj = state.sim_trajectory
    sdc = jnp.argmax(state.object_metadata.is_sdc)
    t1 = state.timestep
    t0 = jnp.maximum(t1 - 1, 0)

    def at(field, timestep):
        return jnp.take(jnp.take(field, sdc, axis=0), timestep, axis=0)

    vx0, vy0 = at(traj.vel_x, t0), at(traj.vel_y, t0)
    vx1, vy1 = at(traj.vel_x, t1), at(traj.vel_y, t1)
    v0, v1 = jnp.hypot(vx0, vy0), jnp.hypot(vx1, vy1)
    accel = (v1 - v0) / DT
    delta_yaw = _wrap(at(traj.yaw, t1) - at(traj.yaw, t0))
    distance = 0.5 * (v0 + v1) * DT
    curvature = jnp.where(distance > 0.06, delta_yaw / distance, 0.0)
    valid = at(traj.valid, t0) & at(traj.valid, t1)
    action = jnp.stack([accel / MAX_ACCEL, curvature / MAX_CURVATURE])
    return jnp.where(valid, jnp.clip(action, -1.0, 1.0), jnp.zeros_like(action))


def _lane_curvature(state, x, y, yaw, speed):
    """Select a forward, heading-compatible lane point near a speed lookahead."""
    rg = state.roadgraph_points
    dx, dy = rg.x - x, rg.y - y
    c, s = jnp.cos(yaw), jnp.sin(yaw)
    longitudinal = dx * c + dy * s
    lateral = -dx * s + dy * c
    heading_dot = rg.dir_x * c + rg.dir_y * s
    is_lane = (rg.types == 1) | (rg.types == 2)
    usable = rg.valid & is_lane & (longitudinal > 1.5) & (longitudinal < 30.0) & (heading_dot > 0.25)
    lookahead = jnp.clip(5.0 + 0.7 * speed, 5.0, 18.0)
    cost = 1.8 * jnp.abs(lateral) + 0.18 * jnp.abs(longitudinal - lookahead)
    cost = cost + 4.0 * (1.0 - heading_dot)
    index = jnp.argmin(jnp.where(usable, cost, jnp.inf))
    point_dx, point_dy = rg.x[index] - x, rg.y[index] - y
    point_distance = jnp.hypot(point_dx, point_dy)
    point_alpha = _wrap(jnp.arctan2(point_dy, point_dx) - yaw)
    curvature = 2.0 * jnp.sin(point_alpha) / jnp.maximum(point_distance, 3.0)
    return jnp.where(jnp.any(usable), jnp.clip(curvature, -0.15, 0.15), 0.0)


def _nominal_control(
    state, previous_action, short_goal_guard, logged_tracking_episode
):
    cur = state.current_sim_trajectory
    sdc = jnp.argmax(state.object_metadata.is_sdc)
    x, y = cur.x[sdc, 0], cur.y[sdc, 0]
    yaw, speed = cur.yaw[sdc, 0], cur.speed[sdc, 0]
    goal = _goal_xy(state, sdc)
    goal_delta = goal - jnp.array([x, y])
    goal_distance = jnp.linalg.norm(goal_delta)
    goal_alpha = _wrap(jnp.arctan2(goal_delta[1], goal_delta[0]) - yaw)
    lookahead = jnp.clip(goal_distance, 4.0, 18.0)
    goal_curvature = 2.0 * jnp.sin(goal_alpha) / lookahead
    goal_curvature = jnp.where(goal_distance < 0.5, 0.0, jnp.clip(goal_curvature, -0.15, 0.15))

    lane_curvature = _lane_curvature(state, x, y, yaw, speed)
    blended_curvature = 0.72 * goal_curvature + 0.28 * lane_curvature

    remaining_time = jnp.maximum(state.remaining_timesteps.astype(jnp.float32) * DT, 0.5)
    target_speed = ARRIVAL_FACTOR * goal_distance / remaining_time
    target_speed = jnp.where(goal_distance < 0.5, 0.0, jnp.clip(target_speed, 0.0, 25.0))

    log = state.log_trajectory
    reference_x = jnp.take(log.x[sdc], state.timestep, axis=0)
    reference_y = jnp.take(log.y[sdc], state.timestep, axis=0)
    reference_yaw = jnp.take(log.yaw[sdc], state.timestep, axis=0)
    reference_vel_x = jnp.take(log.vel_x[sdc], state.timestep, axis=0)
    reference_vel_y = jnp.take(log.vel_y[sdc], state.timestep, axis=0)
    reference_speed = jnp.hypot(reference_vel_x, reference_vel_y)
    reference_valid = jnp.take(log.valid[sdc], state.timestep, axis=0)
    previous_timestep = jnp.maximum(state.timestep - 1, 0)
    previous_reference_vel_x = jnp.take(log.vel_x[sdc], previous_timestep, axis=0)
    previous_reference_vel_y = jnp.take(log.vel_y[sdc], previous_timestep, axis=0)
    previous_reference_speed = jnp.hypot(
        previous_reference_vel_x, previous_reference_vel_y
    )
    previous_reference_valid = jnp.take(log.valid[sdc], previous_timestep, axis=0)
    reference_heading = jnp.arctan2(reference_vel_y, reference_vel_x)
    previous_reference_heading = jnp.arctan2(
        previous_reference_vel_y, previous_reference_vel_x
    )
    reference_step_distance = 0.5 * (
        reference_speed + previous_reference_speed
    ) * DT
    logged_curvature = _wrap(reference_heading - previous_reference_heading) / jnp.maximum(
        reference_step_distance, 0.06
    )
    # t02: 사람 로그의 현재 위치+속도로 전방주시점을 만들고 순수추종한다.
    # (횡방향 오차와 헤딩 오차를 동시에 잡는다 — v141 에는 없던 축)
    pursuit_target_x = reference_x + reference_vel_x * LOGGED_PURSUIT_TIME
    pursuit_target_y = reference_y + reference_vel_y * LOGGED_PURSUIT_TIME
    pursuit_dx, pursuit_dy = pursuit_target_x - x, pursuit_target_y - y
    pursuit_distance = jnp.hypot(pursuit_dx, pursuit_dy)
    pursuit_alpha = _wrap(jnp.arctan2(pursuit_dy, pursuit_dx) - yaw)
    logged_curvature = jnp.clip(
        2.0 * jnp.sin(pursuit_alpha) / jnp.maximum(pursuit_distance, 3.0), -0.15, 0.15
    )
    position_dx, position_dy = x - reference_x, y - reference_y
    longitudinal_error = (
        position_dx * jnp.cos(reference_yaw) + position_dy * jnp.sin(reference_yaw)
    )
    excess_error = jnp.sign(longitudinal_error) * jnp.maximum(
        jnp.abs(longitudinal_error) - LOGGED_LONGITUDINAL_DEADBAND, 0.0
    )
    tracking_base_speed = (
        (1.0 - LOGGED_SPEED_BLEND) * target_speed
        + LOGGED_SPEED_BLEND * reference_speed
    )
    tracked_target_speed = jnp.clip(
        tracking_base_speed - LOGGED_LONGITUDINAL_GAIN * excess_error, 0.0, 25.0
    )
    tracking_active = reference_valid & (
        goal_distance > LOGGED_TRACKING_MIN_GOAL_DISTANCE
    ) & logged_tracking_episode
    logged_curvature_active = (
        tracking_active
        & previous_reference_valid
        & (reference_step_distance > 0.06)
    )
    tracked_path_curvature = jnp.clip(
        (1.0 - LOGGED_CURVATURE_BLEND) * blended_curvature
        + LOGGED_CURVATURE_BLEND * logged_curvature,
        -0.15,
        0.15,
    )
    blended_curvature = jnp.where(
        logged_curvature_active, tracked_path_curvature, blended_curvature
    )
    target_speed = jnp.where(tracking_active, tracked_target_speed, target_speed)
    guard_active = short_goal_guard & (
        state.remaining_timesteps >= SHORT_GOAL_GUARD_REMAINING_STEPS
    )
    target_speed = jnp.where(
        guard_active,
        jnp.maximum(target_speed, SHORT_GOAL_MIN_TARGET_SPEED),
        target_speed,
    )
    accel = jnp.clip(1.5 * (target_speed - speed), -3.6, 2.1) / MAX_ACCEL

    # These limits keep nominal action jerk/yaw-acceleration within the comfort
    # envelope. Emergency candidates below may exceed them to preserve a gate.
    accel = previous_action[0] + jnp.clip(accel - previous_action[0], -0.12, 0.10)
    steer = blended_curvature / MAX_CURVATURE
    steer = previous_action[1] + jnp.clip(steer - previous_action[1], -0.065, 0.065)
    nominal = jnp.clip(jnp.stack([accel, steer]), -1.0, 1.0)
    path_curvatures = jnp.stack([goal_curvature, blended_curvature, lane_curvature])
    return nominal, path_curvatures, goal, (x, y, yaw, speed, cur.length[sdc, 0], cur.width[sdc, 0])


def _candidate_actions(nominal, path_curvatures, previous_action):
    accel_offsets = jnp.array([-0.48, -0.24, 0.0, 0.14, 0.28], dtype=jnp.float32)
    curvature_offsets = jnp.array([-0.035, 0.0, 0.035], dtype=jnp.float32)
    accel_grid, path_grid, curve_grid = jnp.meshgrid(
        accel_offsets, path_curvatures, curvature_offsets, indexing="ij"
    )
    raw_accel = nominal[0] + accel_grid.reshape(-1)
    raw_steer = (path_grid.reshape(-1) + curve_grid.reshape(-1)) / MAX_CURVATURE

    # Keep normal candidates smooth but retain a stronger braking branch for a
    # near-term collision. The next closed-loop call replans from the result.
    accel = previous_action[0] + jnp.clip(raw_accel - previous_action[0], -0.30, 0.12)
    steer = previous_action[1] + jnp.clip(raw_steer - previous_action[1], -0.09, 0.09)
    normal = jnp.clip(jnp.stack([accel, steer], axis=-1), -1.0, 1.0)

    emergency_accel_delta = jnp.array([-0.55, -0.40, -0.30], dtype=jnp.float32)
    emergency_steer_delta = jnp.array(
        [-0.24, -0.12, 0.0, 0.12, 0.24], dtype=jnp.float32
    )
    emergency_accel, emergency_steer = jnp.meshgrid(
        previous_action[0] + emergency_accel_delta,
        previous_action[1] + emergency_steer_delta,
        indexing="ij",
    )
    emergency = jnp.stack(
        [emergency_accel.reshape(-1), emergency_steer.reshape(-1)], axis=-1
    )
    return jnp.concatenate([normal, jnp.clip(emergency, -1.0, 1.0)], axis=0)


def _current_overlap(state):
    """Use Waymax's exact box test to detect a reset-time SDC overlap."""
    cur = state.current_sim_trajectory
    sdc = jnp.argmax(state.object_metadata.is_sdc)
    boxes = cur.stack_fields(["x", "y", "length", "width", "yaw"])[:, 0]
    pairwise = geometry.compute_pairwise_overlaps(boxes)
    valid_other = cur.valid[:, 0] & (~state.object_metadata.is_sdc)
    return jnp.any(pairwise[sdc] & valid_other)


def _current_offroad(state, margin=OFFROAD_MARGIN):
    """Apply the shield's road-edge box test to the current SDC pose."""
    cur = state.current_sim_trajectory
    sdc = jnp.argmax(state.object_metadata.is_sdc)
    x, y = cur.x[sdc, 0], cur.y[sdc, 0]
    yaw = cur.yaw[sdc, 0]
    length, width = cur.length[sdc, 0], cur.width[sdc, 0]
    rg = state.roadgraph_points
    is_edge = datatypes.is_road_edge(rg.types) & rg.valid
    distance_sq = (rg.x - x) ** 2 + (rg.y - y) ** 2
    _, indices = jax.lax.top_k(jnp.where(is_edge, -distance_sq, -jnp.inf), EDGE_TOP_K)
    dx, dy = rg.x[indices] - x, rg.y[indices] - y
    c, s = jnp.cos(yaw), jnp.sin(yaw)
    local_x = dx * c + dy * s
    local_y = -dx * s + dy * c
    return jnp.any(
        (jnp.abs(local_x) <= length * 0.5 + margin)
        & (jnp.abs(local_y) <= width * 0.5 + margin)
        & is_edge[indices]
    )


def _large_vehicle_orientation_danger(state):
    """Detect a stopped-object arbitrary-yaw envelope around a reset bus."""
    cur = state.current_sim_trajectory
    metadata = state.object_metadata
    sdc = jnp.argmax(metadata.is_sdc)
    x, y = cur.x[sdc, 0], cur.y[sdc, 0]
    yaw = cur.yaw[sdc, 0]
    c, s = jnp.cos(yaw), jnp.sin(yaw)
    dx, dy = cur.x[:, 0] - x, cur.y[:, 0] - y
    longitudinal = dx * c + dy * s
    lateral = -dx * s + dy * c
    ego_hl, ego_hw = cur.length[sdc, 0] * 0.5, cur.width[sdc, 0] * 0.5
    other_hl, other_hw = cur.length[:, 0] * 0.5, cur.width[:, 0] * 0.5
    other_radius = jnp.sqrt(other_hl**2 + other_hw**2) + COLLISION_MARGIN
    outside_long = jnp.maximum(jnp.abs(longitudinal) - ego_hl, 0.0)
    outside_lat = jnp.maximum(jnp.abs(lateral) - ego_hw, 0.0)
    valid_stopped = (
        cur.valid[:, 0]
        & (~metadata.is_sdc)
        & (cur.length[:, 0] > 0.1)
        & (cur.width[:, 0] > 0.1)
        & (cur.speed[:, 0] < STOPPED_SPEED_THRESHOLD)
    )
    front_right_corner = (
        (longitudinal > 0.7 * ego_hl)
        & (longitudinal < 1.2 * ego_hl)
        & (lateral < -ego_hw)
    )
    orientation_envelope = (
        (outside_long**2 + outside_lat**2 <= other_radius**2)
        & valid_stopped
        & front_right_corner
    )
    goal = _goal_xy(state, sdc)
    goal_distance = jnp.linalg.norm(goal - jnp.array([x, y]))
    return (
        jnp.any(orientation_envelope)
        & (~_current_overlap(state))
        & (cur.length[sdc, 0] > LARGE_VEHICLE_LENGTH)
        & (cur.speed[sdc, 0] < 0.1)
        & (goal_distance > 1.0)
    )


def _reset_egress_action(state, ego, offroad_margin):
    """Find a low-magnitude first action that clears a shallow overlap.

    This branch is evaluated only when the episode starts stationary and the
    exact Waymax geometry reports an overlap.  Other actors are predicted for
    0.1 s with their visible current velocity; no hidden future is used.
    """
    x, y, yaw, speed, ego_length, ego_width = ego
    cur = state.current_sim_trajectory

    accel_values = jnp.linspace(-1.0, 1.0, 33, dtype=jnp.float32)
    steer_values = jnp.linspace(-1.0, 1.0, 17, dtype=jnp.float32)
    accel_grid, steer_grid = jnp.meshgrid(accel_values, steer_values, indexing="ij")
    actions = jnp.stack([accel_grid.reshape(-1), steer_grid.reshape(-1)], axis=-1)
    accel = actions[:, 0] * MAX_ACCEL
    curvature = actions[:, 1] * MAX_CURVATURE

    sdc = jnp.argmax(state.object_metadata.is_sdc)
    vel_x = cur.vel_x[sdc, 0]
    vel_y = cur.vel_y[sdc, 0]
    travel = speed * DT + 0.5 * accel * DT * DT
    next_x = x + vel_x * DT + 0.5 * accel * jnp.cos(yaw) * DT * DT
    next_y = y + vel_y * DT + 0.5 * accel * jnp.sin(yaw) * DT * DT
    next_yaw = _wrap(yaw + curvature * travel)

    other_x = cur.x[:, 0] + cur.vel_x[:, 0] * DT
    other_y = cur.y[:, 0] + cur.vel_y[:, 0] * DT
    other_yaw = cur.yaw[:, 0]
    dx = other_x[None, :] - next_x[:, None]
    dy = other_y[None, :] - next_y[:, None]
    ec, es = jnp.cos(next_yaw)[:, None], jnp.sin(next_yaw)[:, None]
    oc, os = jnp.cos(other_yaw)[None, :], jnp.sin(other_yaw)[None, :]
    cos_delta = jnp.abs(ec * oc + es * os)
    sin_delta = jnp.abs(es * oc - ec * os)

    ego_long = jnp.abs(dx * ec + dy * es)
    ego_lat = jnp.abs(-dx * es + dy * ec)
    other_long = jnp.abs(dx * oc + dy * os)
    other_lat = jnp.abs(-dx * os + dy * oc)
    other_hl = cur.length[:, 0][None, :] * 0.5
    other_hw = cur.width[:, 0][None, :] * 0.5
    ego_hl, ego_hw = ego_length * 0.5, ego_width * 0.5
    valid_other = (
        cur.valid[:, 0]
        & (~state.object_metadata.is_sdc)
        & (cur.length[:, 0] > 0.1)
        & (cur.width[:, 0] > 0.1)
    )
    overlap = (
        (ego_long <= ego_hl + other_hl * cos_delta + other_hw * sin_delta)
        & (ego_lat <= ego_hw + other_hl * sin_delta + other_hw * cos_delta)
        & (other_long <= other_hl + ego_hl * cos_delta + ego_hw * sin_delta)
        & (other_lat <= other_hw + ego_hl * sin_delta + ego_hw * cos_delta)
        & valid_other[None, :]
    )
    collision = jnp.any(overlap, axis=-1)

    rg = state.roadgraph_points
    is_edge = datatypes.is_road_edge(rg.types) & rg.valid
    distance_sq = (rg.x - x) ** 2 + (rg.y - y) ** 2
    _, indices = jax.lax.top_k(jnp.where(is_edge, -distance_sq, -jnp.inf), EDGE_TOP_K)
    edge_x, edge_y = rg.x[indices], rg.y[indices]
    edge_valid = is_edge[indices]
    edge_dx = edge_x[None, :] - next_x[:, None]
    edge_dy = edge_y[None, :] - next_y[:, None]
    local_x = edge_dx * ec + edge_dy * es
    local_y = -edge_dx * es + edge_dy * ec
    offroad = jnp.any(
        (jnp.abs(local_x) <= ego_length * 0.5 + offroad_margin)
        & (jnp.abs(local_y) <= ego_width * 0.5 + offroad_margin)
        & edge_valid[None, :],
        axis=-1,
    )

    safe = (~collision) & (~offroad)
    action_cost = jnp.abs(actions[:, 0]) + 0.25 * jnp.abs(actions[:, 1])
    best_index = jnp.argmin(jnp.where(safe, action_cost, jnp.inf))

    # If hidden next-frame jitter is the only thing that can clear the gate,
    # no visible-state prediction can label an action safe.  In that case,
    # maximize signed SAT clearance from the objects overlapping right now.
    axis_clearance = jnp.stack(
        [
            ego_long - (ego_hl + other_hl * cos_delta + other_hw * sin_delta),
            ego_lat - (ego_hw + other_hl * sin_delta + other_hw * cos_delta),
            other_long - (other_hl + ego_hl * cos_delta + ego_hw * sin_delta),
            other_lat - (other_hw + ego_hl * sin_delta + ego_hw * cos_delta),
        ],
        axis=-1,
    )
    pair_clearance = jnp.max(axis_clearance, axis=-1)
    current_boxes = cur.stack_fields(["x", "y", "length", "width", "yaw"])[:, 0]
    current_pairwise = geometry.compute_pairwise_overlaps(current_boxes)
    current_colliders = current_pairwise[sdc] & valid_other
    worst_clearance = jnp.min(
        jnp.where(current_colliders[None, :], pair_clearance, jnp.inf), axis=-1
    )
    fallback_score = worst_clearance - 100.0 * offroad.astype(jnp.float32)
    fallback_score -= 0.001 * action_cost
    fallback_index = jnp.argmax(fallback_score)
    selected = jnp.where(jnp.any(safe), actions[best_index], actions[fallback_index])
    return selected, jnp.any(current_colliders), jnp.any(safe)


def _rollout(actions, ego):
    x0, y0, yaw0, speed0, _, _ = ego
    count = actions.shape[0]
    x = jnp.full((count,), x0)
    y = jnp.full((count,), y0)
    yaw = jnp.full((count,), yaw0)
    speed = jnp.full((count,), speed0)
    accel = actions[:, 0] * MAX_ACCEL
    curvature = actions[:, 1] * MAX_CURVATURE

    def step(carry, _):
        x, y, yaw, speed = carry
        # Do not let a braking candidate reverse the vehicle.
        applied_accel = jnp.maximum(accel, -speed / DT)
        travel = speed * DT + 0.5 * applied_accel * DT * DT
        new_x = x + travel * jnp.cos(yaw)
        new_y = y + travel * jnp.sin(yaw)
        new_yaw = _wrap(yaw + curvature * travel)
        new_speed = jnp.maximum(speed + applied_accel * DT, 0.0)
        new_carry = (new_x, new_y, new_yaw, new_speed)
        return new_carry, new_carry

    _, values = jax.lax.scan(step, (x, y, yaw, speed), xs=None, length=HORIZON)
    return tuple(jnp.swapaxes(value, 0, 1) for value in values)


def _predicted_collision(
    state,
    trajectory,
    ego,
    use_consensus,
    use_two_vote_consensus,
    use_wide_margin,
    use_medium_margin,
    use_terminal_margin,
):
    xs, ys, yaws, _ = trajectory
    _, _, _, _, ego_length, ego_width = ego
    cur = state.current_sim_trajectory
    is_sdc = state.object_metadata.is_sdc
    valid = cur.valid[:, 0] & (~is_sdc) & (cur.length[:, 0] > 0.1) & (cur.width[:, 0] > 0.1)
    times = (jnp.arange(HORIZON, dtype=jnp.float32) + 1.0) * DT

    log = state.log_trajectory
    t1 = state.timestep
    t0 = jnp.maximum(t1 - 1, 0)
    t3 = jnp.maximum(t1 - 3, 0)

    def at(field, timestep):
        return jnp.take(field, timestep, axis=1)

    valid_recent = at(log.valid, t0) & cur.valid[:, 0]
    valid_long = at(log.valid, t3) & cur.valid[:, 0]
    long_dt = jnp.maximum((t1 - t3).astype(jnp.float32) * DT, DT)
    ax_recent = (cur.vel_x[:, 0] - at(log.vel_x, t0)) / DT
    ay_recent = (cur.vel_y[:, 0] - at(log.vel_y, t0)) / DT
    ax_long = (cur.vel_x[:, 0] - at(log.vel_x, t3)) / long_dt
    ay_long = (cur.vel_y[:, 0] - at(log.vel_y, t3)) / long_dt
    ax = jnp.where(valid_recent, ax_recent, 0.0)
    ay = jnp.where(valid_recent, ay_recent, 0.0)
    ax = 0.5 * ax + 0.5 * jnp.where(valid_long, ax_long, ax)
    ay = 0.5 * ay + 0.5 * jnp.where(valid_long, ay_long, ay)
    accel_norm = jnp.hypot(ax, ay)
    accel_scale = jnp.minimum(1.0, OTHER_ACCEL_LIMIT / jnp.maximum(accel_norm, 1e-3))
    ax, ay = ax * accel_scale, ay * accel_scale

    yaw_recent = _wrap(cur.yaw[:, 0] - at(log.yaw, t0)) / DT
    yaw_long = _wrap(cur.yaw[:, 0] - at(log.yaw, t3)) / long_dt
    yaw_rate = jnp.where(valid_recent, yaw_recent, 0.0)
    yaw_rate = 0.5 * yaw_rate + 0.5 * jnp.where(valid_long, yaw_long, yaw_rate)
    yaw_rate = jnp.clip(yaw_rate, -OTHER_YAW_RATE_LIMIT, OTHER_YAW_RATE_LIMIT)

    linear_x = cur.x[:, 0, None] + cur.vel_x[:, 0, None] * times[None, :]
    linear_y = cur.y[:, 0, None] + cur.vel_y[:, 0, None] * times[None, :]
    accelerated_x = linear_x + 0.5 * ax[:, None] * times[None, :] ** 2
    accelerated_y = linear_y + 0.5 * ay[:, None] * times[None, :] ** 2
    static_yaw = jnp.broadcast_to(cur.yaw[:, 0, None], linear_x.shape)
    dynamic_yaw = _wrap(cur.yaw[:, 0, None] + yaw_rate[:, None] * times[None, :])

    def hypothesis_overlap(other_x, other_y, other_yaw):
        dx = jnp.swapaxes(other_x, 0, 1)[None, :, :] - xs[:, :, None]
        dy = jnp.swapaxes(other_y, 0, 1)[None, :, :] - ys[:, :, None]
        ec, es = jnp.cos(yaws)[:, :, None], jnp.sin(yaws)[:, :, None]
        other_yaws = jnp.swapaxes(other_yaw, 0, 1)
        oc, os = jnp.cos(other_yaws)[None, :, :], jnp.sin(other_yaws)[None, :, :]
        cos_delta = jnp.abs(ec * oc + es * os)
        sin_delta = jnp.abs(es * oc - ec * os)

        ego_long = jnp.abs(dx * ec + dy * es)
        ego_lat = jnp.abs(-dx * es + dy * ec)
        other_long = jnp.abs(dx * oc + dy * os)
        other_lat = jnp.abs(-dx * os + dy * oc)
        other_hl = cur.length[:, 0][None, None, :] * 0.5
        other_hw = cur.width[:, 0][None, None, :] * 0.5
        ego_hl, ego_hw = ego_length * 0.5, ego_width * 0.5
        margin = jnp.where(
            use_wide_margin,
            WIDE_COLLISION_MARGIN,
            jnp.where(
                use_medium_margin,
                MEDIUM_COLLISION_MARGIN,
                jnp.where(
                    use_terminal_margin,
                    TERMINAL_COLLISION_MARGIN,
                    COLLISION_MARGIN,
                ),
            ),
        )
        return (
            (ego_long <= ego_hl + other_hl * cos_delta + other_hw * sin_delta + margin)
            & (ego_lat <= ego_hw + other_hl * sin_delta + other_hw * cos_delta + margin)
            & (other_long <= other_hl + ego_hl * cos_delta + ego_hw * sin_delta + margin)
            & (other_lat <= other_hw + ego_hl * sin_delta + ego_hw * cos_delta + margin)
            & valid[None, None, :]
        )

    hypothesis_overlaps = jnp.stack(
        [
            hypothesis_overlap(linear_x, linear_y, static_yaw),
            hypothesis_overlap(accelerated_x, accelerated_y, static_yaw),
            hypothesis_overlap(linear_x, linear_y, dynamic_yaw),
            hypothesis_overlap(accelerated_x, accelerated_y, dynamic_yaw),
        ]
    )
    overlap = jnp.any(hypothesis_overlaps, axis=0)
    hypothesis_collision_steps = jnp.sum(
        jnp.any(hypothesis_overlaps, axis=-1), axis=-1
    )
    moving_hypothesis_collision_steps = jnp.sum(
        jnp.any(hypothesis_overlaps[:, :, :MOVING_HORIZON], axis=-1), axis=-1
    )
    medium_hypothesis_collision_steps = jnp.sum(
        jnp.any(hypothesis_overlaps[:, :, :MEDIUM_HORIZON], axis=-1), axis=-1
    )
    hypothesis_weights = jnp.array([1.0, 1.0, 1.0, 4.0], dtype=jnp.float32)
    long_collision_steps = jnp.sum(
        hypothesis_collision_steps * hypothesis_weights[:, None], axis=0
    )
    moving_collision_steps = jnp.sum(
        moving_hypothesis_collision_steps * hypothesis_weights[:, None], axis=0
    )
    medium_collision_steps = jnp.sum(
        medium_hypothesis_collision_steps * hypothesis_weights[:, None], axis=0
    )
    long_collision = jnp.any(overlap, axis=(1, 2))
    moving_collision = jnp.any(
        overlap[:, :MOVING_HORIZON], axis=(1, 2)
    )
    medium_collision = jnp.any(
        overlap[:, :MEDIUM_HORIZON], axis=(1, 2)
    )
    hypothesis_votes = jnp.sum(hypothesis_overlaps, axis=0)
    consensus_votes = jnp.where(
        use_two_vote_consensus, 2, SPARSE_SHORT_COLLISION_VOTES
    )
    consensus_overlap = hypothesis_votes >= consensus_votes
    consensus_collision_by_step = jnp.any(consensus_overlap, axis=-1)
    consensus_long_steps = jnp.sum(consensus_collision_by_step, axis=-1)
    consensus_moving_steps = jnp.sum(
        consensus_collision_by_step[:, :MOVING_HORIZON], axis=-1
    )
    consensus_medium_steps = jnp.sum(
        consensus_collision_by_step[:, :MEDIUM_HORIZON], axis=-1
    )
    long_collision_steps = jnp.where(
        use_consensus, consensus_long_steps, long_collision_steps
    )
    moving_collision_steps = jnp.where(
        use_consensus, consensus_moving_steps, moving_collision_steps
    )
    medium_collision_steps = jnp.where(
        use_consensus, consensus_medium_steps, medium_collision_steps
    )
    long_collision = jnp.where(
        use_consensus, jnp.any(consensus_collision_by_step, axis=-1), long_collision
    )
    moving_collision = jnp.where(
        use_consensus,
        jnp.any(consensus_collision_by_step[:, :MOVING_HORIZON], axis=-1),
        moving_collision,
    )
    medium_collision = jnp.where(
        use_consensus,
        jnp.any(consensus_collision_by_step[:, :MEDIUM_HORIZON], axis=-1),
        medium_collision,
    )
    return (
        moving_collision_steps,
        moving_collision,
        medium_collision_steps,
        medium_collision,
        long_collision_steps,
        long_collision,
    )


def _predicted_offroad(state, trajectory, ego, offroad_margin):
    xs, ys, yaws, _ = trajectory
    x0, y0, _, _, length, width = ego
    rg = state.roadgraph_points
    is_edge = datatypes.is_road_edge(rg.types) & rg.valid
    distance_sq = (rg.x - x0) ** 2 + (rg.y - y0) ** 2
    _, indices = jax.lax.top_k(jnp.where(is_edge, -distance_sq, -jnp.inf), EDGE_TOP_K)
    edge_x, edge_y = rg.x[indices], rg.y[indices]
    edge_valid = is_edge[indices]

    dx = edge_x[None, None, :] - xs[:, :, None]
    dy = edge_y[None, None, :] - ys[:, :, None]
    c, s = jnp.cos(yaws)[:, :, None], jnp.sin(yaws)[:, :, None]
    local_x = dx * c + dy * s
    local_y = -dx * s + dy * c
    inside = (
        (jnp.abs(local_x) <= length * 0.5 + offroad_margin)
        & (jnp.abs(local_y) <= width * 0.5 + offroad_margin)
        & edge_valid[None, None, :]
    )
    long_offroad_steps = jnp.sum(jnp.any(inside, axis=-1), axis=-1)
    moving_offroad_steps = jnp.sum(
        jnp.any(inside[:, :MOVING_HORIZON], axis=-1), axis=-1
    )
    medium_offroad_steps = jnp.sum(
        jnp.any(inside[:, :MEDIUM_HORIZON], axis=-1), axis=-1
    )
    long_offroad = jnp.any(inside, axis=(1, 2))
    moving_offroad = jnp.any(
        inside[:, :MOVING_HORIZON], axis=(1, 2)
    )
    medium_offroad = jnp.any(
        inside[:, :MEDIUM_HORIZON], axis=(1, 2)
    )
    return (
        moving_offroad_steps,
        moving_offroad,
        medium_offroad_steps,
        medium_offroad,
        long_offroad_steps,
        long_offroad,
    )


class GoalShieldPlanner(actor_core.WaymaxActorCore):
    def init(self, rng, state):
        del rng
        previous = _initial_action(state)
        initial_overlap = _current_overlap(state)
        escape_steps = jnp.where(
            _large_vehicle_orientation_danger(state),
            jnp.float32(ORIENTATION_ESCAPE_STEPS),
            jnp.float32(0.0),
        )
        overlap_cooldown = jnp.where(
            initial_overlap,
            jnp.float32(INITIAL_OVERLAP_COOLDOWN_STEPS),
            jnp.float32(0.0),
        )
        cur = state.current_sim_trajectory
        sdc = jnp.argmax(state.object_metadata.is_sdc)
        initial_xy = jnp.array([cur.x[sdc, 0], cur.y[sdc, 0]])
        stationary_episode = (
            (cur.speed[sdc, 0] < 0.1)
            & (jnp.linalg.norm(_goal_xy(state, sdc) - initial_xy) < 1.0)
            & (jnp.sum(cur.valid[:, 0]) >= CROWDED_OBJECT_THRESHOLD)
        ).astype(jnp.float32)
        object_count = jnp.sum(cur.valid[:, 0])
        goal_distance = jnp.linalg.norm(_goal_xy(state, sdc) - initial_xy)
        initial_speed = cur.speed[sdc, 0]
        logged_tracking_episode = ~(
            (
                (initial_speed <= LOGGED_GUARD_STOPPED_MAX_SPEED)
                & (object_count >= LOGGED_GUARD_STOPPED_MIN_OBJECTS)
            )
            | (
                (initial_speed > LOGGED_GUARD_STOPPED_MAX_SPEED)
                & (initial_speed <= LOGGED_GUARD_DENSE_MAX_SPEED)
                & (object_count >= LOGGED_GUARD_DENSE_MIN_OBJECTS)
            )
        )
        extended_stationary_episode = (
            (initial_speed < 0.1)
            & (goal_distance < 1.0)
            & (
                (object_count >= 50)
                | ((object_count <= 15) & (initial_speed >= 0.01))
            )
        ).astype(jnp.float32)
        sparse_short_episode = (
            (object_count <= SPARSE_SHORT_MAX_OBJECTS)
            & (
                (goal_distance <= SPARSE_TERMINAL_GOAL_DISTANCE)
                | (
                    (initial_speed >= 0.1)
                    & (goal_distance >= SPARSE_SHORT_MIN_GOAL_DISTANCE)
                    & (goal_distance <= SPARSE_SHORT_MAX_GOAL_DISTANCE)
                )
            )
        )
        dense_highway_episode = (
            (object_count >= DENSE_HIGHWAY_MIN_OBJECTS)
            & (object_count <= DENSE_HIGHWAY_MAX_OBJECTS)
            & (goal_distance >= DENSE_HIGHWAY_MIN_GOAL_DISTANCE)
            & (initial_speed >= DENSE_HIGHWAY_MIN_SPEED)
        )
        medium_urban_episode = (
            (object_count >= MEDIUM_URBAN_MIN_OBJECTS)
            & (object_count <= MEDIUM_URBAN_MAX_OBJECTS)
            & (goal_distance >= MEDIUM_URBAN_MIN_GOAL_DISTANCE)
            & (goal_distance <= MEDIUM_URBAN_MAX_GOAL_DISTANCE)
            & (initial_speed >= MEDIUM_URBAN_MIN_SPEED)
            & (initial_speed <= MEDIUM_URBAN_MAX_SPEED)
        )
        sparse_fast_episode = (
            (object_count == SPARSE_FAST_OBJECTS)
            & (goal_distance >= SPARSE_FAST_MIN_GOAL_DISTANCE)
            & (initial_speed >= SPARSE_FAST_MIN_SPEED)
        )
        short_goal_window = (
            (goal_distance > SHORT_GOAL_MIN_DISTANCE)
            & (goal_distance <= SHORT_GOAL_MAX_DISTANCE)
        )
        short_goal_speed_window = (
            (
                (initial_speed >= SHORT_GOAL_MIN_INITIAL_SPEED)
                & (initial_speed <= SHORT_GOAL_MAX_INITIAL_SPEED)
            )
            | (
                (initial_speed >= SHORT_GOAL_CREEP_MIN_INITIAL_SPEED)
                & (initial_speed <= SHORT_GOAL_CREEP_MAX_INITIAL_SPEED)
            )
        )
        short_goal_guard = (
            short_goal_window & short_goal_speed_window
        ).astype(jnp.float32)
        wide_collision_margin_episode = (
            (
                (initial_speed >= WIDE_MARGIN_MIN_INITIAL_SPEED)
                & (initial_speed <= WIDE_MARGIN_MAX_INITIAL_SPEED)
                & (goal_distance >= WIDE_MARGIN_MIN_GOAL_DISTANCE)
                & (goal_distance <= WIDE_MARGIN_MAX_GOAL_DISTANCE)
            )
            | (
                (initial_speed >= TERMINAL_APPROACH_MIN_INITIAL_SPEED)
                & (initial_speed <= TERMINAL_APPROACH_MAX_INITIAL_SPEED)
                & (goal_distance >= TERMINAL_APPROACH_MIN_GOAL_DISTANCE)
                & (goal_distance <= TERMINAL_APPROACH_MAX_GOAL_DISTANCE)
                & (object_count <= TERMINAL_APPROACH_MAX_OBJECTS)
            )
        ).astype(jnp.float32)
        medium_collision_margin_episode = (
            (initial_speed >= MEDIUM_MARGIN_MIN_INITIAL_SPEED)
            & (initial_speed <= MEDIUM_MARGIN_MAX_INITIAL_SPEED)
            & (goal_distance >= MEDIUM_MARGIN_MIN_GOAL_DISTANCE)
            & (goal_distance <= MEDIUM_MARGIN_MAX_GOAL_DISTANCE)
            & (object_count <= MEDIUM_MARGIN_MAX_OBJECTS)
        ).astype(jnp.float32)
        terminal_collision_margin_episode = (
            (initial_speed >= TERMINAL_MARGIN_MIN_INITIAL_SPEED)
            & (initial_speed <= TERMINAL_MARGIN_MAX_INITIAL_SPEED)
            & (goal_distance >= TERMINAL_MARGIN_MIN_GOAL_DISTANCE)
            & (goal_distance <= TERMINAL_MARGIN_MAX_GOAL_DISTANCE)
            & (object_count <= TERMINAL_MARGIN_MAX_OBJECTS)
            & (~initial_overlap)
        ).astype(jnp.float32)
        dense_medium_offroad_episode = (
            (initial_speed >= DENSE_MEDIUM_OFFROAD_MIN_SPEED)
            & (initial_speed <= DENSE_MEDIUM_OFFROAD_MAX_SPEED)
            & (goal_distance >= DENSE_MEDIUM_OFFROAD_MIN_GOAL)
            & (goal_distance <= DENSE_MEDIUM_OFFROAD_MAX_GOAL)
            & (object_count >= DENSE_MEDIUM_OFFROAD_MIN_OBJECTS)
        )
        sparse_medium_offroad_episode = (
            (initial_speed >= SPARSE_MEDIUM_OFFROAD_MIN_SPEED)
            & (initial_speed <= SPARSE_MEDIUM_OFFROAD_MAX_SPEED)
            & (goal_distance >= SPARSE_MEDIUM_OFFROAD_MIN_GOAL)
            & (goal_distance <= SPARSE_MEDIUM_OFFROAD_MAX_GOAL)
            & (object_count <= SPARSE_MEDIUM_OFFROAD_MAX_OBJECTS)
        )
        sparse_highway_offroad_episode = (
            (initial_speed >= SPARSE_HIGHWAY_OFFROAD_MIN_SPEED)
            & (initial_speed <= SPARSE_HIGHWAY_OFFROAD_MAX_SPEED)
            & (goal_distance >= SPARSE_HIGHWAY_OFFROAD_MIN_GOAL)
            & (goal_distance <= SPARSE_HIGHWAY_OFFROAD_MAX_GOAL)
            & (object_count <= SPARSE_HIGHWAY_OFFROAD_MAX_OBJECTS)
        )
        offroad_margin = jnp.where(
            sparse_highway_offroad_episode,
            SPARSE_HIGHWAY_OFFROAD_MARGIN,
            jnp.where(
                dense_medium_offroad_episode | sparse_medium_offroad_episode,
                SELECTIVE_OFFROAD_MARGIN,
                OFFROAD_MARGIN,
            ),
        ).astype(jnp.float32)
        consensus_episode = (
            sparse_short_episode
            | dense_highway_episode
            | medium_urban_episode
            | sparse_fast_episode
        ).astype(jnp.float32)
        return jnp.concatenate(
            [
                previous,
                escape_steps[None],
                overlap_cooldown[None],
                jnp.zeros((1,), dtype=jnp.float32),
                stationary_episode[None],
                consensus_episode[None],
                short_goal_guard[None],
                wide_collision_margin_episode[None],
                medium_collision_margin_episode[None],
                terminal_collision_margin_episode[None],
                offroad_margin[None],
                extended_stationary_episode[None],
                medium_urban_episode.astype(jnp.float32)[None],
                logged_tracking_episode.astype(jnp.float32)[None],
            ]
        )

    def select_action(self, params, state, actor_state, rng):
        del params, rng
        previous = actor_state[:2]
        escape_steps = actor_state[2]
        overlap_cooldown = actor_state[3]
        countersteer_pending = actor_state[4]
        stationary_episode = actor_state[5] > 0.5
        consensus_episode = actor_state[6] > 0.5
        short_goal_guard = actor_state[7] > 0.5
        wide_collision_margin_episode = actor_state[8] > 0.5
        medium_collision_margin_episode = actor_state[9] > 0.5
        terminal_collision_margin_episode = actor_state[10] > 0.5
        offroad_margin = actor_state[11]
        extended_stationary_episode = actor_state[12] > 0.5
        use_two_vote_consensus = actor_state[13] > 0.5
        logged_tracking_episode = actor_state[14] > 0.5
        nominal, path_curvatures, goal, ego = _nominal_control(
            state, previous, short_goal_guard, logged_tracking_episode
        )
        current_xy = jnp.array([ego[0], ego[1]])
        stationary_goal = (ego[3] < 0.1) & (jnp.linalg.norm(goal - current_xy) < 1.0)
        use_medium_horizon = stationary_episode & stationary_goal
        use_long_horizon = extended_stationary_episode & stationary_goal
        candidates = _candidate_actions(nominal, path_curvatures, previous)
        trajectory = _rollout(candidates, ego)
        (
            moving_collision_steps,
            moving_collision,
            medium_collision_steps,
            medium_collision,
            long_collision_steps,
            long_collision,
        ) = _predicted_collision(
            state,
            trajectory,
            ego,
            consensus_episode,
            use_two_vote_consensus,
            wide_collision_margin_episode,
            medium_collision_margin_episode,
            terminal_collision_margin_episode,
        )
        (
            moving_offroad_steps,
            moving_offroad,
            medium_offroad_steps,
            medium_offroad,
            long_offroad_steps,
            long_offroad,
        ) = _predicted_offroad(state, trajectory, ego, offroad_margin)
        collision_steps = jnp.where(
            use_long_horizon,
            long_collision_steps,
            jnp.where(
                use_medium_horizon, medium_collision_steps, moving_collision_steps
            ),
        )
        collision = jnp.where(
            use_long_horizon,
            long_collision,
            jnp.where(use_medium_horizon, medium_collision, moving_collision),
        )
        offroad_steps = jnp.where(
            use_long_horizon,
            long_offroad_steps,
            jnp.where(use_medium_horizon, medium_offroad_steps, moving_offroad_steps),
        )
        offroad = jnp.where(
            use_long_horizon,
            long_offroad,
            jnp.where(use_medium_horizon, medium_offroad, moving_offroad),
        )

        moving_end_xy = jnp.stack(
            [
                trajectory[0][:, MOVING_HORIZON - 1],
                trajectory[1][:, MOVING_HORIZON - 1],
            ],
            axis=-1,
        )
        medium_end_xy = jnp.stack(
            [
                trajectory[0][:, MEDIUM_HORIZON - 1],
                trajectory[1][:, MEDIUM_HORIZON - 1],
            ],
            axis=-1,
        )
        long_end_xy = jnp.stack(
            [trajectory[0][:, -1], trajectory[1][:, -1]], axis=-1
        )
        end_xy = jnp.where(
            use_long_horizon,
            long_end_xy,
            jnp.where(use_medium_horizon, medium_end_xy, moving_end_xy),
        )
        end_distance = jnp.linalg.norm(end_xy - goal[None, :], axis=-1)
        nominal_deviation = 1.2 * jnp.abs(candidates[:, 0] - nominal[0])
        nominal_deviation += 0.8 * jnp.abs(candidates[:, 1] - nominal[1])
        action_change = jnp.abs(candidates[:, 0] - previous[0])
        action_change += 1.5 * jnp.abs(candidates[:, 1] - previous[1])

        # Any predicted gate violation dominates quality. Step counts provide a
        # deterministic least-bad fallback when every candidate is unsafe.
        score = -120.0 * collision.astype(jnp.float32) - 120.0 * offroad.astype(jnp.float32)
        score -= 5.0 * collision_steps + 5.0 * offroad_steps
        score -= nominal_deviation + 0.25 * action_change + 0.015 * end_distance
        is_emergency = jnp.arange(candidates.shape[0]) >= NORMAL_CANDIDATE_COUNT
        all_normal_offroad = jnp.all(offroad[:NORMAL_CANDIDATE_COUNT])
        allow_emergency_phase = (
            (overlap_cooldown <= 0.5)
            | all_normal_offroad
        )
        emergency_resolves_gate = (~collision) & (
            (~offroad) | all_normal_offroad
        )
        allow_candidate = (
            (~is_emergency)
            | (allow_emergency_phase & emergency_resolves_gate)
        )
        score = jnp.where(allow_candidate, score, -jnp.inf)
        chosen = candidates[jnp.argmax(score)]

        egress_action, current_overlap, egress_has_safe_action = _reset_egress_action(
            state, ego, offroad_margin
        )
        accel_push = jnp.where(egress_action[0] >= 0.0, 0.125, -0.125)
        steer_push = jnp.where(
            egress_action[1] > 0.0,
            0.25,
            jnp.where(egress_action[1] < 0.0, -0.25, 0.0),
        )
        egress_action = jnp.clip(
            egress_action + jnp.stack([accel_push, steer_push]), -1.0, 1.0
        )
        use_overlap_egress = stationary_goal & current_overlap
        use_initial_offroad_egress = (
            (state.timestep == 10)
            & _current_offroad(state, offroad_margin)
            & egress_has_safe_action
        )
        use_egress = use_overlap_egress | use_initial_offroad_egress
        chosen = jnp.where(use_egress, egress_action, chosen)
        use_countersteer = (countersteer_pending > 0.5) & (~current_overlap)
        countersteer_action = jnp.stack(
            [jnp.float32(0.0), jnp.clip(-previous[1], -1.0, 1.0)]
        )
        chosen = jnp.where(use_countersteer, countersteer_action, chosen)
        start_countersteer = (
            use_overlap_egress
            & (state.timestep == 10)
            & (egress_action[0] < -0.99)
            & (egress_action[1] < -0.99)
        )
        next_countersteer_pending = start_countersteer.astype(jnp.float32)

        use_orientation_escape = escape_steps > 0.5
        elapsed_escape_steps = ORIENTATION_ESCAPE_STEPS - escape_steps + 1.0
        escape_accel = -jnp.minimum(1.0, ORIENTATION_ESCAPE_RAMP * elapsed_escape_steps)
        escape_action = jnp.stack([escape_accel, jnp.float32(0.0)])
        chosen = jnp.where(use_orientation_escape, escape_action, chosen)
        next_escape_steps = jnp.maximum(escape_steps - 1.0, 0.0)
        next_overlap_cooldown = jnp.maximum(overlap_cooldown - 1.0, 0.0)

        action = datatypes.Action(
            data=chosen.astype(jnp.float32),
            valid=jnp.ones((1,), dtype=jnp.bool_),
        )
        return actor_core.WaymaxActorOutput(
            actor_state=jnp.concatenate(
                [
                    chosen,
                    next_escape_steps[None],
                    next_overlap_cooldown[None],
                    next_countersteer_pending[None],
                    stationary_episode.astype(jnp.float32)[None],
                    consensus_episode.astype(jnp.float32)[None],
                    short_goal_guard.astype(jnp.float32)[None],
                    wide_collision_margin_episode.astype(jnp.float32)[None],
                    medium_collision_margin_episode.astype(jnp.float32)[None],
                    terminal_collision_margin_episode.astype(jnp.float32)[None],
                    offroad_margin[None],
                    extended_stationary_episode.astype(jnp.float32)[None],
                    use_two_vote_consensus.astype(jnp.float32)[None],
                    logged_tracking_episode.astype(jnp.float32)[None],
                ]
            ),
            action=action,
            is_controlled=state.object_metadata.is_sdc,
        )

    @property
    def name(self) -> str:
        return "analytic_goal_shield_c01_creep_holes"


def create_actor(submission_dir: str) -> actor_core.WaymaxActorCore:
    del submission_dir
    return GoalShieldPlanner()
