#############################################################################################
# DON'T CHANGE ANY OF THE BELOW. NECESSARY FOR CORRECTLY JOINING THE COMPETITION SIMULATION #
#############################################################################################

import os, sys, time, datetime, traceback
import spaceteams as st
def custom_exception_handler(exctype, value, tb):
    error_message = "".join(traceback.format_exception(exctype, value, tb))
    st.logger_fatal(error_message)
sys.excepthook = custom_exception_handler
st.connect_to_sim(sys.argv)

import numpy as np
import numpy.typing as npt


this = st.GetThisSystem()

doSpawning = st.SimGlobals.GetSimEntity().GetParam(st.VarType.bool, "DoSpawning")
if doSpawning:
    while True:
        time.sleep(1.0 / this.GetParam(st.VarType.double, "LoopFreqHz"))

phobos: st.Entity = this.GetParam(st.VarType.entityRef, "Phobos")
phobosFrame = phobos.GetBodyFixedFrame()
shuttle: st.Entity = this.GetParam(st.VarType.entityRef, "Shuttle")
shuttleFrame = shuttle.GetBodyFixedFrame()
target: st.Entity = this.GetParam(st.VarType.entityRef, "Target")

topCamera: st.Entity = this.GetParam(st.VarType.entityRef, "TopCamera")
topCameraFrame = topCamera.GetBodyFixedFrame()
bottomCamera: st.Entity = this.GetParam(st.VarType.entityRef, "BottomCamera")
bottomCameraFrame = bottomCamera.GetBodyFixedFrame()


def GetNearestDebrisLocations(N: int) -> list[npt.NDArray]:
    allDebrisInfo: list[npt.NDArray] = shuttle.GetParamArray(st.VarType.doubleM3x3, 
                                                            ["Radar", "SortedDebrisData"])
    nearest = allDebrisInfo[:min(N, len(allDebrisInfo))]
    return nearest


def PanTiltCamera(camera: st.Entity, yaw_deg: float, pitch_deg: float) -> None:
    if camera == topCamera:
        yaw_deg = np.clip(yaw_deg, -89.0, 89.0)
        pitch_deg = np.clip(pitch_deg, -50.0, 0.0)
    elif camera == bottomCamera:
        yaw_deg = np.clip(yaw_deg, -89.0, 89.0)
        pitch_deg = np.clip(pitch_deg, 0.0, 50.0)
    camera.setRotation_DCM(st.math.Euler321_to_DCM([np.deg2rad(yaw_deg), np.deg2rad(pitch_deg), 0.0]), shuttleFrame)


def SetCommand(accelCommand: npt.NDArray, angAccelCommand: npt.NDArray) -> None:
    shuttle.SetParam(st.VarType.doubleV3, ["JetSelection", "CommandedAccel"], accelCommand)
    shuttle.SetParam(st.VarType.doubleV3, ["JetSelection", "CommandedAngAccel"], angAccelCommand)


#############################################################################################
# DON'T CHANGE ANY OF THE ABOVE. NECESSARY FOR CORRECTLY JOINING THE COMPETITION SIMULATION #
#############################################################################################


###############################################################
# ESCAPE FROM PHOBOS — MARK 1 (initial, untested controller)   #
# Only this user-code region is changed.                      #
###############################################################

# Controller design:
#   1. Approach the destination with a speed-limited PD controller.
#   2. Use radar to select a sideways velocity around debris in our path.
#   3. Convert the final acceleration from Phobos frame to body frame.
#
# IMPORTANT: This is a prototype. It is not a proven collision-free
# trajectory planner. Evaluate the STP logs and video before tuning.

# --- Tuning parameters (start conservatively) ---
CRUISE_SPEED_M_S = 17.0       # Maximum requested cruise speed
MAX_ACCEL_M_S2 = 1.1          # Maximum requested acceleration magnitude
GUIDANCE_GAIN_HZ = 0.40       # Velocity tracking gain
BRAKING_ACCEL_M_S2 = 0.65    # Lower bound for conservative braking estimate
ARRIVAL_BUFFER_M = 9.0       # Start aiming for near-zero speed near target

RADAR_OBJECTS = 30            # Number of nearby debris observations
LOOKAHEAD_SECONDS = 9.0
MIN_LOOKAHEAD_M = 100.0
SIDE_CLEARANCE_M = 23.0       # Extra clearance beyond debris radius
AVOID_SPEED_M_S = 13.0       # Maximum sideways correction speed
EMERGENCY_RANGE_M = 13.0     # Extra correction for very close debris


def unit(vector, fallback=None):
    """Return a unit vector, or a safe fallback for near-zero vectors."""
    vector = np.asarray(vector, dtype=float)
    n = float(np.linalg.norm(vector))
    if n > 1.0e-9 and np.isfinite(n):
        return vector / n
    if fallback is None:
        return np.zeros(3, dtype=float)
    return np.asarray(fallback, dtype=float)


def limit_vector(vector, limit):
    """Cap the magnitude of a 3D vector without changing its direction."""
    vector = np.asarray(vector, dtype=float)
    magnitude = float(np.linalg.norm(vector))
    if magnitude > limit and magnitude > 0.0:
        return vector * (limit / magnitude)
    return vector


def get_side_direction(goal_dir, outward_dir):
    """Find a consistent sideways direction, preferably level to the surface."""
    side = np.cross(outward_dir, goal_dir)
    if np.linalg.norm(side) < 1.0e-6:
        # If goal is radial, build a tangent to the outward direction.
        reference = np.array([0.0, 0.0, 1.0])
        if abs(np.dot(outward_dir, reference)) > 0.9:
            reference = np.array([1.0, 0.0, 0.0])
        side = np.cross(outward_dir, reference)
    return unit(side)


def choose_avoidance_velocity(radar, goal_dir, outward_dir, shuttle_velocity):
    """Simple, radar-only, look-ahead obstacle avoidance.

    Radar center positions are *relative to the shuttle* but their vector
    components are expressed in the *Phobos-fixed* reference frame.
    We do not read debris entity state directly (competition restriction).
    """
    speed = float(np.linalg.norm(shuttle_velocity))
    lookahead = max(MIN_LOOKAHEAD_M, speed * LOOKAHEAD_SECONDS)
    avoidance = np.zeros(3, dtype=float)
    fallback_side = get_side_direction(goal_dir, outward_dir)
    nearest_range = float('inf')
    threats = 0

    for item in radar:
        try:
            relative = np.asarray(item[0, :], dtype=float)
            closest_range = float(item[1, 0])
            range_rate = float(item[1, 1])
            diameter = max(0.0, float(item[1, 2]))
        except (IndexError, ValueError, TypeError):
            continue

        if not (np.all(np.isfinite(relative)) and np.isfinite(closest_range)
                and np.isfinite(range_rate) and np.isfinite(diameter)):
            continue

        center_distance = float(np.linalg.norm(relative))
        if center_distance < 1.0e-6:
            continue
        nearest_range = min(nearest_range, closest_range)

        radius = 0.5 * diameter
        forward = float(np.dot(relative, goal_dir))
        sideways = relative - forward * goal_dir
        lateral_distance = float(np.linalg.norm(sideways))
        clearance = radius + SIDE_CLEARANCE_M + min(12.0, speed * 0.5)

        # The rock lies ahead and blocks the goal corridor: aim to pass
        # around it, instead of pushing straight backwards into a stall.
        if 0.0 < forward < lookahead and lateral_distance < clearance:
            # Steer away from whichever side of the line the rock occupies.
            if lateral_distance > 1.0:
                side_dir = -sideways / lateral_distance
                # Reduce downward detours near Phobos' surface.
                side_dir -= np.dot(side_dir, outward_dir) * outward_dir
                side_dir = unit(side_dir, fallback_side)
            else:
                side_dir = fallback_side

            lateral_weight = 1.0 - lateral_distance / clearance
            ahead_weight = 1.0 - 0.5 * forward / lookahead
            # A rock already receding from us is a somewhat smaller threat.
            closing_weight = 1.0 if range_rate < 0.0 else 0.6
            weight = lateral_weight * ahead_weight * closing_weight
            avoidance += AVOID_SPEED_M_S * weight * side_dir
            threats += 1

        # An immediate close approach demands separation even if the
        # rock is slightly to the side or behind the nominal goal line.
        if closest_range < EMERGENCY_RANGE_M and closest_range >= -radius:
            away = -relative / center_distance
            weight = (EMERGENCY_RANGE_M - max(0.0, closest_range)) / EMERGENCY_RANGE_M
            avoidance += (AVOID_SPEED_M_S * weight) * away
            avoidance += (2.0 * weight) * outward_dir
            threats += 1

    return limit_vector(avoidance, AVOID_SPEED_M_S), nearest_range, threats


# --- Controller main loop ---
t0_sim = st.SimGlobals.GetSimEntity().GetParam(
    st.VarType.timestamp, ["Time", "StartDateTAI"]
)
t0 = t0_sim.as_datetime()
iteration = 0
exit_flag = False

while not exit_flag:
    time.sleep(1.0 / this.GetParam(st.VarType.double, "LoopFreqHz"))

    t_now = st.SimGlobals.SimClock.GetTimeNow().as_datetime()
    elapsed_s = (t_now - t0).total_seconds()

    # Trusted navigation state of shuttle and mission destination.
    state = shuttle.getLocVelAcc()
    shuttle_pos = np.asarray(state.loc_WRT_ExprIn(phobosFrame), dtype=float)
    shuttle_vel = np.asarray(state.vel_WRT_ExprIn(phobosFrame), dtype=float)
    target_pos = np.asarray(
        target.getLocVelAcc().loc_WRT_ExprIn(phobosFrame), dtype=float
    )

    to_target = target_pos - shuttle_pos
    distance = float(np.linalg.norm(to_target))
    goal_dir = unit(to_target)
    outward_dir = unit(shuttle_pos, np.array([0.0, 0.0, 1.0]))

    # Speed decreases near the target to avoid overshooting it.
    speed_limit = np.sqrt(2.0 * BRAKING_ACCEL_M_S2 * max(
        0.0, distance - ARRIVAL_BUFFER_M
    ))
    commanded_speed = min(CRUISE_SPEED_M_S, speed_limit)
    desired_velocity = commanded_speed * goal_dir

    # Radar returns noisy measurements, so the clearance is intentionally
    # wider than the reported rock radius.
    radar = GetNearestDebrisLocations(RADAR_OBJECTS)
    avoidance_velocity, nearest_range, threats = choose_avoidance_velocity(
        radar, goal_dir, outward_dir, shuttle_vel
    )
    desired_velocity += avoidance_velocity

    # PD velocity tracking: desired acceleration in the Phobos frame.
    acceleration_phobos = GUIDANCE_GAIN_HZ * (desired_velocity - shuttle_vel)
    acceleration_phobos = limit_vector(acceleration_phobos, MAX_ACCEL_M_S2)

    # Actuators require *body frame*, not Phobos frame.
    acceleration_body = st.frames.FramedVector(
        acceleration_phobos, phobosFrame
    ).ExprIn(shuttleFrame)
    angular_acceleration_body = np.zeros(3)
    SetCommand(acceleration_body, angular_acceleration_body)

    # Lightweight diagnostics that can be seen in STP and in recordings.
    if iteration % 50 == 0:
        damage = shuttle.GetParam(st.VarType.double, ["Stats", "Damage"])
        fuel_used = shuttle.GetParam(st.VarType.double, ["Stats", "FuelUsed"])
        nearest_text = (f"{nearest_range:.1f} m" if np.isfinite(nearest_range)
                        else "none")
        st.OnScreenLogMessage(
            f"MARK 1 | t={elapsed_s:.0f}s | target={distance:.0f}m | "
            f"speed={np.linalg.norm(shuttle_vel):.1f}m/s | "
            f"radar={nearest_text} | threats={threats} | "
            f"damage={damage:.2f} | fuel={fuel_used:.2f}",
            "Mark1Status", st.Severity.Info
        )

    iteration += 1

st.leave_sim()
