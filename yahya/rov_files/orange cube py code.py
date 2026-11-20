import pygame
import time
import math
import sys
import shutil
from collections import deque
from pymavlink import mavutil


# ==========================================
# 1. SETTINGS
# ==========================================

CONNECTION = "com9"
BAUD_RATE = 115200

ARM_BUTTON = 7       # START
DISARM_BUTTON = 6    # BACK

HOLD_TIME = 3.0
DEADZONE = 0.12
SEND_RATE = 20

AXIS_LATERAL = 0
AXIS_FORWARD = 1
AXIS_YAW = 2
AXIS_THROTTLE = 3

INVERT_FORWARD = True
INVERT_THROTTLE = True

SURFACE_PRESSURE_HPA = None   # None = auto-calibrate from first reading
WATER_DENSITY = 1025
GRAVITY = 9.80665


# ==========================================
# 2. CONNECT TO CUBE ORANGE
# ==========================================

print("Connecting to Cube Orange...")

master = mavutil.mavlink_connection(CONNECTION, baud=BAUD_RATE)


def wait_for_autopilot(timeout=30):
    """Wait for the heartbeat that really comes from the autopilot
    (not a GCS, companion computer, gimbal, etc.) and lock onto it."""
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg is None:
            continue
        if (
            msg.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
            and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
            and msg.type != mavutil.mavlink.MAV_TYPE_GCS
        ):
            return msg.get_srcSystem(), msg.get_srcComponent()
    raise RuntimeError("No autopilot heartbeat received!")


AUTOPILOT_SYS, AUTOPILOT_COMP = wait_for_autopilot()
master.target_system = AUTOPILOT_SYS
master.target_component = AUTOPILOT_COMP

# Ask the Cube to stream telemetry (attitude, pressure, ...)
master.mav.request_data_stream_send(
    AUTOPILOT_SYS,
    AUTOPILOT_COMP,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    10,
    1
)

print("Cube Orange connected!")


# ==========================================
# 3. CONNECT CONTROLLER
# ==========================================

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("USB controller not detected!")

controller = pygame.joystick.Joystick(0)
controller.init()

print("Controller:", controller.get_name())


# ==========================================
# 4. VARIABLES
# ==========================================

roll = None
pitch = None
yaw = None
depth = None
pressure = None

armed = None                  # None = unknown, True/False only from Cube heartbeat
last_cube_heartbeat = None    # time of last heartbeat from the Cube
HEARTBEAT_TIMEOUT = 3.0

heartbeat_count = 0           # total Cube heartbeats received
heartbeat_times = deque(maxlen=10)   # recent heartbeat times (for Hz)

# Estimated state: after we send an arm/disarm command, we assume it will
# work until the Cube's heartbeat confirms (or the timeout expires).
pending_state = None          # True = arming, False = disarming, None = none
pending_since = 0.0
PENDING_TIMEOUT = 5.0
last_ack = ""                 # last COMMAND_ACK result for arm/disarm

last_known_armed = None       # last actual state, kept for the stale estimate
tx_errors = 0                 # failed control sends

tx_count = 0                  # control packets sent to the Cube
tx_times = deque(maxlen=40)   # recent TX times (for Hz)

arm_start = None
disarm_start = None

arm_command_sent = False
disarm_command_sent = False

last_send = 0
last_heartbeat = 0
last_print = 0


# ==========================================
# 5. CONTROLLER FUNCTIONS
# ==========================================

def apply_deadzone(value):
    if abs(value) < DEADZONE:
        return 0.0
    return value


def get_axis(index, invert=False):
    if index >= controller.get_numaxes():
        return 0.0

    value = controller.get_axis(index)

    if invert:
        value = -value

    return apply_deadzone(value)


def get_controls():
    pygame.event.pump()

    forward = get_axis(AXIS_FORWARD, INVERT_FORWARD)
    lateral = get_axis(AXIS_LATERAL)
    yaw_control = get_axis(AXIS_YAW)
    throttle = get_axis(AXIS_THROTTLE, INVERT_THROTTLE)

    return forward, lateral, throttle, yaw_control


# ==========================================
# 6. ARM / DISARM COMMANDS
# ==========================================

def send_arm_command(arm):
    global pending_state, pending_since, last_ack

    pending_state = bool(arm)
    pending_since = time.monotonic()
    last_ack = ""

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1 if arm else 0,
        0, 0, 0, 0, 0, 0
    )


def send_gcs_heartbeat():
    """ArduSub may fail-safe/disarm without a GCS heartbeat."""
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0
    )


# ==========================================
# 7. MANUAL ARM / DISARM BUTTONS
# ==========================================

def handle_arm_buttons(forward, lateral, throttle, yaw_control):

    global arm_start, disarm_start
    global arm_command_sent, disarm_command_sent

    current_time = time.monotonic()

    arm_pressed = controller.get_button(ARM_BUTTON)
    disarm_pressed = controller.get_button(DISARM_BUTTON)

    controls_neutral = (
        abs(forward) < 0.05
        and abs(lateral) < 0.05
        and abs(throttle) < 0.05
        and abs(yaw_control) < 0.05
    )

    # ARM: hold START for 3 seconds
    if arm_pressed and armed is False and controls_neutral:
        if arm_start is None:
            arm_start = current_time

        if current_time - arm_start >= HOLD_TIME and not arm_command_sent:
            send_arm_command(True)
            arm_command_sent = True
    else:
        arm_start = None
        if not arm_pressed:
            arm_command_sent = False

    # DISARM: hold BACK for 3 seconds
    if disarm_pressed and armed:
        if disarm_start is None:
            disarm_start = current_time

        if current_time - disarm_start >= HOLD_TIME and not disarm_command_sent:
            send_arm_command(False)
            disarm_command_sent = True
    else:
        disarm_start = None
        if not disarm_pressed:
            disarm_command_sent = False


# ==========================================
# 8. MOVEMENT COMMANDS
# ==========================================

def send_movement(forward, lateral, throttle, yaw_control):
    x = int(forward * 1000)
    y = int(lateral * 1000)
    z = int(500 + throttle * 500)
    r = int(yaw_control * 1000)

    master.mav.manual_control_send(
        master.target_system,
        x, y, z, r,
        0
    )


# ==========================================
# 9. READ CUBE TELEMETRY
# ==========================================

def read_telemetry():

    global roll, pitch, yaw
    global pressure, depth, armed
    global SURFACE_PRESSURE_HPA
    global last_cube_heartbeat, heartbeat_count
    global pending_state, last_ack

    while True:
        msg = master.recv_match(blocking=False)

        if msg is None:
            break

        msg_type = msg.get_type()

        if msg_type == "BAD_DATA":
            continue

        # Only accept messages from the autopilot itself
        if msg.get_srcSystem() != AUTOPILOT_SYS:
            continue

        if msg_type == "HEARTBEAT":

            # Only the Cube (autopilot) heartbeat decides the armed state.
            # Ignore GCS, companion computers, gimbals, etc.
            if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                continue
            if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                continue

            armed = bool(
                msg.base_mode
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            last_cube_heartbeat = time.monotonic()
            heartbeat_count += 1
            heartbeat_times.append(last_cube_heartbeat)

            # Actual state now known: estimate is no longer needed
            if pending_state is not None and armed == pending_state:
                pending_state = None

        elif msg_type == "COMMAND_ACK":

            if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    last_ack = "ACK:OK"
                else:
                    last_ack = f"ACK:REJECTED({msg.result})"
                    pending_state = None   # command refused, drop the estimate

        elif msg_type == "ATTITUDE":

            roll = math.degrees(msg.roll)
            pitch = math.degrees(msg.pitch)
            yaw = math.degrees(msg.yaw)

        elif msg_type in (
            "SCALED_PRESSURE",
            "SCALED_PRESSURE2",
            "SCALED_PRESSURE3"
        ):

            if msg.press_abs > 0:

                pressure = msg.press_abs

                # Auto-calibrate surface pressure on first reading
                if SURFACE_PRESSURE_HPA is None:
                    SURFACE_PRESSURE_HPA = pressure

                depth = (
                    (pressure - SURFACE_PRESSURE_HPA) * 100
                ) / (WATER_DENSITY * GRAVITY)


# ==========================================
# 10. OUTPUT FORMATTING
# ==========================================

def fmt(value, width=7):
    if value is None:
        return "N/A".rjust(width)
    return f"{value:{width}.2f}"


def rate_hz(times):
    """Average rate (Hz) from a deque of timestamps."""
    if len(times) < 2:
        return 0.0
    span = times[-1] - times[0]
    if span <= 0:
        return 0.0
    return (len(times) - 1) / span


def print_live_line(text):
    """Print a single line that always overwrites itself,
    clipped to the terminal width so it can never wrap."""
    width = shutil.get_terminal_size((120, 20)).columns - 1
    text = text[:width]
    sys.stdout.write("\r" + text.ljust(width))
    sys.stdout.flush()


# ==========================================
# 11. MAIN LOOP
# ==========================================

print("Starting ROV control...\n")

try:

    while True:

        forward, lateral, throttle, yaw_control = get_controls()

        read_telemetry()

        handle_arm_buttons(forward, lateral, throttle, yaw_control)

        current_time = time.monotonic()

        # GCS heartbeat at 1 Hz
        if current_time - last_heartbeat >= 1.0:
            send_gcs_heartbeat()
            last_heartbeat = current_time

        # Is the Cube heartbeat alive?
        heartbeat_ok = (
            last_cube_heartbeat is not None
            and current_time - last_cube_heartbeat < HEARTBEAT_TIMEOUT
        )

        # Heartbeat lost -> actual state is unknown (remember the last one)
        if not heartbeat_ok and armed is not None:
            last_known_armed = armed
            armed = None
        elif armed is not None:
            last_known_armed = armed

        # Drop the estimate if the Cube never confirmed the command
        if (
            pending_state is not None
            and current_time - pending_since > PENDING_TIMEOUT
        ):
            pending_state = None
            last_ack = (last_ack + " NO-CONFIRM").strip()

        # ALWAYS send the controls to the Cube at SEND_RATE
        # (the Cube itself ignores them while disarmed)
        if current_time - last_send >= 1 / SEND_RATE:
            try:
                send_movement(forward, lateral, throttle, yaw_control)
                tx_count += 1
                tx_times.append(current_time)
            except Exception:
                tx_errors += 1
            last_send = current_time

        # Update the live line 10 times per second
        if current_time - last_print >= 0.1:

            # ---- State: ACTUAL (from heartbeat) or EST (estimated) ----
            if pending_state is not None:
                # Command sent, waiting for the Cube to confirm
                shown = "ARMED" if pending_state else "DISARMED"
                kind = "EST"
            elif armed is not None:
                shown = "ARMED" if armed else "DISARMED"
                kind = "ACTUAL"
            elif last_known_armed is not None:
                # Heartbeat lost: show last known state as an estimate
                shown = "ARMED" if last_known_armed else "DISARMED"
                kind = "EST-STALE"
            else:
                shown = "UNKNOWN"
                kind = "WAITING"

            # ---- Heartbeat: count, age since last, rate ----
            if last_cube_heartbeat is None:
                hb = "HB:none"
            else:
                age = current_time - last_cube_heartbeat
                hb = (
                    f"HB:#{heartbeat_count} {age:3.1f}s "
                    f"{rate_hz(heartbeat_times):3.1f}Hz"
                )
                if not heartbeat_ok:
                    hb += " LOST!"

            # ---- Control packets sent to the Cube ----
            tx = f"TX:{rate_hz(tx_times):4.1f}Hz #{tx_count}"
            if tx_errors:
                tx += f" ERR:{tx_errors}"

            ack = f" {last_ack}" if last_ack else ""

            print_live_line(
                f"[{shown}|{kind}] {hb} | {tx}{ack} | "
                f"R:{fmt(roll, 6)} P:{fmt(pitch, 6)} Y:{fmt(yaw, 6)} "
                f"D:{fmt(depth, 5)}m | "
                f"F:{forward:5.2f} L:{lateral:5.2f} "
                f"T:{throttle:5.2f} Yw:{yaw_control:5.2f}"
            )

            last_print = current_time

        time.sleep(0.01)


except KeyboardInterrupt:
    sys.stdout.write("\nProgram stopped by user.\n")
    sys.stdout.flush()

finally:
    pygame.quit()