#!/usr/bin/env python3
"""
Pool Controller - Raspberry Pi
Stage 1: Heater control, temp sensor, OLED display, rotary encoder
Stage 2: Pool/Spa valve buttons (software state only until actuators installed)
Stage 3: Valve actuators (stub ready, activate when hardware connected)

Hardware:
  - DS18B20 temp sensor     GPIO 4  (kernel managed, 1-Wire)
  - Heater relay            GPIO 17
  - Rotary encoder CLK      GPIO 23
  - Rotary encoder DT       GPIO 24
  - Rotary encoder button   GPIO 22
  - Pool button             GPIO 5
  - Spa button              GPIO 6
  - Valve relay A (open)    GPIO 27  (Stage 3)
  - Valve relay A (close)   GPIO 13  (Stage 3)
  - Valve relay B (open)    GPIO 7   (Stage 3)
  - Valve relay B (close)   GPIO 8   (Stage 3)
  - OLED display            GPIO 2 (SDA), GPIO 3 (SCL) - I2C, kernel managed

MQTT Broker: 192.168.1.13:1883
"""

import time
import json
import threading
import logging
import glob
import os
import datetime
import gpiod
from gpiod.line import Direction, Value, Edge, Bias

import paho.mqtt.client as mqtt
from luma.core.interface.serial import i2c
from luma.oled.device import sh1106
from luma.core.render import canvas
from PIL import ImageFont

# -------------------------------------------------------
# Logging
# -------------------------------------------------------

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
log = logging.getLogger("pool")

# -------------------------------------------------------
# Configuration
# -------------------------------------------------------

BROKER_IP   = "192.168.1.13"
BROKER_PORT = 1883
BROKER_USER = "mqtt"
BROKER_PASS = "codex123"

TEMP_SENSOR_PATH = "/sys/bus/w1/devices/28-00000025218c/w1_slave"

SETPOINT_MIN     = 65.0
SETPOINT_MAX     = 104.0
SETPOINT_DEFAULT = 80.0

SENSOR_UNAVAILABLE_TIMEOUT = 180
CONTROL_LOOP_INTERVAL      = 10
HYSTERESIS                 = 2.0

CPU_TEMP_PATH = "/sys/class/thermal/thermal_zone0/temp"

VALVE_ACTUATORS_CONNECTED = True
VALVE_TRAVEL_TIME = 32  # seconds — measured 26s actual travel, 32s adds safety margin
STATE_FILE = "/home/pi/pool_state.json"
GPIO_CHIP  = "/dev/gpiochip0"

# -------------------------------------------------------
# GPIO Pin Assignments
# -------------------------------------------------------

# GPIO 4 reserved — DS18B20 1-Wire (kernel managed via dtoverlay=w1-gpio)

PIN_HEATER_RELAY  = 17
PIN_ENCODER_CLK   = 23
PIN_ENCODER_DT    = 24
PIN_ENCODER_SW    = 22
PIN_BTN_POOL      = 5
PIN_BTN_SPA       = 6
PIN_VALVE_OPEN    = 27   # Stage 3 — Actuator A open
PIN_VALVE_CLOSE   = 13   # Stage 3 — Actuator A close
PIN_VALVE_B_OPEN  = 7    # Stage 3 — Actuator B open
PIN_VALVE_B_CLOSE = 8    # Stage 3 — Actuator B close

# -------------------------------------------------------
# State
# -------------------------------------------------------

state = {
    "heater_enabled":           False,
    "heater_relay_on":          False,
    "water_temp":               None,
    "setpoint":                 SETPOINT_DEFAULT,
    "valve_position":           "pool",   # combined — "pool"/"spa"/"split"
    "valve_a_position":         "pool",   # Valve A individual position
    "valve_b_position":         "pool",   # Valve B individual position
    "sensor_unavailable_since": None,
    "pump_should_run":          False,   # From Node-RED schedule (bigtimer)
    "pump_is_on":               False,   # Actual pump state from HA
    "standby":                  False,   # Standby mode — disables physical controls
    "cpu_temp":                 None,
}

# -------------------------------------------------------
# MQTT Client
# -------------------------------------------------------

mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqtt_connected = False

# Valve movement locks — one per actuator, prevents simultaneous activation
_valve_a_moving        = False
_valve_b_moving        = False
_valve_a_lock          = threading.Lock()
_valve_b_lock          = threading.Lock()
_valve_sequence_moving = False  # System-level lock — blocks Pool/Spa buttons during full sequence
_pending_pump_off      = False  # Set True when pool mode delays pump-off; cleared if spa is pressed

# -------------------------------------------------------
# Persistent State
# -------------------------------------------------------

def save_state():
    data = {
        "setpoint":         state["setpoint"],
        "heater_enabled":   state["heater_enabled"],
        "valve_position":   state["valve_position"],
        "valve_a_position": state["valve_a_position"],
        "valve_b_position": state["valve_b_position"],
        "standby":          state["standby"],
    }
    try:
        with open(STATE_FILE, "w") as f:
            json.dump(data, f)
    except Exception as e:
        log.warning(f"Failed to save state: {e}")

def load_state():
    if not os.path.exists(STATE_FILE):
        log.info("No saved state found — using defaults")
        return
    try:
        with open(STATE_FILE, "r") as f:
            data = json.load(f)
        state["setpoint"]         = float(data.get("setpoint", SETPOINT_DEFAULT))
        state["heater_enabled"]   = bool(data.get("heater_enabled", False))
        state["valve_position"]   = data.get("valve_position", "pool")
        state["valve_a_position"] = data.get("valve_a_position", state["valve_position"])
        state["valve_b_position"] = data.get("valve_b_position", state["valve_position"])
        state["standby"]          = bool(data.get("standby", False))
        log.info(f"State restored: setpoint={state['setpoint']}, heater={state['heater_enabled']}, valve={state['valve_position']}, standby={state['standby']}")
    except Exception as e:
        log.warning(f"Failed to load state: {e}")

# -------------------------------------------------------
# Temperature Sensor
# -------------------------------------------------------

def find_temp_sensor():
    if "XXXXXXXXXXXX" not in TEMP_SENSOR_PATH:
        return TEMP_SENSOR_PATH
    devices = glob.glob("/sys/bus/w1/devices/28-*/w1_slave")
    if devices:
        log.info(f"Auto-discovered temp sensor: {devices[0]}")
        return devices[0]
    return None

SENSOR_PATH = None

def read_temperature():
    global SENSOR_PATH
    if SENSOR_PATH is None:
        SENSOR_PATH = find_temp_sensor()
    if SENSOR_PATH is None:
        return None
    try:
        with open(SENSOR_PATH, "r") as f:
            lines = f.readlines()
        if len(lines) >= 2 and "YES" in lines[0]:
            temp_c = float(lines[1].split("t=")[1]) / 1000.0
            temp_f = (temp_c * 9 / 5) + 32
            return round(temp_f, 1)
    except Exception as e:
        log.warning(f"Temp sensor read error: {e}")
    return None

def read_cpu_temp():
    """Read RPi CPU/SoC temperature in Fahrenheit."""
    try:
        with open(CPU_TEMP_PATH, "r") as f:
            millideg_c = int(f.read().strip())
        temp_c = millideg_c / 1000.0
        temp_f = (temp_c * 9 / 5) + 32
        return round(temp_f, 1)
    except Exception as e:
        log.warning(f"CPU temp read error: {e}")
        return None

# -------------------------------------------------------
# GPIO — libgpiod v2
# -------------------------------------------------------

_output_lines = None
_chip = None

def setup_gpio():
    """Initialize GPIO using libgpiod v2."""
    global _chip, _output_lines

    _chip = gpiod.Chip(GPIO_CHIP)

    _output_lines = _chip.request_lines(
        config={
            PIN_HEATER_RELAY: gpiod.LineSettings(
                direction=Direction.OUTPUT,
                output_value=Value.INACTIVE
            ),
            PIN_VALVE_OPEN: gpiod.LineSettings(
                direction=Direction.OUTPUT,
                output_value=Value.INACTIVE
            ),
            PIN_VALVE_CLOSE: gpiod.LineSettings(
                direction=Direction.OUTPUT,
                output_value=Value.INACTIVE
            ),
            PIN_VALVE_B_OPEN: gpiod.LineSettings(
                direction=Direction.OUTPUT,
                output_value=Value.INACTIVE
            ),
            PIN_VALVE_B_CLOSE: gpiod.LineSettings(
                direction=Direction.OUTPUT,
                output_value=Value.INACTIVE
            ),
        },
        consumer="pool-outputs"
    )

    threading.Thread(target=_monitor_encoder,    daemon=True).start()
    threading.Thread(target=_monitor_encoder_sw, daemon=True).start()
    threading.Thread(target=_monitor_buttons,    daemon=True).start()

    log.info("GPIO initialized")

def _set_output(pin: int, value: bool):
    _output_lines.set_value(pin, Value.ACTIVE if value else Value.INACTIVE)

def _monitor_encoder():
    """Monitor KY-040 rotary encoder CLK/DT using edge detection."""
    with gpiod.request_lines(
        GPIO_CHIP,
        consumer="pool-encoder",
        config={
            PIN_ENCODER_CLK: gpiod.LineSettings(
                edge_detection=Edge.FALLING,
                bias=Bias.PULL_UP,
                debounce_period=datetime.timedelta(milliseconds=5)
            ),
            PIN_ENCODER_DT: gpiod.LineSettings(
                direction=Direction.INPUT,
                bias=Bias.PULL_UP,
            ),
        }
    ) as enc_lines:
        log.info("Encoder monitoring started")
        while True:
            for event in enc_lines.read_edge_events():
                if event.line_offset == PIN_ENCODER_CLK:
                    dt = enc_lines.get_value(PIN_ENCODER_DT) == Value.ACTIVE
                    if dt:
                        encoder_cw()
                    else:
                        encoder_ccw()

def _monitor_encoder_sw():
    """Monitor encoder push button — short press toggles heater, long press (3s) toggles standby."""
    with gpiod.request_lines(
        GPIO_CHIP,
        consumer="pool-encoder-sw",
        config={
            PIN_ENCODER_SW: gpiod.LineSettings(
                edge_detection=Edge.BOTH,
                bias=Bias.PULL_UP,
                debounce_period=datetime.timedelta(milliseconds=50)
            ),
        }
    ) as sw_lines:
        log.info("Encoder button monitoring started")
        press_time = None
        while True:
            for event in sw_lines.read_edge_events():
                if event.event_type == gpiod.EdgeEvent.Type.FALLING_EDGE:
                    press_time = time.monotonic()
                elif event.event_type == gpiod.EdgeEvent.Type.RISING_EDGE:
                    if press_time is not None:
                        held = time.monotonic() - press_time
                        press_time = None
                        if held >= 3.0:
                            toggle_standby()
                        else:
                            encoder_sw_callback()

def _monitor_buttons():
    """Monitor Pool and Spa buttons using edge detection."""
    with gpiod.request_lines(
        GPIO_CHIP,
        consumer="pool-buttons",
        config={
            PIN_BTN_POOL: gpiod.LineSettings(
                edge_detection=Edge.FALLING,
                bias=Bias.PULL_UP,
                debounce_period=datetime.timedelta(milliseconds=300)
            ),
            PIN_BTN_SPA: gpiod.LineSettings(
                edge_detection=Edge.FALLING,
                bias=Bias.PULL_UP,
                debounce_period=datetime.timedelta(milliseconds=300)
            ),
        }
    ) as btn_lines:
        log.info("Button monitoring started")
        while True:
            for event in btn_lines.read_edge_events():
                if event.line_offset == PIN_BTN_POOL:
                    btn_pool_pressed()
                elif event.line_offset == PIN_BTN_SPA:
                    btn_spa_pressed()

# -------------------------------------------------------
# Relay Control
# -------------------------------------------------------

def set_heater_relay(on: bool):
    """Set heater relay state. Only writes/logs when the state actually changes."""
    if state["heater_relay_on"] == on:
        return  # Already in this state — skip redundant GPIO write and log
    state["heater_relay_on"] = on
    _set_output(PIN_HEATER_RELAY, on)
    log.info(f"Heater relay: {'ON' if on else 'OFF'}")

def _move_single_valve(valve: str, position: str):
    """
    Move one valve actuator to position.
      valve:    'a' or 'b'
      position: 'pool' or 'spa'
    Runs the relay pulse in a background thread.
    Returns False immediately if the actuator is already moving.

    Direction wiring (GVA-24):
      White wire = SPA  direction → OPEN  relay (PIN_VALVE_OPEN / PIN_VALVE_B_OPEN)
      Red   wire = POOL direction → CLOSE relay (PIN_VALVE_CLOSE / PIN_VALVE_B_CLOSE)
    """
    global _valve_a_moving, _valve_b_moving

    if valve == 'a':
        open_pin  = PIN_VALVE_OPEN
        close_pin = PIN_VALVE_CLOSE
        lock      = _valve_a_lock
        pos_key   = 'valve_a_position'
    else:
        open_pin  = PIN_VALVE_B_OPEN
        close_pin = PIN_VALVE_B_CLOSE
        lock      = _valve_b_lock
        pos_key   = 'valve_b_position'

    # Acquire lock before checking/setting moving flag to prevent race condition
    if not lock.acquire(blocking=False):
        log.warning(f"Valve {valve.upper()} move to {position} ignored — already moving")
        return False

    currently_moving = _valve_a_moving if valve == 'a' else _valve_b_moving
    if currently_moving:
        lock.release()
        log.warning(f"Valve {valve.upper()} move to {position} ignored — already moving")
        return False

    if position == state[pos_key]:
        lock.release()
        log.info(f"Valve {valve.upper()} already in {position} — no movement needed")
        return True

    # Set moving flag immediately while holding lock, before spawning thread
    if valve == 'a':
        _valve_a_moving = True
    else:
        _valve_b_moving = True
    lock.release()

    def _do_move():
        global _valve_a_moving, _valve_b_moving
        log.info(f"Valve {valve.upper()} moving to: {position}")
        state_topic = f"pool/state/valve_{'a' if valve == 'a' else 'b'}_position"
        if mqtt_connected:
            mqtt_client.publish(state_topic, "transitioning", retain=True)
        update_display()

        try:
            _set_output(open_pin,  False)
            _set_output(close_pin, False)
            time.sleep(0.1)
            if position == "pool":
                _set_output(close_pin, True)   # Red wire = POOL direction
            else:
                _set_output(open_pin, True)    # White wire = SPA direction
            time.sleep(VALVE_TRAVEL_TIME)
        finally:
            _set_output(open_pin,  False)
            _set_output(close_pin, False)
            if valve == 'a':
                _valve_a_moving = False
            else:
                _valve_b_moving = False

        state[pos_key] = position
        # Update combined valve_position
        if state['valve_a_position'] == state['valve_b_position']:
            state['valve_position'] = state['valve_a_position']
        else:
            state['valve_position'] = 'split'
        log.info(f"Valve {valve.upper()} move complete: {position}")
        save_state()
        publish_state()
        update_display()

    threading.Thread(target=_do_move, daemon=True).start()
    return True


def set_valve(position: str):
    """Move BOTH valve actuators to position sequentially (Pool/Spa button behavior).
    Spa mode:  Return (B) moves first, then Suction (A) — prevents hot tub level drop.
    Pool mode: Suction (A) moves first, then Return (B).
    Blocked if a sequence is already in progress.
    """
    global _valve_sequence_moving

    if position not in ("pool", "spa"):
        log.warning(f"Invalid valve position: {position}")
        return

    if not VALVE_ACTUATORS_CONNECTED:
        state["valve_position"]   = position
        state["valve_a_position"] = position
        state["valve_b_position"] = position
        log.info(f"Valve position set to: {position} (software state only)")
        save_state()
        publish_state()
        update_display()
        return

    if _valve_sequence_moving:
        log.warning(f"Valve sequence already in progress — {position} request ignored")
        return

    timeout_secs = VALVE_TRAVEL_TIME + 10  # Travel time + 10s safety buffer

    def _sequential_move():
        global _valve_sequence_moving
        _valve_sequence_moving = True
        try:
            if position == "spa":
                # Spa: Return (B) first, then Suction (A)
                log.info("Sequential move to SPA: B (return) first, then A (suction)")
                _move_single_valve('b', position)
                time.sleep(0.2)  # Allow thread to start and set moving flag
                deadline = time.monotonic() + timeout_secs
                while _valve_b_moving and time.monotonic() < deadline:
                    time.sleep(0.5)
                if _valve_b_moving:
                    log.error("Valve B timed out during SPA sequence — aborting")
                    return
                _move_single_valve('a', position)
            else:
                # Pool: Suction (A) first, then Return (B)
                log.info("Sequential move to POOL: A (suction) first, then B (return)")
                _move_single_valve('a', position)
                time.sleep(0.2)  # Allow thread to start and set moving flag
                deadline = time.monotonic() + timeout_secs
                while _valve_a_moving and time.monotonic() < deadline:
                    time.sleep(0.5)
                if _valve_a_moving:
                    log.error("Valve A timed out during POOL sequence — aborting")
                    return
                _move_single_valve('b', position)
        finally:
            _valve_sequence_moving = False

    threading.Thread(target=_sequential_move, daemon=True).start()


def move_valve_a(position: str):
    """Move Valve A only — HA individual control, no heater side effects."""
    if _valve_sequence_moving:
        log.warning("Valve sequence in progress — valve_a command ignored")
        return
    if not VALVE_ACTUATORS_CONNECTED:
        state["valve_a_position"] = position
        if state["valve_a_position"] == state["valve_b_position"]:
            state["valve_position"] = position
        else:
            state["valve_position"] = "split"
        save_state()
        publish_state()
        update_display()
        return
    _move_single_valve('a', position)


def move_valve_b(position: str):
    """Move Valve B only — HA individual control, no heater side effects."""
    if _valve_sequence_moving:
        log.warning("Valve sequence in progress — valve_b command ignored")
        return
    if not VALVE_ACTUATORS_CONNECTED:
        state["valve_b_position"] = position
        if state["valve_a_position"] == state["valve_b_position"]:
            state["valve_position"] = position
        else:
            state["valve_position"] = "split"
        save_state()
        publish_state()
        update_display()
        return
    _move_single_valve('b', position)

# -------------------------------------------------------
# Control Loop
# -------------------------------------------------------

def control_loop():
    while True:
        try:
            state["cpu_temp"] = read_cpu_temp()
            temp = read_temperature()
            if temp is None:
                if state["sensor_unavailable_since"] is None:
                    state["sensor_unavailable_since"] = time.time()
                    log.warning("Temp sensor unavailable — watchdog started")
                elif state["sensor_unavailable_since"] != -1 and \
                     time.time() - state["sensor_unavailable_since"] > SENSOR_UNAVAILABLE_TIMEOUT:
                    log.error("Sensor unavailable > 3 min — forcing heater off")
                    set_heater_relay(False)
                    state["sensor_unavailable_since"] = -1
                mqtt_client.publish("pool/sensor/water_temp", "unavailable", retain=True)
                publish_state()
                update_display()
            else:
                if state["sensor_unavailable_since"] is not None:
                    log.info("Temp sensor recovered")
                    state["sensor_unavailable_since"] = None
                state["water_temp"] = temp
                if state["heater_enabled"] and not state["standby"]:
                    if temp < (state["setpoint"] - HYSTERESIS):
                        set_heater_relay(True)
                    elif temp > (state["setpoint"] + HYSTERESIS):
                        set_heater_relay(False)
                else:
                    set_heater_relay(False)
                publish_state()
                update_display()
        except Exception as e:
            # Never let an unhandled exception kill the control thread —
            # that would silently stop heater regulation. Log and continue.
            log.error(f"Control loop iteration error: {e}", exc_info=True)
        time.sleep(CONTROL_LOOP_INTERVAL)

# -------------------------------------------------------
# MQTT — Publish
# -------------------------------------------------------

def publish_state():
    if not mqtt_connected:
        return
    msgs = {
        "pool/state/heater_enabled":  "ON"  if state["heater_enabled"]  else "OFF",
        "pool/state/heater_relay":    "ON"  if state["heater_relay_on"] else "OFF",
        "pool/sensor/setpoint":       str(state["setpoint"]),
        "pool/state/valve_position":  state["valve_position"],
        "pool/state/valve_a_position": state["valve_a_position"],
        "pool/state/valve_b_position": state["valve_b_position"],
        "pool/state/standby":         "ON"  if state["standby"]         else "OFF",
    }
    if state["water_temp"] is not None:
        msgs["pool/sensor/water_temp"] = str(state["water_temp"])
    if state["cpu_temp"] is not None:
        msgs["pool/sensor/cpu_temp"] = str(state["cpu_temp"])
    for topic, payload in msgs.items():
        mqtt_client.publish(topic, payload, retain=True)

def publish_discovery():
    device = {
        "identifiers": ["pool_controller"],
        "name":         "Pool Controller",
        "model":        "RPi Pool Controller",
        "manufacturer": "Custom Build",
    }
    entities = [
        ("homeassistant/sensor/pool_water_temp/config", {
            "name": "Pool Water Temp", "state_topic": "pool/sensor/water_temp",
            "unit_of_measurement": "°F", "device_class": "temperature",
            "unique_id": "pool_water_temp_01", "device": device,
        }),
        ("homeassistant/sensor/pool_cpu_temp/config", {
            "name": "Pool Controller CPU Temp", "state_topic": "pool/sensor/cpu_temp",
            "unit_of_measurement": "°F", "device_class": "temperature",
            "entity_category": "diagnostic",
            "unique_id": "pool_cpu_temp_01", "device": device,
        }),
        ("homeassistant/number/pool_setpoint/config", {
            "name": "Pool Setpoint", "state_topic": "pool/sensor/setpoint",
            "command_topic": "pool/cmd/setpoint",
            "min": SETPOINT_MIN, "max": SETPOINT_MAX, "step": 1,
            "unit_of_measurement": "°F", "unique_id": "pool_setpoint_01", "device": device,
        }),
        ("homeassistant/switch/pool_heater_enabled/config", {
            "name": "Pool Heater", "state_topic": "pool/state/heater_enabled",
            "command_topic": "pool/cmd/heater_enabled",
            "payload_on": "ON", "payload_off": "OFF",
            "unique_id": "pool_heater_enabled_01", "device": device,
        }),
        ("homeassistant/binary_sensor/pool_heater_relay/config", {
            "name": "Pool Heating", "state_topic": "pool/state/heater_relay",
            "payload_on": "ON", "payload_off": "OFF",
            "unique_id": "pool_heater_relay_01", "device": device,
        }),
        ("homeassistant/select/pool_valve_position/config", {
            "name": "Pool Valves (Both)", "state_topic": "pool/state/valve_position",
            "command_topic": "pool/cmd/valve",
            "options": ["pool", "spa", "transitioning", "split"],
            "unique_id": "pool_valve_position_01", "device": device,
        }),
        ("homeassistant/select/pool_valve_a/config", {
            "name": "Pool Valve A (Suction)", "state_topic": "pool/state/valve_a_position",
            "command_topic": "pool/cmd/valve_a",
            "options": ["pool", "spa", "transitioning"],
            "unique_id": "pool_valve_a_01", "device": device,
        }),
        ("homeassistant/select/pool_valve_b/config", {
            "name": "Pool Valve B (Return)", "state_topic": "pool/state/valve_b_position",
            "command_topic": "pool/cmd/valve_b",
            "options": ["pool", "spa", "transitioning"],
            "unique_id": "pool_valve_b_01", "device": device,
        }),
        ("homeassistant/switch/pool_standby/config", {
            "name": "Pool Standby", "state_topic": "pool/state/standby",
            "command_topic": "pool/cmd/standby",
            "payload_on": "ON", "payload_off": "OFF",
            "icon": "mdi:power-standby",
            "unique_id": "pool_standby_01", "device": device,
        }),
        ("homeassistant/button/pool_mode/config", {
            "name": "Pool Mode", "command_topic": "pool/cmd/mode",
            "payload_press": "pool",
            "icon": "mdi:pool",
            "unique_id": "pool_mode_button_01", "device": device,
        }),
        ("homeassistant/button/spa_mode/config", {
            "name": "Spa Mode", "command_topic": "pool/cmd/mode",
            "payload_press": "spa",
            "icon": "mdi:hot-tub",
            "unique_id": "spa_mode_button_01", "device": device,
        }),
    ]
    # Clear stale binary_sensor standby discovery (replaced by switch in newer version)
    mqtt_client.publish("homeassistant/binary_sensor/pool_standby/config", "", retain=True)

    for topic, payload in entities:
        mqtt_client.publish(topic, json.dumps(payload), retain=True)
        log.info(f"Discovery published: {topic}")

# -------------------------------------------------------
# MQTT — Receive Commands
# -------------------------------------------------------

def on_message(client, userdata, msg):
    topic   = msg.topic
    payload = msg.payload.decode().strip()
    log.info(f"MQTT command: {topic} = {payload}")
    if topic == "pool/cmd/heater_enabled":
        if state["standby"]:
            log.warning("Standby active — heater enable command ignored")
            publish_state()
            return
        state["heater_enabled"] = (payload == "ON")
        if not state["heater_enabled"]:
            set_heater_relay(False)
        save_state()
        publish_state()
        update_display()
    elif topic == "pool/cmd/setpoint":
        try:
            val = float(payload)
            state["setpoint"] = max(SETPOINT_MIN, min(SETPOINT_MAX, val))
            save_state()
            publish_state()
            update_display()
        except ValueError:
            log.warning(f"Invalid setpoint value: {payload}")
    elif topic == "pool/cmd/valve":
        if payload in ("pool", "spa"):
            set_valve(payload)
        else:
            log.warning(f"Invalid valve command: {payload}")
    elif topic == "pool/cmd/valve_a":
        if payload in ("pool", "spa"):
            move_valve_a(payload)
        else:
            log.warning(f"Invalid valve_a command: {payload}")
    elif topic == "pool/cmd/valve_b":
        if payload in ("pool", "spa"):
            move_valve_b(payload)
        else:
            log.warning(f"Invalid valve_b command: {payload}")
    elif topic == "pool/cmd/standby":
        desired = (payload == "ON")
        if desired != state["standby"]:
            toggle_standby()
        else:
            publish_state()
    elif topic == "pool/cmd/mode":
        if payload == "pool":
            btn_pool_pressed()
        elif payload == "spa":
            btn_spa_pressed()
        else:
            log.warning(f"Invalid mode command: {payload}")
    elif topic == "pool/schedule/pump_should_run":
        state["pump_is_on"] = (payload.lower() == "on")
        log.info(f"Pump is on: {state['pump_is_on']}")
    elif topic == "pool/schedule/pump_in_schedule":
        state["pump_should_run"] = (payload.lower() == "on")
        log.info(f"Pump in schedule: {state['pump_should_run']}")

def on_connect(client, userdata, flags, reason_code, properties):
    global mqtt_connected
    if reason_code == 0:
        mqtt_connected = True
        log.info(f"Connected to MQTT broker at {BROKER_IP}:{BROKER_PORT}")
        client.subscribe("pool/cmd/#")
        client.subscribe("pool/schedule/pump_should_run")
        client.subscribe("pool/schedule/pump_in_schedule")
        publish_discovery()
        publish_state()
    else:
        log.error(f"MQTT connection failed, reason={reason_code}")

def on_disconnect(client, userdata, flags, reason_code, properties):
    global mqtt_connected
    mqtt_connected = False
    log.warning(f"MQTT disconnected, reason={reason_code} — will retry")

# -------------------------------------------------------
# OLED Display
# -------------------------------------------------------

try:
    font_small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 11)
    font_large = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 24)
except IOError:
    font_small = ImageFont.load_default()
    font_large = ImageFont.load_default()

display_device = None

def init_display():
    global display_device
    try:
        serial = i2c(port=1, address=0x3C)
        display_device = sh1106(serial)
        log.info("OLED display initialized")
    except Exception as e:
        log.error(f"OLED init failed: {e}")
        display_device = None

def center_x(text, font, width=128):
    try:
        bbox = font.getbbox(text)
        text_width = bbox[2] - bbox[0]
    except AttributeError:
        text_width = len(text) * 7
    return max(0, (width - text_width) // 2)

def update_display():
    if display_device is None:
        return
    if state["standby"]:
        mode_text = "** STANDBY **"
    elif _valve_a_moving or _valve_b_moving:
        mode_text = "Moving..."
    elif state["valve_a_position"] == state["valve_b_position"] == "pool":
        mode_text = "Pool Mode"
    elif state["valve_a_position"] == state["valve_b_position"] == "spa":
        mode_text = "Spa Mode"
    else:
        mode_text = "Split Mode"
    heater_str  = "ON"  if state["heater_enabled"]  else "OFF"
    heating_str = "YES" if state["heater_relay_on"] else "NO"
    status_text = f"Heater:{heater_str}  Heat:{heating_str}"
    current  = f"{state['water_temp']:.0f}\u00b0" if state["water_temp"] is not None else "---\u00b0"
    setpoint = f"{state['setpoint']:.0f}\u00b0"
    cur_w   = font_large.getbbox(current)[2]
    set_w   = font_large.getbbox(setpoint)[2]
    arr_w   = font_small.getbbox("\u2192")[2]
    total_w = cur_w + arr_w + set_w + 6
    start_x = max(0, (128 - total_w) // 2)
    cur_x   = start_x
    arr_x   = cur_x + cur_w + 3
    set_x   = arr_x + arr_w + 3
    try:
        with canvas(display_device) as draw:
            draw.text((center_x(mode_text,   font_small), 0),  mode_text,   font=font_small, fill="white")
            draw.text((center_x(status_text, font_small), 14), status_text, font=font_small, fill="white")
            draw.text((cur_x, 36), current,  font=font_large, fill="white")
            draw.text((arr_x, 40), "\u2192", font=font_small, fill="white")
            draw.text((set_x, 36), setpoint, font=font_large, fill="white")
    except Exception as e:
        log.error(f"Display update error: {e}")

# -------------------------------------------------------
# Encoder / Button Callbacks
# -------------------------------------------------------

def encoder_cw():
    if state["standby"]:
        return
    state["setpoint"] = min(SETPOINT_MAX, state["setpoint"] + 1)
    log.info(f"Setpoint adjusted to {state['setpoint']}°F")
    save_state()
    publish_state()
    update_display()

def encoder_ccw():
    if state["standby"]:
        return
    state["setpoint"] = max(SETPOINT_MIN, state["setpoint"] - 1)
    log.info(f"Setpoint adjusted to {state['setpoint']}°F")
    save_state()
    publish_state()
    update_display()

def encoder_sw_callback():
    if state["standby"]:
        return
    state["heater_enabled"] = not state["heater_enabled"]
    if not state["heater_enabled"]:
        set_heater_relay(False)
    log.info(f"Heater toggled: {'ON' if state['heater_enabled'] else 'OFF'}")
    save_state()
    publish_state()
    update_display()

def toggle_standby():
    state["standby"] = not state["standby"]
    if state["standby"]:
        state["heater_enabled"] = False
        set_heater_relay(False)
        log.info("Standby mode ON — heater disabled, physical controls locked")
    else:
        log.info("Standby mode OFF — normal operation resumed")
    save_state()
    publish_state()
    update_display()

def btn_pool_pressed():
    global _pending_pump_off
    if state["standby"]:
        return
    if _valve_sequence_moving:
        log.warning("Valve sequence in progress — pool button ignored")
        return
    log.info("Pool button pressed")
    state["heater_enabled"] = False
    state["setpoint"] = 80.0
    set_heater_relay(False)
    if mqtt_connected and not state["pump_should_run"]:
        # Delay pump-off 30s to flush hot water from heater past the temp sensor
        _pending_pump_off = True
        log.info("Pool mode: pump off delayed 30s to flush heater backwash")
        def _delayed_pump_off():
            global _pending_pump_off
            time.sleep(30)
            if _pending_pump_off and mqtt_connected:
                mqtt_client.publish("pool/cmd/pump", "OFF", retain=False)
                log.info("Pool mode: delayed pump off command sent")
            _pending_pump_off = False
        threading.Thread(target=_delayed_pump_off, daemon=True).start()
    else:
        log.info("Pool mode: within pump schedule — pump left running")
    set_valve("pool")

def btn_spa_pressed():
    global _pending_pump_off
    if state["standby"]:
        return
    if _valve_sequence_moving:
        log.warning("Valve sequence in progress — spa button ignored")
        return
    if _pending_pump_off:
        _pending_pump_off = False
        log.info("Spa mode: cancelled pending pool pump-off")
    log.info("Spa button pressed")
    state["heater_enabled"] = True
    state["setpoint"] = 100.0
    if mqtt_connected:
        mqtt_client.publish("pool/cmd/pump", "ON", retain=False)
        log.info("Spa mode: heater on, setpoint 100, pump on command sent")
    set_valve("spa")

# -------------------------------------------------------
# Main
# -------------------------------------------------------

def main():
    log.info("Pool Controller starting...")
    load_state()
    setup_gpio()
    init_display()
    update_display()

    mqtt_client.username_pw_set(BROKER_USER, BROKER_PASS)
    mqtt_client.on_connect    = on_connect
    mqtt_client.on_disconnect = on_disconnect
    mqtt_client.on_message    = on_message
    mqtt_client.reconnect_delay_set(min_delay=5, max_delay=60)

    try:
        mqtt_client.connect(BROKER_IP, BROKER_PORT, keepalive=60)
    except Exception as e:
        log.warning(f"Initial MQTT connect failed: {e} — will retry in background")

    mqtt_client.loop_start()
    threading.Thread(target=control_loop, daemon=True).start()
    log.info("Pool Controller running")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        log.info("Shutting down...")
    except Exception as e:
        log.error(f"Fatal error in main loop: {e}", exc_info=True)
    finally:
        # Fail-safe: force all relays OFF on ANY exit (clean, interrupt, or crash).
        # Write pins directly rather than via set_heater_relay(), which may early-return
        # if in-memory state is stale after a crash.
        try:
            if _output_lines:
                _set_output(PIN_HEATER_RELAY,  False)
                _set_output(PIN_VALVE_OPEN,    False)
                _set_output(PIN_VALVE_CLOSE,   False)
                _set_output(PIN_VALVE_B_OPEN,  False)
                _set_output(PIN_VALVE_B_CLOSE, False)
                log.info("Fail-safe: all relays forced OFF")
        except Exception as e:
            log.error(f"Error forcing relays off during shutdown: {e}")
        mqtt_client.loop_stop()
        if _output_lines:
            _output_lines.release()
        if _chip:
            _chip.close()
        log.info("Shutdown complete")

if __name__ == "__main__":
    main()
