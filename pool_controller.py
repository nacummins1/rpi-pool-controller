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
  - Valve relay B (close)   GPIO 13  (Stage 3)
  - OLED display            GPIO 2 (SDA), GPIO 3 (SCL) - I2C, kernel managed

MQTT Broker: 172.30.33.1:1883
"""

import time
import json
import threading
import logging
import glob
import os

from gpiozero import OutputDevice, Button, RotaryEncoder
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

# DS18B20 — update with actual serial after first boot:
#   ls /sys/bus/w1/devices/  → find your 28-xxxx device
TEMP_SENSOR_PATH = "/sys/bus/w1/devices/28-XXXXXXXXXXXX/w1_slave"

SETPOINT_MIN     = 65.0
SETPOINT_MAX     = 104.0
SETPOINT_DEFAULT = 80.0

SENSOR_UNAVAILABLE_TIMEOUT = 180   # seconds before heater forced off
CONTROL_LOOP_INTERVAL      = 10    # seconds between temp checks
HYSTERESIS                 = 2.0   # ±2°F band

# Stage 3 — set to True when valve actuators are physically connected
VALVE_ACTUATORS_CONNECTED = False

# Persistent state file — survives reboots
STATE_FILE = "/home/pi/pool_state.json"

# -------------------------------------------------------
# GPIO Pin Assignments
# -------------------------------------------------------

# GPIO 4 reserved — DS18B20 1-Wire (kernel managed via dtoverlay=w1-gpio)

PIN_HEATER_RELAY    = 17
PIN_ENCODER_CLK     = 23
PIN_ENCODER_DT      = 24
PIN_ENCODER_SW      = 22   # Push button — heater toggle
PIN_BTN_POOL        = 5
PIN_BTN_SPA         = 6
PIN_VALVE_OPEN      = 27   # Stage 3
PIN_VALVE_CLOSE     = 13   # Stage 3

# -------------------------------------------------------
# State
# -------------------------------------------------------

state = {
    "heater_enabled":           False,
    "heater_relay_on":          False,
    "water_temp":               None,
    "setpoint":                 SETPOINT_DEFAULT,
    "valve_position":           "pool",
    "sensor_unavailable_since": None,
}

# -------------------------------------------------------
# MQTT Client
# -------------------------------------------------------

mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqtt_connected = False

# -------------------------------------------------------
# Persistent State
# -------------------------------------------------------

def save_state():
    """Save persistent state to disk — survives reboots."""
    data = {
        "setpoint":       state["setpoint"],
        "heater_enabled": state["heater_enabled"],
        "valve_position": state["valve_position"],
    }
    try:
        with open(STATE_FILE, "w") as f:
            json.dump(data, f)
    except Exception as e:
        log.warning(f"Failed to save state: {e}")

def load_state():
    """Load persistent state from disk on startup."""
    if not os.path.exists(STATE_FILE):
        log.info("No saved state found — using defaults")
        return
    try:
        with open(STATE_FILE, "r") as f:
            data = json.load(f)
        state["setpoint"]       = float(data.get("setpoint", SETPOINT_DEFAULT))
        state["heater_enabled"] = bool(data.get("heater_enabled", False))
        state["valve_position"] = data.get("valve_position", "pool")
        log.info(f"State restored: setpoint={state['setpoint']}, heater={state['heater_enabled']}, valve={state['valve_position']}")
    except Exception as e:
        log.warning(f"Failed to load state: {e}")

# -------------------------------------------------------
# Temperature Sensor
# -------------------------------------------------------

def find_temp_sensor():
    """Auto-discover DS18B20 sensor path if placeholder not yet updated."""
    if "XXXXXXXXXXXX" not in TEMP_SENSOR_PATH:
        return TEMP_SENSOR_PATH
    devices = glob.glob("/sys/bus/w1/devices/28-*/w1_slave")
    if devices:
        log.info(f"Auto-discovered temp sensor: {devices[0]}")
        return devices[0]
    return None

SENSOR_PATH = None

def read_temperature():
    """Read DS18B20 temperature in Fahrenheit. Returns None on failure."""
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

# -------------------------------------------------------
# GPIO Devices (gpiozero)
# -------------------------------------------------------

heater_relay = None
valve_open   = None
valve_close  = None
encoder      = None
encoder_btn  = None
btn_pool     = None
btn_spa      = None

def setup_gpio():
    """Initialize all GPIO devices using gpiozero."""
    global heater_relay, valve_open, valve_close
    global encoder, encoder_btn, btn_pool, btn_spa

    # Outputs
    heater_relay = OutputDevice(PIN_HEATER_RELAY, active_high=True, initial_value=False)
    valve_open   = OutputDevice(PIN_VALVE_OPEN,   active_high=True, initial_value=False)
    valve_close  = OutputDevice(PIN_VALVE_CLOSE,  active_high=True, initial_value=False)

    # Rotary encoder
    encoder = RotaryEncoder(PIN_ENCODER_CLK, PIN_ENCODER_DT, max_steps=0)
    encoder.when_rotated_clockwise         = encoder_cw
    encoder.when_rotated_counter_clockwise = encoder_ccw

    # Encoder push button
    encoder_btn = Button(PIN_ENCODER_SW, pull_up=True, bounce_time=0.3)
    encoder_btn.when_pressed = encoder_sw_callback

    # Pool/Spa buttons
    btn_pool = Button(PIN_BTN_POOL, pull_up=True, bounce_time=0.3)
    btn_spa  = Button(PIN_BTN_SPA,  pull_up=True, bounce_time=0.3)
    btn_pool.when_pressed = btn_pool_pressed
    btn_spa.when_pressed  = btn_spa_pressed

    log.info("GPIO initialized")

# -------------------------------------------------------
# Relay Control
# -------------------------------------------------------

def set_heater_relay(on: bool):
    """Set heater relay state. Always writes to GPIO to prevent sync issues.
    Caller is responsible for publish_state() and update_display()."""
    state["heater_relay_on"] = on
    if on:
        heater_relay.on()
    else:
        heater_relay.off()
    log.info(f"Heater relay: {'ON' if on else 'OFF'}")

def set_valve(position: str):
    """Set valve position. Stage 3: drives relay when VALVE_ACTUATORS_CONNECTED = True."""
    if position not in ("pool", "spa"):
        log.warning(f"Invalid valve position: {position}")
        return
    state["valve_position"] = position
    log.info(f"Valve position set to: {position}")
    if VALVE_ACTUATORS_CONNECTED:
        # Stage 3 — drive relay
        # TODO: implement actuator timing when hardware connected
        pass
    else:
        log.info("Valve actuators not connected — software state only")
    save_state()
    publish_state()
    update_display()

# -------------------------------------------------------
# Control Loop
# -------------------------------------------------------

def control_loop():
    """Main heater control loop. Runs every CONTROL_LOOP_INTERVAL seconds."""
    while True:
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

            if state["heater_enabled"]:
                if temp < (state["setpoint"] - HYSTERESIS):
                    set_heater_relay(True)
                elif temp > (state["setpoint"] + HYSTERESIS):
                    set_heater_relay(False)
            else:
                set_heater_relay(False)

            publish_state()
            update_display()

        time.sleep(CONTROL_LOOP_INTERVAL)

# -------------------------------------------------------
# MQTT — Publish
# -------------------------------------------------------

def publish_state():
    """Publish current state to MQTT broker."""
    if not mqtt_connected:
        return
    msgs = {
        "pool/state/heater_enabled":  "ON"  if state["heater_enabled"]  else "OFF",
        "pool/state/heater_relay":    "ON"  if state["heater_relay_on"] else "OFF",
        "pool/sensor/setpoint":       str(state["setpoint"]),
        "pool/state/valve_position":  state["valve_position"],
    }
    if state["water_temp"] is not None:
        msgs["pool/sensor/water_temp"] = str(state["water_temp"])
    for topic, payload in msgs.items():
        mqtt_client.publish(topic, payload, retain=True)

def publish_discovery():
    """Publish MQTT discovery payloads — HA auto-creates entities on receipt."""
    device = {
        "identifiers": ["pool_controller"],
        "name":         "Pool Controller",
        "model":        "RPi Pool Controller",
        "manufacturer": "Custom Build",
    }
    entities = [
        ("homeassistant/sensor/pool_water_temp/config", {
            "name":                "Pool Water Temp",
            "state_topic":         "pool/sensor/water_temp",
            "unit_of_measurement": "°F",
            "device_class":        "temperature",
            "unique_id":           "pool_water_temp_01",
            "device":              device,
        }),
        ("homeassistant/number/pool_setpoint/config", {
            "name":                "Pool Setpoint",
            "state_topic":         "pool/sensor/setpoint",
            "command_topic":       "pool/cmd/setpoint",
            "min":                 SETPOINT_MIN,
            "max":                 SETPOINT_MAX,
            "step":                1,
            "unit_of_measurement": "°F",
            "unique_id":           "pool_setpoint_01",
            "device":              device,
        }),
        ("homeassistant/switch/pool_heater_enabled/config", {
            "name":          "Pool Heater",
            "state_topic":   "pool/state/heater_enabled",
            "command_topic": "pool/cmd/heater_enabled",
            "payload_on":    "ON",
            "payload_off":   "OFF",
            "unique_id":     "pool_heater_enabled_01",
            "device":        device,
        }),
        ("homeassistant/binary_sensor/pool_heater_relay/config", {
            "name":        "Pool Heating",
            "state_topic": "pool/state/heater_relay",
            "payload_on":  "ON",
            "payload_off": "OFF",
            "unique_id":   "pool_heater_relay_01",
            "device":      device,
        }),
        ("homeassistant/select/pool_valve_position/config", {
            "name":          "Pool Valve Position",
            "state_topic":   "pool/state/valve_position",
            "command_topic": "pool/cmd/valve",
            "options":       ["pool", "spa"],
            "unique_id":     "pool_valve_position_01",
            "device":        device,
        }),
    ]
    for topic, payload in entities:
        mqtt_client.publish(topic, json.dumps(payload), retain=True)
        log.info(f"Discovery published: {topic}")

# -------------------------------------------------------
# MQTT — Receive Commands
# -------------------------------------------------------

def on_message(client, userdata, msg):
    """Handle incoming MQTT commands from HA."""
    topic   = msg.topic
    payload = msg.payload.decode().strip()
    log.info(f"MQTT command: {topic} = {payload}")

    if topic == "pool/cmd/heater_enabled":
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

def on_connect(client, userdata, flags, reason_code, properties):
    """Called when MQTT connection is established."""
    global mqtt_connected
    if reason_code == 0:
        mqtt_connected = True
        log.info(f"Connected to MQTT broker at {BROKER_IP}:{BROKER_PORT}")
        client.subscribe("pool/cmd/#")
        publish_discovery()
        publish_state()
    else:
        log.error(f"MQTT connection failed, reason={reason_code}")

def on_disconnect(client, userdata, flags, reason_code, properties):
    """Called when MQTT connection is lost."""
    global mqtt_connected
    mqtt_connected = False
    log.warning(f"MQTT disconnected, reason={reason_code} — will retry")

# -------------------------------------------------------
# OLED Display
# -------------------------------------------------------

try:
    font_small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 11)
    font_large = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)
except IOError:
    font_small = ImageFont.load_default()
    font_large = ImageFont.load_default()

display_device = None

def init_display():
    """Initialize OLED display via I2C."""
    global display_device
    try:
        serial = i2c(port=1, address=0x3C)
        display_device = sh1106(serial)
        log.info("OLED display initialized")
    except Exception as e:
        log.error(f"OLED init failed: {e}")
        display_device = None

def center_x(text, font, width=128):
    """Calculate x offset to center text on display."""
    try:
        bbox = font.getbbox(text)
        text_width = bbox[2] - bbox[0]
    except AttributeError:
        text_width = len(text) * 7
    return max(0, (width - text_width) // 2)

def update_display():
    """Refresh OLED with current state."""
    if display_device is None:
        return
    mode_text   = "Pool Mode" if state["valve_position"] == "pool" else "Spa Mode"
    heater_str  = "ON"  if state["heater_enabled"]  else "OFF"
    heating_str = "YES" if state["heater_relay_on"] else "NO"
    status_text = f"Heater:{heater_str}  Heat:{heating_str}"
    current     = f"{state['water_temp']:.1f}\u00b0F" if state["water_temp"] is not None else "---\u00b0F"
    setpoint    = f"{state['setpoint']:.1f}\u00b0F"
    temp_text   = f"{current} \u2192 {setpoint}"
    try:
        with canvas(display_device) as draw:
            draw.text((center_x(mode_text,   font_small), 0),  mode_text,   font=font_small, fill="white")
            draw.text((center_x(status_text, font_small), 14), status_text, font=font_small, fill="white")
            draw.text((center_x(temp_text,   font_large), 36), temp_text,   font=font_large, fill="white")
    except Exception as e:
        log.error(f"Display update error: {e}")

# -------------------------------------------------------
# Rotary Encoder Callbacks
# -------------------------------------------------------

def encoder_cw():
    """Clockwise rotation — increase setpoint."""
    state["setpoint"] = min(SETPOINT_MAX, state["setpoint"] + 1)
    log.info(f"Setpoint adjusted to {state['setpoint']}°F")
    save_state()
    publish_state()
    update_display()

def encoder_ccw():
    """Counter-clockwise rotation — decrease setpoint."""
    state["setpoint"] = max(SETPOINT_MIN, state["setpoint"] - 1)
    log.info(f"Setpoint adjusted to {state['setpoint']}°F")
    save_state()
    publish_state()
    update_display()

def encoder_sw_callback():
    """Encoder push button — toggle heater enabled."""
    state["heater_enabled"] = not state["heater_enabled"]
    if not state["heater_enabled"]:
        set_heater_relay(False)
    log.info(f"Heater toggled: {'ON' if state['heater_enabled'] else 'OFF'}")
    save_state()
    publish_state()
    update_display()

# -------------------------------------------------------
# Pool / Spa Button Callbacks
# -------------------------------------------------------

def btn_pool_pressed():
    """Set valve position to Pool Mode."""
    log.info("Pool button pressed")
    set_valve("pool")

def btn_spa_pressed():
    """Set valve position to Spa Mode."""
    log.info("Spa button pressed")
    set_valve("spa")

# -------------------------------------------------------
# Main
# -------------------------------------------------------

def main():
    log.info("Pool Controller starting...")

    # Load saved state
    load_state()

    # GPIO
    setup_gpio()

    # OLED
    init_display()
    update_display()

    # MQTT
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
        set_heater_relay(False)
        mqtt_client.loop_stop()
        heater_relay.close()
        valve_open.close()
        valve_close.close()
        log.info("Shutdown complete")

if __name__ == "__main__":
    main()
