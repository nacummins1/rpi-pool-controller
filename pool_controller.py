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

import RPi.GPIO as GPIO
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

BROKER_IP   = "172.30.33.1"
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
    "heater_enabled":           False,       # Master heater on/off
    "heater_relay_on":          False,       # Actual relay state
    "water_temp":               None,        # Current temp reading
    "setpoint":                 SETPOINT_DEFAULT,
    "valve_position":           "pool",      # "pool" or "spa"
    "sensor_unavailable_since": None,
}

# -------------------------------------------------------
# MQTT Client
# -------------------------------------------------------

mqtt_client = mqtt.Client()
mqtt_connected = False

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

SENSOR_PATH = None   # Set on startup

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
# Relay Control
# -------------------------------------------------------

def set_heater_relay(on: bool):
    """Set heater relay state. No-op if state unchanged."""
    if state["heater_relay_on"] == on:
        return
    state["heater_relay_on"] = on
    GPIO.output(PIN_HEATER_RELAY, GPIO.HIGH if on else GPIO.LOW)
    log.info(f"Heater relay: {'ON' if on else 'OFF'}")
    publish_state()
    update_display()

def set_valve(position: str):
    """
    Set valve position.
    Stage 3: drives relay when VALVE_ACTUATORS_CONNECTED = True.
    """
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
            # Sensor unavailable
            if state["sensor_unavailable_since"] is None:
                state["sensor_unavailable_since"] = time.time()
                log.warning("Temp sensor unavailable — watchdog started")
            elif time.time() - state["sensor_unavailable_since"] > SENSOR_UNAVAILABLE_TIMEOUT:
                log.error("Sensor unavailable > 3 min — forcing heater off")
                set_heater_relay(False)
            # Publish unavailable state
            mqtt_client.publish("pool/sensor/water_temp", "unavailable", retain=True)
            update_display()

        else:
            # Sensor available — clear watchdog
            if state["sensor_unavailable_since"] is not None:
                log.info("Temp sensor recovered")
                state["sensor_unavailable_since"] = None

            state["water_temp"] = temp

            # Hysteresis logic
            if state["heater_enabled"]:
                if temp < (state["setpoint"] - HYSTERESIS):
                    set_heater_relay(True)
                elif temp > (state["setpoint"] + HYSTERESIS):
                    set_heater_relay(False)
                # Between band: hold current relay state
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
        # Water temp sensor
        ("homeassistant/sensor/pool_water_temp/config", {
            "name":                "Pool Water Temp",
            "state_topic":         "pool/sensor/water_temp",
            "unit_of_measurement": "°F",
            "device_class":        "temperature",
            "unique_id":           "pool_water_temp_01",
            "device":              device,
        }),
        # Setpoint number
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
        # Heater enabled switch
        ("homeassistant/switch/pool_heater_enabled/config", {
            "name":          "Pool Heater",
            "state_topic":   "pool/state/heater_enabled",
            "command_topic": "pool/cmd/heater_enabled",
            "payload_on":    "ON",
            "payload_off":   "OFF",
            "unique_id":     "pool_heater_enabled_01",
            "device":        device,
        }),
        # Heater relay binary sensor (read-only — actual relay state)
        ("homeassistant/binary_sensor/pool_heater_relay/config", {
            "name":        "Pool Heating",
            "state_topic": "pool/state/heater_relay",
            "payload_on":  "ON",
            "payload_off": "OFF",
            "unique_id":   "pool_heater_relay_01",
            "device":      device,
        }),
        # Valve position select
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
        publish_state()
        update_display()

    elif topic == "pool/cmd/setpoint":
        try:
            val = float(payload)
            state["setpoint"] = max(SETPOINT_MIN, min(SETPOINT_MAX, val))
            publish_state()
            update_display()
        except ValueError:
            log.warning(f"Invalid setpoint value: {payload}")

    elif topic == "pool/cmd/valve":
        if payload in ("pool", "spa"):
            set_valve(payload)
        else:
            log.warning(f"Invalid valve command: {payload}")

def on_connect(client, userdata, flags, rc):
    """Called when MQTT connection is established."""
    global mqtt_connected
    if rc == 0:
        mqtt_connected = True
        log.info(f"Connected to MQTT broker at {BROKER_IP}:{BROKER_PORT}")
        client.subscribe("pool/cmd/#")
        publish_discovery()
        publish_state()
    else:
        log.error(f"MQTT connection failed, rc={rc}")

def on_disconnect(client, userdata, rc):
    """Called when MQTT connection is lost."""
    global mqtt_connected
    mqtt_connected = False
    log.warning(f"MQTT disconnected, rc={rc} — will retry")

# -------------------------------------------------------
# OLED Display
# -------------------------------------------------------

# Load fonts — falls back to default if not available
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
        text_width = len(text) * 7   # Fallback estimate
    return max(0, (width - text_width) // 2)

def update_display():
    """Refresh OLED with current state."""
    if display_device is None:
        return

    # Mode line
    mode_text = "Pool Mode" if state["valve_position"] == "pool" else "Spa Mode"

    # Heater status line
    heater_str  = "ON" if state["heater_enabled"]  else "OFF"
    heating_str = "YES" if state["heater_relay_on"] else "NO"
    status_text = f"Heater:{heater_str}  Heat:{heating_str}"

    # Temp line
    current  = f"{state['water_temp']:.1f}°F" if state["water_temp"] is not None else "---°F"
    setpoint = f"{state['setpoint']:.1f}°F"
    temp_text = f"{current} → {setpoint}"

    try:
        with canvas(display_device) as draw:
            # Line 1 — Mode (small, centered)
            draw.text(
                (center_x(mode_text, font_small), 0),
                mode_text,
                font=font_small,
                fill="white"
            )
            # Line 2 — Heater status (small, centered)
            draw.text(
                (center_x(status_text, font_small), 14),
                status_text,
                font=font_small,
                fill="white"
            )
            # Lines 3/4 — Temps (large, centered)
            draw.text(
                (center_x(temp_text, font_large), 36),
                temp_text,
                font=font_large,
                fill="white"
            )
    except Exception as e:
        log.error(f"Display update error: {e}")

# -------------------------------------------------------
# GPIO Setup
# -------------------------------------------------------

def setup_gpio():
    """Initialize all GPIO pins."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Outputs
    GPIO.setup(PIN_HEATER_RELAY, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_VALVE_OPEN,   GPIO.OUT, initial=GPIO.LOW)   # Stage 3
    GPIO.setup(PIN_VALVE_CLOSE,  GPIO.OUT, initial=GPIO.LOW)   # Stage 3

    # Inputs — rotary encoder
    GPIO.setup(PIN_ENCODER_CLK, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(PIN_ENCODER_DT,  GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(PIN_ENCODER_SW,  GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Inputs — pool/spa buttons
    GPIO.setup(PIN_BTN_POOL, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(PIN_BTN_SPA,  GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Interrupts — rotary encoder
    GPIO.add_event_detect(
        PIN_ENCODER_CLK, GPIO.BOTH,
        callback=encoder_callback,
        bouncetime=5
    )
    GPIO.add_event_detect(
        PIN_ENCODER_SW, GPIO.FALLING,
        callback=encoder_sw_callback,
        bouncetime=300
    )

    # Interrupts — pool/spa buttons
    GPIO.add_event_detect(
        PIN_BTN_POOL, GPIO.FALLING,
        callback=btn_pool_pressed,
        bouncetime=300
    )
    GPIO.add_event_detect(
        PIN_BTN_SPA, GPIO.FALLING,
        callback=btn_spa_pressed,
        bouncetime=300
    )

    log.info("GPIO initialized")

# -------------------------------------------------------
# Rotary Encoder Callbacks
# -------------------------------------------------------

_last_clk_state = None

def encoder_callback(channel):
    """Handle rotary encoder rotation — adjust setpoint."""
    global _last_clk_state
    clk_state = GPIO.input(PIN_ENCODER_CLK)
    dt_state  = GPIO.input(PIN_ENCODER_DT)

    if clk_state == _last_clk_state:
        return  # No change

    if dt_state != clk_state:
        # Clockwise — increase setpoint
        state["setpoint"] = min(SETPOINT_MAX, state["setpoint"] + 1)
    else:
        # Counterclockwise — decrease setpoint
        state["setpoint"] = max(SETPOINT_MIN, state["setpoint"] - 1)

    _last_clk_state = clk_state
    log.info(f"Setpoint adjusted to {state['setpoint']}°F")
    publish_state()
    update_display()

def encoder_sw_callback(channel):
    """Handle rotary encoder push — toggle heater enabled."""
    state["heater_enabled"] = not state["heater_enabled"]
    if not state["heater_enabled"]:
        set_heater_relay(False)
    log.info(f"Heater toggled: {'ON' if state['heater_enabled'] else 'OFF'}")
    publish_state()
    update_display()

# -------------------------------------------------------
# Pool / Spa Button Callbacks
# -------------------------------------------------------

def btn_pool_pressed(channel):
    """Set valve position to Pool Mode."""
    log.info("Pool button pressed")
    set_valve("pool")

def btn_spa_pressed(channel):
    """Set valve position to Spa Mode."""
    log.info("Spa button pressed")
    set_valve("spa")

# -------------------------------------------------------
# Main
# -------------------------------------------------------

def main():
    global _last_clk_state

    log.info("Pool Controller starting...")

    # GPIO
    setup_gpio()
    _last_clk_state = GPIO.input(PIN_ENCODER_CLK)

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

    mqtt_client.loop_start()   # MQTT runs in background thread

    # Control loop in background thread
    threading.Thread(target=control_loop, daemon=True).start()

    log.info("Pool Controller running")

    # Keep main thread alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        log.info("Shutting down...")
        set_heater_relay(False)
        mqtt_client.loop_stop()
        GPIO.cleanup()
        log.info("Shutdown complete")

if __name__ == "__main__":
    main()
