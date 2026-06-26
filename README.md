# rpi-pool-controller

Raspberry Pi pool controller — Home Assistant integration via MQTT.

Controls a Hayward gas pool heater (via dry contact), two GVA-24 valve actuators for pool/spa mode switching, a DS18B20 water temperature sensor, an SH1106 OLED display, a rotary encoder, and Pool/Spa mode buttons. Publishes state and listens for commands over MQTT with Home Assistant discovery.

## Hardware

| Component | GPIO |
| --- | --- |
| DS18B20 temperature sensor | 4 (1-Wire, kernel managed) |
| Heater relay | 17 |
| Rotary encoder CLK / DT / SW | 23 / 24 / 22 |
| Pool button / Spa button | 5 / 6 |
| Valve A (suction) open / close | 27 / 13 |
| Valve B (return) open / close | 7 / 8 |
| OLED display (I2C, SDA / SCL) | 2 / 3 |

MQTT broker: `192.168.1.13:1883`. Static IP for the Pi: `192.168.1.17` (`poolcontroller`).

## First-time setup on the Pi

```bash
# 1. Clone the repo
cd ~
git clone https://github.com/nacummins1/rpi-pool-controller.git
cd rpi-pool-controller

# 2. Create the env file (this holds the MQTT password — kept out of git)
sudo mkdir -p /etc/pool-controller
sudo tee /etc/pool-controller/env > /dev/null <<'EOF'
POOL_MQTT_PASS=your_actual_password_here
EOF
sudo chmod 600 /etc/pool-controller/env
sudo chown root:root /etc/pool-controller/env

# 3. Install the systemd service
sudo cp pool.service /etc/systemd/system/pool.service
sudo systemctl daemon-reload
sudo systemctl enable pool.service
sudo systemctl start pool.service

# 4. Verify
sudo systemctl status pool.service
journalctl -u pool.service -f
```

The optional env vars `POOL_MQTT_BROKER`, `POOL_MQTT_PORT`, and `POOL_MQTT_USER` can also be set in `/etc/pool-controller/env` to override the defaults (`192.168.1.13`, `1883`, `mqtt`). `POOL_MQTT_PASS` has no default — the service will refuse to start without it.

## Standard update / deploy

```bash
cd ~/rpi-pool-controller
git pull
sudo systemctl restart pool.service
```

If `pool.service` itself changed (the repo will show it in the `git pull` diff), also run:

```bash
sudo cp pool.service /etc/systemd/system/pool.service
sudo systemctl daemon-reload
sudo systemctl restart pool.service
```

## Rotating the MQTT password

```bash
sudo nano /etc/pool-controller/env
sudo systemctl restart pool.service
```

No code change, no git commit — credentials never enter the repo.

## Repository layout

- `pool_controller.py` — main controller (single file, ~1000 lines)
- `pool.service` — canonical systemd unit; deploy by copying to `/etc/systemd/system/`
- `.gitignore` — keeps env files and local runtime state out of the repo
