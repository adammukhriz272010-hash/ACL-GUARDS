# ACL-GUARD — Smart Knee Support (Kumpulan Vanguard, SMK Darulaman Heights 1)

The knee brace measures the knee angle. If the patient bends past the safe limit for their recovery week, **the brace vibrates and the red LED blinks**. The phone dashboard shows everything live and can also make the brace vibrate.

```
 [MPU6050 thigh] ─┐                         WiFi "ACL-GUARD"
 [MPU6050 shin ] ─┼─ I2C ─ [ESP32] ───────────────────────────► Phone browser
 [Vibration motor] ◄── GPIO25 │  web page on port 80             http://192.168.4.1
 [Red / Green LED] ◄── 26/27  │  live data on WebSocket port 81  (angle, warnings, vibrate button)
```

The brace works on its own, without the phone: the ESP32 checks the angle and starts the vibration itself. The phone is only for monitoring and settings.

## Project files

| File | What it is |
|---|---|
| `firmware/ACL_Guard/ACL_Guard.ino` | ESP32 code (sensors, warning, vibration, WiFi, web server) |
| `firmware/ACL_Guard/dashboard.h` | The phone page packed for the ESP32 (generated, don't edit) |
| `dashboard/index.html` | The phone dashboard (edit this one) |
| `build_dashboard.py` | Turns `index.html` into `dashboard.h` |

---

## 1. Parts

| Part | Qty | Notes |
|---|---|---|
| ESP32 DevKit V1 (ESP32-WROOM-32) | 1 | any ESP32 board works, but check the pin numbers |
| MPU6050 (GY-521 module) | 2 | one on the thigh, one on the shin |
| Coin vibration motor (3 V) | 1 | or a vibration motor **module** with a built-in driver |
| NPN transistor 2N2222 / S8050 (or a small logic MOSFET) | 1 | not needed if you use a motor module |
| Diode 1N4148 / 1N4007 | 1 | across the motor |
| Resistors 1 kΩ ×1, 220 Ω ×2 | | |
| LED red + LED green | 1 + 1 | |
| 3.7 V LiPo + TP4056 charger + 5 V boost (or a small power bank) | 1 | |

## 2. Wiring

| From | To |
|---|---|
| ESP32 **3V3** | VCC of both MPU6050 |
| ESP32 **GND** | GND of both MPU6050, transistor emitter, LED cathodes |
| ESP32 **GPIO 21** | SDA of both MPU6050 |
| ESP32 **GPIO 22** | SCL of both MPU6050 |
| MPU6050 **thigh** – AD0 | **GND** → address 0x68 |
| MPU6050 **shin** – AD0 | **3V3** → address 0x69 (important, otherwise both have the same address) |
| ESP32 **GPIO 25** | 1 kΩ → transistor **base** |
| Transistor **collector** | motor (−) |
| Motor (+) | 3V3 (or 5V/VIN for a 5 V motor) |
| Diode | across the motor, stripe (cathode) on motor (+) |
| ESP32 **GPIO 26** | 220 Ω → **red** LED (+), LED (−) → GND |
| ESP32 **GPIO 27** | 220 Ω → **green** LED (+), LED (−) → GND |

Using a vibration motor module (3 pins)? Connect **IN → GPIO 25**, VCC → 3V3/5V, GND → GND. Skip the transistor and diode.

**Mounting the sensors:** put both MPU6050 boards on the **outer side** of the leg: one on the thigh part of the brace and one on the shin part. Point the chip side outwards and the board's X arrow along the leg. **Both boards must face the same way.** Fix them firmly so they cannot wobble.

### Option: potentiometer instead of two MPU6050
If your brace has a hinge, you can fix a 10 kΩ rotary potentiometer to the hinge: outer pins → 3V3 and GND, middle pin → **GPIO 34**. In `ACL_Guard.ino`, set `#define SENSOR_MODE 2`, then adjust `POT_AT_0` and `POT_AT_90` (read the values in Serial Monitor).

## 3. Uploading the code

1. Install **Arduino IDE 2**.
2. *File → Preferences → Additional boards manager URLs*, add:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
3. *Tools → Board → Boards Manager*: install **esp32 by Espressif Systems**.
4. *Tools → Manage Libraries*: install **WebSockets** by **Markus Sattler**.
5. Open `firmware/ACL_Guard/ACL_Guard.ino`. The file `dashboard.h` must be in the same folder.
6. *Tools → Board*: **ESP32 Dev Module**. Choose the COM port, then click **Upload**.
7. Open Serial Monitor at **115200**. It should print `Thigh MPU6050 (0x68): OK` and `Shin MPU6050 (0x69): OK`. The motor buzzes once at startup.

## 4. Using it on the phone

1. On the phone, join the WiFi **ACL-GUARD**. The password is **acl12345** (you can change it at the top of the .ino file).
   - The phone may say "No internet". That's normal, so stay connected. On Android, turn off mobile data if the page won't open.
2. Open the browser and go to **http://192.168.4.1**. The status shows **Bersambung / Connected** in green.
3. Straighten the leg fully and press **Kalibrasi (kaki lurus)**. This sets 0°.
4. Choose the recovery week: **Minggu 1 (0–45°)**, **Minggu 2 (45–70°)**, **Minggu 3 (70–90°)**, or set your own limit with the slider. The limit is saved in the brace.
5. Bend the knee. If it goes past the limit for more than 0.3 s:
   - the **brace vibrates** (pulsing) and the **red LED blinks**
   - the phone screen flashes red and beeps (and vibrates on Android)
   - the warning is saved in **Sejarah amaran**
6. **Getar pendakap** makes the brace vibrate right away, so you can test it or call the patient's attention.

Tip: on the phone, use *Add to Home Screen* so it opens like an app.

LEDs: **green** = OK, **red fast blink** = past the limit, **red slow blink** = sensor problem.

## 5. Changing the dashboard
Edit `dashboard/index.html`, then run `python build_dashboard.py` to rebuild `firmware/ACL_Guard/dashboard.h`, and upload again. You can also open `index.html` directly on a computer: with no brace connected, it starts in **demo mode** with simulated data.

## 6. Data the brace sends (for your report)
Every 50 ms the brace sends JSON over WebSocket port 81:
`{"a":32.5,"lim":45,"w":0,"auto":1,"err":""}`, where a = knee angle, lim = limit, w = warning on, err = sensor problem.
Commands from the phone: `LIMIT:45`, `VIB:700` (ms), `ZERO`, `AUTO:1` / `AUTO:0`.

## 7. Troubleshooting

| Problem | Fix |
|---|---|
| "Sensor betis (0x69) tidak dikesan" | The shin MPU6050 AD0 pin isn't connected to 3V3, or SDA/SCL is loose |
| Angle jumps or shows backwards | The boards don't face the same way or are loose. Fix them, then calibrate again |
| Motor doesn't vibrate | Check the transistor direction (E-B-C) and that the motor (+) has power. Test with the *Getar pendakap* button |
| Page won't open | Make sure the phone is on ACL-GUARD WiFi, and type `http://` (not https) |
| No phone vibration on iPhone | Safari doesn't allow websites to vibrate the phone. The beep and red flash still work, and the brace still vibrates |

> ⚠️ This is a school prototype, not a certified medical device. The safe angles must come from the patient's doctor or physiotherapist.
