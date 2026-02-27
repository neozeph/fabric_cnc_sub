# Raspberry Pi Pico Firmware Setup

The Fabric CNC system uses a Raspberry Pi Pico running **grblHAL** to control the stepper motors. The main Raspberry Pi sends G-code commands to the Pico via USB.

## 1. Get the Firmware

Because `config.py` defines a specific pinout (GP0-GP7 for motors), the standard pre-compiled grblHAL binaries might not match. The easiest way is to use the Web Builder to create a custom firmware file.

1. Go to the **grblHAL Web Builder**.
2. Select driver: **RP2040**.
3. Select board: **Generic 4 axis**.
4. Click **Next** until you reach the **Pin Map** or **Plugins** section.
   - *Note: If you cannot change pins easily in the web builder, you may need to wire your motors to match the standard grblHAL Pico pinout instead of the config.py defaults.*

### Wiring Connections (Standard Generic 4-axis)

Wire your motors to the Raspberry Pi Pico as follows (matches `config.py`):

| Axis | Step (PUL) Pin | Dir Pin | Enable Pin |
|------|----------------|---------|------------|
| **X**    | GP2            | GP3     | GP1        |
| **Y**    | GP4            | GP5     | GP1        |
| **Z**    | GP6            | GP7     | GP1        |
| **A**    | GP8            | GP9     | GP1        |

*Note: All axes share the same Enable pin (GP1).*

**Recommendation:** Download the standard `grblHAL` firmware for Pico and wire your motors according to the **Standard grblHAL Pin** column above.

### Partial Setup (X/Y Only)

If you are only testing the X and Y axes:
1.  Wire the **X** and **Y** motors as shown above.
2.  You can leave the **Z** and **A** pins disconnected on the Pico.
3.  The software will still show Z and A coordinates changing, but no physical movement will occur.

### Driver Wiring (Common Cathode)

For standard stepper drivers (like TB6600), use the **Common Cathode** wiring configuration. This connects the negative terminals to Ground and the positive terminals to the Pico.

1.  **Connect all `-` (Minus) terminals** (`PUL-`, `DIR-`, `ENA-`) from all drivers to a **GND** pin on the Pico.
2.  **Connect the `+` (Plus) terminals** to the corresponding Pico GPIO pins listed above.

**Example for X Axis:**
*   **PUL+** $\rightarrow$ Pico **GP2**
*   **DIR+** $\rightarrow$ Pico **GP3**
*   **ENA+** $\rightarrow$ Pico **GP1**
*   **PUL- / DIR- / ENA-** $\rightarrow$ Pico **GND**

*Note: If your motors are locked when they should be moving (or free when they should be locked), you may need to change the Enable Invert setting (`$4`) in GRBL.*

## 5. Motor Driver Settings (TB6600)

The software configuration (`config.py`) assumes specific microstepping settings. You must set the DIP switches on your TB6600 drivers to match.

### Microstep Settings (Pulse/Rev)

Look at the table printed on your TB6600 driver and set the switches to match these **Pulse/Rev** values:

| Axis | Setting Required | Microstep | Why? |
|------|------------------|-----------|------|
| **X** | **800** | 1/4 | Standard precision |
| **Y** | **800** | 1/4 | Standard precision |
| **Z** | **800** | 1/4 | Standard precision |
| **A** | **3200** | 1/16 | Higher precision for rotation |

### Current Settings (Amps)

1. Check the **Rated Current** on your motor's label or datasheet (e.g., 1.5A, 2.8A).
2. Set the current DIP switches on the TB6600 to the closest value **equal to or slightly lower** than your motor's rating.

## 6. Power Supply Wiring

Connect your **24V 5A Power Supply** to the drivers as follows:

1.  **VCC / V+**: Connect to the **+24V** terminal of your power supply.
2.  **GND / V-**: Connect to the **- / GND** terminal of your power supply.

**⚠️ IMPORTANT:** Do **NOT** connect the 24V power supply to the Raspberry Pi Pico or the Raspberry Pi 4. These devices require 5V. Only the **TB6600 drivers** should be connected to the 24V supply.

## 2. Flashing the Pico

1. **Download** the `grblHAL_RP2040.uf2` file (from the Web Builder or grblHAL GitHub releases).
2. **Unplug** the Pico from USB.
3. Hold down the **BOOTSEL** button on the Pico.
4. **Plug** the Pico into your computer (or Raspberry Pi) while holding BOOTSEL.
5. Release BOOTSEL. The Pico will appear as a USB Mass Storage drive named `RPI-RP2`.
6. **Drag and drop** the `.uf2` file onto the `RPI-RP2` drive.
7. The Pico will automatically reboot and disappear. It is now running GRBL.

## 3. Configuration

Once flashed, connect the Pico to your Raspberry Pi via USB. It should appear as `/dev/ttyACM0`.

Run the configuration checker script included in this repo to verify connection and set machine parameters:

```bash
python3 check_grbl_config.py
```

If the script connects successfully, it will check your GRBL settings (`$100`, `$101`, etc.) against the defaults needed for this machine.

### Essential GRBL Settings

You can manually send these commands via the Arduino IDE Serial Monitor or `minicom` if needed:

```
$100=2032.000 (x, step/mm - calculated from config.py)
$101=2032.000 (y, step/mm)
$102=10160.000 (z, step/mm)
$103=254.000 (a, step/mm)
$110=3000.000 (x max rate, mm/min)
$111=3000.000 (y max rate, mm/min)
$120=100.000 (x accel, mm/sec^2)
$121=100.000 (y accel, mm/sec^2)
```

## 4. Troubleshooting

**Pico not detected?**
- Check your USB cable (some are power-only).
- Ensure you are using `/dev/ttyACM0` or check `ls /dev/tty*` to find the correct port.

**Motors not moving?**
- Check the **Enable** pin wiring.
- Verify the pinout matches the firmware you flashed.