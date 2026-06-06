# Flow Calibration for Vanilla Klipper (Kobra S1 + ACE)

Automatic **pressure-advance** calibration using the Anycubic Kobra S1's built-in
**CS1237 nozzle pressure sensor** — on a vanilla / fork Klipper instance connected
to the toolhead MCU over a socat tunnel. No extra hardware: it uses the load cell
that's already in your toolhead.

Integrates with the **ACEPRO** multi-material plugin (slot picker, auto tool load,
filament temperature from the ACE), and works without ACE too.

---

## ✨ Features

- One command to measure **and apply** pressure advance: `FLOW_CALIBRATE`
- **ACE integration** — pick a loaded slot from a Mainsail/Fluidd popup, auto
  loads/swaps the tool, reads the filament temperature from the ACE, multi-unit
  aware (T0–T3 / T4–T7 …)
- Works **without ACE** as well (normal filament rack)
- **Per-nozzle** calibration, repeatable (median of N runs)
- Saves the result to your config via the **SAVE_CONFIG** banner
- Installable & updatable through the **Mainsail/Fluidd Update Manager**

---

## ⚙️ Requirements

- Klipper connected to the Kobra S1 `nozzle_mcu` over a socat tunnel
  ([setup guide](https://github.com/Kobra-S1/vanilla-klipper-swu/blob/main/tunneled-klipper.md))
- `[respond]` enabled (for the slot popup)
- Stock Kobra S1 macros (`G28`, `TO_THROW_POSITION`, `CUT_TIP`, `PURGE_AND_POOP`, …)
- *Optional:* the [ACEPRO](https://github.com/Kobra-S1/ACEPRO) plugin for
  multi-material

---

## 📦 Installation

### Update Manager (recommended)

```bash
cd ~
git clone https://github.com/<your-user>/flow_calibration.git
cd flow_calibration
bash install.sh
```

`install.sh` symlinks the modules into `klippy/extras/`, registers the repo with
Moonraker, and restarts. Updates then appear in **Mainsail → Update Manager** —
one click to update.

### Manual

```bash
sudo cp extras/*.py ~/klipper/klippy/extras/
sudo systemctl restart klipper
```

---

## 🔧 Configuration

Add two sections to your `printer.cfg`. The fully annotated example is in
[`config_example/flow_calibration.cfg`](config_example/flow_calibration.cfg).

**`[cs1237]`** — the sensor (stock pins):

```ini
[cs1237]
level_pin: nozzle_mcu:PA7
dout_pin:  nozzle_mcu:PA6
sclk_pin:  nozzle_mcu:PA5
register: 60
sensitivity: -2500
```

**`[flow_calibration]`** — the algorithm. **Important:** copy the
`interp_fit_factor` table from *your own machine's* stock config — it is
machine-specific and gives the best results. See the example file.

After any config change: **`RESTART`**.

---

## 🎯 First-time calibration (once per nozzle)

The measurement is **relative**, so it has to be anchored to one real print once
per nozzle size:

1. Run `FLOW_CALIBRATE TOOL=0 VERBOSE=1` two or three times and note the `K_raw`
   value from the `Flow cal fit …` line.
2. Print a PA test and pick the best value by eye (call it `PA_real`):
   ```gcode
   TUNING_TOWER COMMAND=SET_PRESSURE_ADVANCE PARAMETER=ADVANCE START=0.005 FACTOR=0.0005
   ```
3. Add `nozzle, PA_real / K_raw` to `nozzle_pa_scale`, e.g. `0.6, 2.33`.
4. `RESTART` — now `FLOW_CALIBRATE` outputs the right value for that nozzle.

> The built-in factors (0.2 / 0.4 / 0.8) are rough estimates — calibrate each
> nozzle you actually use. When i can collect enouhg values i will adjust these in the example config file.

---

## 🚀 Usage

```gcode
FLOW_CALIBRATE                 # popup: pick a loaded ACE slot
FLOW_CALIBRATE TOOL=0          # calibrate slot 0 directly
FLOW_CALIBRATE TOOL=0 RUNS=3   # median of 3 runs (best after a filament change)
```

When it finishes, click the **SAVE_CONFIG** banner to keep the value.

In your slicer's **print-start** g-code (after the tool is loaded and hot):

```gcode
FLOW_CALIBRATE TOOL={initial_tool} KEEP_HOT=1 SAVE=0
```

`QUERY_FLOW_SENSOR` checks the sensor without any movement or heating.

| Parameter | Meaning |
|---|---|
| `TOOL=` | ACE slot (omit → popup) |
| `TEMPERATURE=` | override nozzle temperature |
| `RUNS=` | repeat N times, apply the median |
| `KEEP_HOT=1` | skip cooldown (for print-start) |
| `SAVE=0` | don't stage to config |
| `VERBOSE=1` | print measurement details |

---

## 🩺 Troubleshooting

| Problem | Fix |
|---|---|
| `QUERY_FLOW_SENSOR` → `NO RESPONSE` | check the socat tunnel to `nozzle_mcu` |
| Result is always too low / too high | not anchored — do the per-nozzle calibration above |
| First run after a tool change is way off | use `RUNS=3` (median) |
| `[cs1237] ... is not a valid config section` | copy `cs1237.py` and restart Klipper |
| Popup doesn't open | make sure `[respond]` is in your config |

---

## ℹ️ How it works

It sweeps pressure advance while extruding a small zig-zag over the poop chute and
reads the nozzle back-pressure from the CS1237 load cell at each step, then
robust-fits the best pressure advance. (A small XY move is needed for PA to apply,
and a short dwell after each move lets the sensor capture the pressure decay.)

---

## Support

If you find this project useful and want to help cover the cost of filament spools from
more brands for NFC testing and implementation — donations are always welcome, never expected.

[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/jupsi)

---