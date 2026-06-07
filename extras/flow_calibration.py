# Flow Calibration (Pressure-Advance auto-calibration) for vanilla Klipper.
#
# Uses the CS1237 strain-gauge ADC on the toolhead MCU (extras/cs1237.py)
# to measure nozzle back-pressure at two extrusion speeds across a PA sweep, then
# robust-fits the best pressure-advance value.
#
# Integrates with the ACEPRO multi-material plugin (github Kobra-S1/ACEPRO@dev):
#   - auto-detects connected ACE unit(s) (multi-unit aware, T0-3 / T4-7 ...)
#   - FLOW_CALIBRATE TOOL=<n>; without TOOL a Mainsail/Fluidd popup lets you pick
#     a loaded slot
#   - reads the recommended nozzle temperature for the slot from the ACE
#   - loads / swaps the selected slot with the correct macros before calibrating
#   - falls back to normal "filament rack" behaviour when no ACE is connected
#
# Config section: [flow_calibration]   (see config_example/flow_calibration.cfg)

import math
import logging
from collections import OrderedDict

logger = logging.getLogger("klippy.extras.flow_calibration")

# ── Calibration phase enums ────────────────
STOP_CALIBRATION = 0
START_CALIBRATION = 1
ZERO_SPEED = 0
STOP_LOW_SPEED = 2
START_LOW_SPEED = 3
START_HIGH_SPEED = 4
STOP_HIGH_SPEED = 5

# ── Algorithm constants ────────────────────
PA_SWEEP_MIN = 0.02              # pressure-advance sweep range
PA_SWEEP_MAX = 0.075
PA_STEP = 0.005
INIT_SPEED_100 = 6000.0          # mm/min (100 mm/s)
INIT_SPEED_300 = 18000.0         # mm/min (300 mm/s)
BLOCK_MAX_CNT = 5
BLOCK_THRESHOLD = 550000
RACK_FILAMENT_INDEX = -1         # result key used in non-ACE (rack) mode

INTERP_SPEEDS = [100.0, 200.0, 300.0, 400.0]
INTERP_TEMPS = [210.0, 220.0, 230.0, 240.0, 250.0]

# Per-nozzle calibration gain (anchors the absolute output to a real PA test
# print). 0.6 is measured; 0.4 is a neutral placeholder; 0.2 and 0.8 are rough
# estimates. Anchor each nozzle you actually use (see README.md) and override
# via the [flow_calibration] nozzle_pa_scale block.
PA_SCALE_DEFAULTS = {0.2: 0.24, 0.4: 1.0, 0.6: 2.33, 0.8: 4.18}

# Calibration status
CALIBRATION_UNCALIBRATED = 0
CALIBRATION_SUCCESS = 1
CALIBRATION_FAILED = 2


def saturate(val, vmin, vmax):
    if val < vmin:
        return vmin
    if val > vmax:
        return vmax
    return val


# ── 2D bilinear interpolation ─────────────────────
def interpolate_2d(xt, yt, xs, ys, grid):
    x1, x2 = 0, 1
    for i in range(len(xs) - 1):
        if xs[i] <= xt <= xs[i + 1]:
            x1, x2 = i, i + 1
            break
    y1, y2 = 0, 1
    for j in range(len(ys) - 1):
        if ys[j] <= yt <= ys[j + 1]:
            y1, y2 = j, j + 1
            break
    q11 = grid[x1][y1]
    q21 = grid[x2][y1]
    q12 = grid[x1][y2]
    q22 = grid[x2][y2]
    dx = xs[x2] - xs[x1]
    dy = ys[y2] - ys[y1]
    if dx == 0 or dy == 0:
        return q11
    r1 = q11 + (xt - xs[x1]) * (q21 - q11) / dx
    r2 = q12 + (xt - xs[x1]) * (q22 - q12) / dx
    return r1 + (yt - ys[y1]) * (r2 - r1) / dy


# ── Robust linear fit — IRLS / bisquare ───────────
def _median(data):
    n = len(data)
    if n == 0:
        return 0.0
    s = sorted(data)
    if n % 2 == 1:
        return s[n // 2]
    return (s[n // 2 - 1] + s[n // 2]) / 2.0


def _linear_fit(X, Y, W):
    """Weighted least squares for a P=2 design matrix (rows [x, 1]).

    Returns (p, leverage, residual). leverage (hat-matrix diagonal of the
    UNWEIGHTED X) is computed only when W is falsy (i.e. on the first,
    unweighted call). residual = Dy - J*p.
    """
    n = len(X)
    if W:
        sw = [math.sqrt(w) for w in W]
        J = [[sw[i] * X[i][0], sw[i] * X[i][1]] for i in range(n)]
        Dy = [sw[i] * Y[i] for i in range(n)]
    else:
        J = X
        Dy = Y
    a11 = a12 = a22 = b1 = b2 = 0.0
    for i in range(n):
        j0, j1 = J[i][0], J[i][1]
        a11 += j0 * j0
        a12 += j0 * j1
        a22 += j1 * j1
        b1 += j0 * Dy[i]
        b2 += j1 * Dy[i]
    det = a11 * a22 - a12 * a12
    if abs(det) < 1e-12:
        mean = (sum(Y) / n) if n else 0.0
        return [0.0, mean], [0.0] * n, [0.0] * n
    p0 = (b1 * a22 - b2 * a12) / det
    p1 = (a11 * b2 - a12 * b1) / det
    p = [p0, p1]
    leverage = []
    if not W:
        m11 = m12 = m22 = 0.0
        for i in range(n):
            x0, x1 = X[i][0], X[i][1]
            m11 += x0 * x0
            m12 += x0 * x1
            m22 += x1 * x1
        dm = m11 * m22 - m12 * m12
        if abs(dm) < 1e-12:
            leverage = [0.0] * n
        else:
            i11 = m22 / dm
            i12 = -m12 / dm
            i22 = m11 / dm
            leverage = [X[i][0] * (i11 * X[i][0] + i12 * X[i][1])
                        + X[i][1] * (i12 * X[i][0] + i22 * X[i][1])
                        for i in range(n)]
    residual = [Dy[i] - (J[i][0] * p0 + J[i][1] * p1) for i in range(n)]
    return p, leverage, residual


def _bisquare_weights(r):
    w = [0.0] * len(r)
    all_zero = True
    for i, v in enumerate(r):
        if abs(v) < 1.0:
            w[i] = (1.0 - v * v) ** 2
            all_zero = False
        else:
            w[i] = 0.0
    if all_zero:
        w = [1.0] * len(r)
    return w


def robust_fit(X, Y):
    """IRLS robust regression with Tukey bisquare weights.

    X: rows [DeltaP, 1.0]; Y: PA values. Returns p = [slope, intercept].
    """
    p, leverage, residual = _linear_fit(X, Y, None)
    P = len(p)
    n = len(Y)
    # adjFactor computed ONCE from the unweighted leverage (do not recompute).
    h = [min(v, 0.9999) for v in leverage]
    adj = [1.0 / math.sqrt(1.0 - hv) for hv in h]
    mean = sum(Y) / n if n else 0.0
    var = (sum((v - mean) ** 2 for v in Y) / (n - 1)) if n > 1 else 0.0
    std = math.sqrt(var)
    tiny_s = 1e-6 * std
    D = 1e-6
    for _ in range(50):
        radj = [residual[i] * adj[i] for i in range(n)]
        rs = sorted(abs(v) for v in radj)
        sub = rs[P - 1:]                      # drop the P-1 smallest (MATLAB-ism)
        median = _median(sub)
        sigma = median / 0.6745
        norm = max(tiny_s, sigma) * 4.685
        if norm == 0.0:
            norm = 1.0
        r = [v / norm for v in radj]
        bw = _bisquare_weights(r)
        p0 = p
        p, _, _ = _linear_fit(X, Y, bw)
        residual = [Y[i] - (X[i][0] * p[0] + X[i][1] * p[1]) for i in range(n)]
        done = True
        for j in range(P):
            if abs(p[j] - p0[j]) > D * max(abs(p[j]), abs(p0[j])):
                done = False
                break
        if done:
            break
    return p


# ── Sensor adapter ─────────────────────────────────────────────────────
# Some Klipper builds (the tunnelled Kobra S1 fork) already ship their own
# [cs1237] module — it is also used by their bed probe, so this plugin must
# NOT replace it. That module exposes the same sensor under different method
# names. This adapter wraps such a foreign object and presents the small
# interface the calibration algorithm uses, so the algorithm code stays
# unchanged regardless of which [cs1237] module is loaded.
class _ForeignCS1237Adapter:
    def __init__(self, cs, printer):
        self._cs = cs
        self._printer = printer
        self.stock_calibration = self.has_stock_calibration(cs)

    @staticmethod
    def matches(cs):
        # A foreign module that provides the calibration data call under its
        # own name (and lacks this plugin's query_calibration_val()).
        return (not hasattr(cs, 'query_calibration_val')
                and hasattr(cs, '_stock_cs1237_calibration_data_process'))

    @staticmethod
    def has_stock_calibration(cs):
        # The foreign module only wires up the calibration commands when the
        # toolhead MCU actually exposes them (stock Anycubic firmware).
        caps = getattr(cs, 'capabilities', None)
        if caps is not None:
            return 'stock_calibration' in caps
        return hasattr(cs, '_cmd_calibration_data')

    def _dwell(self, ms):
        gcode = self._printer.lookup_object('gcode')
        gcode.run_script_from_command("G4 P%d" % ms)

    def enable(self):
        self._cs._enable_cs1237(1)
        self._dwell(500)          # match this plugin's sensor settle time

    def disable(self):
        self._cs._enable_cs1237(0)

    def calibration(self, cali_state, speed_state):
        self._cs._stock_cs1237_calibration_phase(cali_state, speed_state)

    def query_calibration_val(self):
        # Foreign module raises on timeout; convert to this plugin's None
        # sentinel so the algorithm's existing handling applies.
        try:
            d = self._cs._stock_cs1237_calibration_data_process()
        except Exception:
            return None, None, None
        return (d.get('BlockPreVal'), d.get('TargetVal'), d.get('RealVal'))

    def query_diff(self):
        try:
            d = self._cs.cs1237_diff_process()
        except Exception:
            return None, None
        return d.get('diff'), d.get('raw')


# ── Flow Calibration module ────────────────────────────────────────────
class FlowCalibration:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.cmderr = self.printer.command_error

        # Algorithm parameters
        self.enable_dyna_PA = config.getboolean('enable_dyna_PA', False)
        self.pa_min = config.getfloat('pa_min', 0.015)
        self.pa_min_nozzled_comp = config.getfloat('pa_min_nozzled_comp', 0.01)
        self.pa_max = config.getfloat('pa_max', 0.075)
        # The interp_fit_factor table and FitVal heuristic are tuned to a
        # measurement geometry that differs from the kinematic one used here, so
        # the ABSOLUTE PA output must be anchored once per nozzle to a real PA
        # test print. The relative measurement (dP-vs-PA) is correct regardless.
        # pa_scale is the fallback for nozzle sizes not in nozzle_pa_scale.
        self.pa_scale = config.getfloat('pa_scale', 1.0, above=0.)
        self.nozzle_pa_scale = dict(PA_SCALE_DEFAULTS)
        raw_scale = config.get('nozzle_pa_scale', '').strip()
        for line in raw_scale.split('\n'):
            line = line.split('#', 1)[0].strip()
            if not line:
                continue
            parts = line.replace(',', ' ').split()
            if len(parts) >= 2:
                self.nozzle_pa_scale[round(float(parts[0]), 2)] = float(parts[1])
        self.delta_press_max = config.getfloat('delta_press_max', 60000.)
        self.fit_val_max = config.getfloat('fit_val_max', 10000.)
        self.cali_speed_default = config.getfloat('cali_speed', 200.,
                                                  above=0.)
        # Settle dwell (ms) after STOP_HIGH_SPEED. The MCU's feature detector
        # samples the CS1237 at ~1280 Hz and needs the post-peak pressure decay
        # (a ~200-sample tail, ~156 ms) to set RealVal distinct from TargetVal.
        # Querying immediately returns RealVal==TargetVal -> dP=0; the dwell
        # makes the post-peak decay observable before the query.
        self.measure_settle_ms = config.getint('measure_settle_ms', 250,
                                               minval=0)
        # Repeat the sweep N times and apply the MEDIAN. Rejects the unstable
        # first run right after a tool change (melt not yet conditioned).
        self.measure_runs = config.getint('measure_runs', 1, minval=1)
        # CRITICAL: pressure advance is only applied to moves that also have X/Y
        # motion (kinematics/extruder.py: can_pressure_advance requires
        # axes_d[0] or axes_d[1]). Pure-E calibration moves get NO PA, so the K
        # sweep would be inert. We therefore do small kinematic zig-zag moves
        # over the chute. They must also last long enough (~200 ms) for the MCU
        # feature detector to gather >=50 samples and resolve a rise+fall.
        self.measure_axis = config.get('measure_axis', 'X').upper()
        if self.measure_axis not in ('X', 'Y'):
            raise config.error("[flow_calibration] measure_axis must be X or Y")
        self.measure_travel = config.getfloat('measure_travel', 10., above=0.)
        self.low_extrude = config.getfloat('low_extrude', 1., above=0.)
        self.low_move_ms = config.getint('low_move_ms', 80, minval=20)
        self.high_extrude = config.getfloat('high_extrude', 2., above=0.)
        self.high_move_ms = config.getint('high_move_ms', 200, minval=50)
        self._center = None
        self.interp_fit_factor = self._parse_fit_table(config)

        # Macro hooks (defaults match the standard Kobra S1 macros)
        self.home_macro = config.get('home_macro', 'G28')
        self.throw_position_macro = config.get('throw_position_macro',
                                               'TO_THROW_POSITION')
        self.cut_tip_macro = config.get('cut_tip_macro', 'CUT_TIP')
        self.ace_object_name = config.get('ace_object_name', 'ace')
        self.ace_change_tool_tmpl = config.get('ace_change_tool_macro',
                                               'ACE_CHANGE_TOOL TOOL={tool}')
        self.ace_unload_macro = config.get('ace_unload_macro',
                                           'ACE_CHANGE_TOOL TOOL=-1')
        self.ace_full_unload_macro = config.get('ace_full_unload_macro',
                                                'ACE_FULL_UNLOAD')
        # Feed assist keeps the ACE buffer pumped so the toolhead extruder
        # doesn't pull against the bowden. Enabled for the tool, disabled in
        # cleanup.
        self.ace_feed_assist = config.getboolean('ace_feed_assist', True)
        self.ace_feed_assist_on = config.get(
            'ace_feed_assist_on_macro', 'ACE_ENABLE_FEED_ASSIST T={tool}')
        self.ace_feed_assist_off = config.get(
            'ace_feed_assist_off_macro', 'ACE_DISABLE_FEED_ASSIST T={tool}')
        self._fa_tool = None
        self.use_ace_change_tool = config.getboolean('use_ace_change_tool',
                                                     True)
        self.wipe_gcode = config.get('wipe_gcode', 'WIPE_ENTER\nWIPE_STOP')
        self.flush_poop_macro = config.get('flush_poop_macro', 'FLUSH_POOP')
        self.fan_on_tmpl = config.get('fan_on_template', 'M106 S{speed}')
        self.fan_off_macro = config.get('fan_off_macro', 'M106 S0')
        self.prime_gcode = config.get('prime_gcode',
                                      'G92 E0\nG1 E10 F1000')
        # Fresh prime over the chute right before the sweep, so the melt zone is
        # pressurized and any aged/oozed tip is dropped before measuring.
        self.pre_cal_prime = config.getboolean('pre_cal_prime', True)
        # Optional: reuse an existing purge macro for the pre-cal prime instead
        # of the raw extrude below (e.g. "PURGE_AND_POOP PURGELENGTH=40" which
        # matches the system's print-start purge default).
        self.pre_cal_prime_macro = config.get('pre_cal_prime_macro', '')
        # Raw-extrude fallback. Default 40 mm to match the print-start purge
        # (printer_generic_macros PURGE default). Filament is already loaded at
        # the nozzle during flow cal, so the 85 mm full-load purge isn't needed.
        self.prime_length = config.getfloat('prime_length', 40., minval=0.)
        self.prime_speed = config.getfloat('prime_speed', 300., above=0.)
        self.pre_cal_flush = config.getboolean('pre_cal_flush', True)
        self.cooldown_temp = config.getfloat('cooldown_temp', 0.)
        self.z_hop = config.getfloat('z_hop', 5., minval=0.)
        self.z_hop_speed = config.getfloat('z_hop_speed', 10., above=0.)
        self.do_poop_cleanup = config.getboolean('post_cal_poop_wipe', True)
        # Optional toolhead filament sensor (rack mode presence check)
        self.filament_sensor_name = config.get('filament_sensor',
                                                'filament_runout_nozzle')
        # Persist the result via SAVE_CONFIG (like cartographer / PID_CALIBRATE).
        self.save_to_config = config.getboolean('save_to_config', True)
        self.save_section = config.get('save_config_section', 'extruder')
        self.save_option = config.get('save_config_option', 'pressure_advance')
        # Print live per-sample sensor values to the console.
        self.verbose = config.getboolean('verbose', False)
        self._verbose = False
        self._keep_hot = False

        # State
        self.cs1237 = None
        self.calib_pa_map = OrderedDict()
        self.calib_status_map = {}
        self.inter_bestk_map = {}
        self.inter_speeds = []

        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.gcode.register_command(
            "FLOW_CALIBRATE", self.cmd_FLOW_CALIBRATE,
            desc=self.cmd_FLOW_CALIBRATE_help)
        self.gcode.register_command(
            "PA_AUTO_CALIBRATE", self.cmd_PA_AUTO_CALIBRATE,
            desc="Anycubic-compatible alias for FLOW_CALIBRATE")
        logger.info("FlowCalibration loaded: dyna=%s pa=[%.3f,%.3f] "
                    "dp_max=%.0f fit_max=%.0f",
                    self.enable_dyna_PA, self.pa_min, self.pa_max,
                    self.delta_press_max, self.fit_val_max)

    # ------------------------------------------------------------------
    def _parse_fit_table(self, config):
        raw = config.get('interp_fit_factor', '').strip()
        if not raw:
            raise config.error("[flow_calibration] interp_fit_factor is required")
        rows = []
        for line in raw.split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            line = line.split('#', 1)[0]
            vals = []
            for tok in line.replace(',', ' ').split():
                try:
                    vals.append(float(tok))
                except ValueError:
                    pass
            if vals:
                rows.append(vals)
        if len(rows) != 4 or any(len(r) != 5 for r in rows):
            raise config.error(
                "[flow_calibration] interp_fit_factor must be a 4x5 matrix "
                "(4 speeds x 5 temps), got %d rows" % len(rows))
        return rows

    def _resolve_cs1237(self):
        # Look up the [cs1237] object and, if it is a foreign module (e.g. the
        # Kobra S1 fork's own cs1237.py, shared with its bed probe), wrap it in
        # an adapter so the algorithm can use it without changes.
        cs = self.printer.lookup_object('cs1237', None)
        if cs is not None and _ForeignCS1237Adapter.matches(cs):
            if not _ForeignCS1237Adapter.has_stock_calibration(cs):
                logger.warning(
                    "FlowCalibration: the installed [cs1237] module has no "
                    "stock calibration commands — the toolhead MCU firmware "
                    "does not expose them. FLOW_CALIBRATE needs the original "
                    "Anycubic nozzle-MCU firmware.")
            cs = _ForeignCS1237Adapter(cs, self.printer)
            logger.info("FlowCalibration: using foreign [cs1237] module "
                        "via adapter")
        return cs

    def _handle_ready(self):
        self.cs1237 = self._resolve_cs1237()
        if self.cs1237 is None:
            logger.warning("FlowCalibration: no [cs1237] section found — "
                           "FLOW_CALIBRATE will not work until it is added")

    def get_status(self, eventtime):
        return {
            'calib_pa': dict(self.calib_pa_map),
            'calib_status': dict(self.calib_status_map),
            'dynamic': self.enable_dyna_PA,
        }

    # ------------------------------------------------------------------
    # Small helpers
    # ------------------------------------------------------------------
    def _run(self, script):
        self.gcode.run_script_from_command(script)

    def _try_run(self, script, what):
        try:
            self.gcode.run_script_from_command(script)
        except Exception as e:
            logger.warning("FlowCalibration: '%s' failed: %s", what, e)
            self.gcode.respond_info(
                "Flow cal: optional step '%s' skipped (%s)" % (what, e))

    def _now(self):
        return self.reactor.monotonic()

    def _is_homed(self, axes):
        toolhead = self.printer.lookup_object('toolhead')
        homed = toolhead.get_status(self._now())['homed_axes']
        return all(a in homed for a in axes)

    def _extruder(self):
        return self.printer.lookup_object('extruder')

    def _nozzle_temp(self):
        heater = self._extruder().get_heater()
        return heater.get_temp(self._now())[0]

    def _check_can_extrude(self):
        st = self._extruder().get_status(self._now())
        if not st.get('can_extrude', False):
            raise self.cmderr(
                "Nozzle too cold to extrude (temp %.0f). Heat the hotend "
                "before calibrating." % self._nozzle_temp())

    def _current_pa(self):
        try:
            return self._extruder().get_status(self._now())['pressure_advance']
        except Exception:
            return None

    def _set_pa(self, value):
        self._run("SET_PRESSURE_ADVANCE ADVANCE=%.4f" % value)

    # ------------------------------------------------------------------
    # ACE detection / slot enumeration
    # ------------------------------------------------------------------
    def _ace_toggle_on(self):
        tog = self.printer.lookup_object('output_pin ACE_Pro', None)
        if tog is None:
            return True
        try:
            return tog.get_status(self._now()).get('value', 1.) > 0.
        except Exception:
            return True

    def _connected_units(self):
        mgr = self.printer.lookup_object(self.ace_object_name, None)
        if mgr is None or not self._ace_toggle_on():
            return []
        try:
            n = mgr.get_status(self._now()).get('ace_instances', 0)
        except Exception:
            return []
        units = []
        for i in range(n):
            inst = self.printer.lookup_object('ace_instance_%d' % i, None)
            if inst is None:
                continue
            try:
                if inst.serial_mgr.is_connected():
                    units.append((i, inst))
            except Exception:
                continue
        return units

    def _occupied_slots(self, units):
        slots = []
        for (i, inst) in units:
            try:
                st = inst.get_status(self._now())
            except Exception:
                continue
            for s in st.get('slots', []):
                if s.get('status') in (None, '', 'empty'):
                    continue
                tool = s.get('tool')
                if tool is None:
                    # Fall back to global numbering: unit*4 + local index.
                    local = s.get('index', 0)
                    tool = i * 4 + (local or 0)
                try:
                    tool = int(tool)
                except (TypeError, ValueError):
                    continue
                slots.append({
                    'tool': tool,
                    'unit': i,
                    'material': s.get('material') or '',
                    'temp': s.get('temp') or 0,
                    'color': s.get('color'),
                    'status': s.get('status'),
                })
        return slots

    def _ace_current_index(self):
        mgr = self.printer.lookup_object(self.ace_object_name, None)
        if mgr is None:
            return -1
        try:
            return mgr.get_status(self._now()).get('current_index', -1)
        except Exception:
            return -1

    def _ace_filament_pos(self):
        sv = self.printer.lookup_object('save_variables', None)
        if sv is None:
            return 'none'
        try:
            return sv.allVariables.get('ace_filament_pos', 'none')
        except Exception:
            return 'none'

    def _prompt(self, line):
        # Mainsail/Fluidd only react to action:prompt_* when each line is sent
        # as its own RESPOND TYPE=command message (same as the ACEPRO plugin).
        self._run('RESPOND TYPE=command MSG="%s"' % line)

    def _show_slot_popup(self, slots, extra_args=""):
        self._prompt("action:prompt_begin Flow Calibration")
        self._prompt("action:prompt_text Slot fuer die Flow-Kalibrierung "
                     "waehlen:")
        for s in slots:
            label = ("T%s" % s['tool'])
            if s['material']:
                label += " %s" % s['material']
            if s['temp']:
                label += " (%d C)" % int(s['temp'])
            label = label.replace('"', '').replace('|', '/')
            # Inner cancel/cmd uses no nested double-quotes (MSG=... unquoted).
            self._prompt("action:prompt_button %s|FLOW_CALIBRATE TOOL=%s%s"
                         "|primary" % (label, s['tool'], extra_args))
        self._prompt("action:prompt_footer_button Abbrechen|"
                     "RESPOND TYPE=command MSG=action:prompt_end|error")
        self._prompt("action:prompt_show")

    def _close_popup(self):
        self._run('RESPOND TYPE=command MSG="action:prompt_end"')

    # ------------------------------------------------------------------
    # Fresh prime + poop drop over the chute before the measurement sweep
    # ------------------------------------------------------------------
    def _pre_cal_prime(self):
        if not self.pre_cal_prime:
            return
        if self.pre_cal_prime_macro:
            # Reuse the system's purge macro (e.g. PURGE_AND_POOP / PURGE_IN_CHUNKS
            # with the configured PURGELENGTH) instead of a raw extrude, then
            # return over the chute for the sweep.
            self._try_run(self.pre_cal_prime_macro, "pre-cal prime macro")
            self._try_run(self.throw_position_macro, "throw position")
            return
        if self.prime_length > 0:
            self.gcode.respond_info(
                "Flow cal: priming %.0f mm before sweep..." % self.prime_length)
            self._run("M83")
            self._run("G1 E%.1f F%.0f" % (self.prime_length, self.prime_speed))
            self._run("M400")
        if self.pre_cal_flush:
            # Drop the primed ooze, then return over the chute for the sweep.
            self._try_run(self.flush_poop_macro, "pre-cal poop drop")
            self._try_run(self.throw_position_macro, "throw position")

    # ------------------------------------------------------------------
    # Persist result via SAVE_CONFIG (like cartographer / PID_CALIBRATE)
    # ------------------------------------------------------------------
    def _save_to_config(self, pa):
        configfile = self.printer.lookup_object('configfile')
        configfile.set(self.save_section, self.save_option, "%.4f" % pa)
        # Two lines: SAVE_CONFIG at the start of its own line so Mainsail/Fluidd
        # auto-link it. The yellow save-config banner (driven by the
        # save_config_pending state that configfile.set sets) is the primary
        # clickable element.
        self.gcode.respond_info(
            "Flow cal: %s.%s = %.4f staged (already active this session)\n"
            "SAVE_CONFIG to write it to printer.cfg and restart"
            % (self.save_section, self.save_option, pa))

    # ------------------------------------------------------------------
    # Load / swap logic
    # ------------------------------------------------------------------
    def _ensure_loaded(self, tool, temp):
        current = self._ace_current_index()
        pos = self._ace_filament_pos()
        if current == tool and pos == 'nozzle':
            self.gcode.respond_info(
                "Flow cal: T%d already loaded at nozzle" % tool)
            return
        change = self.ace_change_tool_tmpl.format(tool=tool)
        if current is None or current < 0 or pos != 'nozzle':
            # Nothing loaded in the head -> load the selected slot.
            self.gcode.respond_info("Flow cal: loading T%d ..." % tool)
            if not self._is_homed('xy'):
                self._run(self.home_macro)
            self._run(self.throw_position_macro)
            self._run(change)
        else:
            # A different slot is loaded -> swap.
            self.gcode.respond_info(
                "Flow cal: swapping head T%s -> T%d ..." % (current, tool))
            self._run("M109 S%.0f" % temp)
            if self.use_ace_change_tool:
                # ACE_CHANGE_TOOL self-homes and cuts/unloads/loads/purges.
                self._run(change)
            else:
                if not self._is_homed('xy'):
                    self._run(self.home_macro)
                self._run(self.cut_tip_macro)
                self._run(self.ace_full_unload_macro)
                self._run(change)

    # ------------------------------------------------------------------
    # Rack (no-ACE) preparation
    # ------------------------------------------------------------------
    def _rack_prep(self, gcmd, temp):
        bed_temp = gcmd.get_float('BEDTEMP', 0.)
        self._try_run(self.fan_off_macro, "fan off")
        if bed_temp > 0:
            self._run("M140 S%.0f" % bed_temp)
        self._run("M104 S%.0f" % temp)
        if not self._is_homed('xy'):
            self._run(self.home_macro)
        if self.z_hop > 0:
            self._run("G91\nG1 Z%.2f F%.0f\nG90"
                      % (self.z_hop, self.z_hop_speed * 60.))
        self._try_run(self.throw_position_macro, "throw position")
        self._run("M400")
        self._run("M109 S%.0f" % temp)
        # Prime so the nozzle is full before measuring back-pressure.
        self._run(self.prime_gcode)
        self._run("M400")
        # Warn if the toolhead filament sensor says no filament present.
        sensor = self.printer.lookup_object(
            'filament_switch_sensor %s' % self.filament_sensor_name, None)
        if sensor is not None:
            try:
                if not sensor.get_status(self._now())['filament_detected']:
                    self.gcode.respond_info(
                        "WARNING: no filament detected at the nozzle sensor. "
                        "Flow calibration needs filament loaded and primed!")
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Core calibration
    # ------------------------------------------------------------------
    def _low_move(self):
        half = self.measure_travel / 2.0
        target = self._center + half
        feed = self.measure_travel / (self.low_move_ms / 1000.0) * 60.0
        return "G1 %s%.3f E%.4f F%.0f" % (self.measure_axis, target,
                                          self.low_extrude, feed)

    def _high_move(self):
        half = self.measure_travel / 2.0
        target = self._center - half
        feed = self.measure_travel / (self.high_move_ms / 1000.0) * 60.0
        return "G1 %s%.3f E%.4f F%.0f" % (self.measure_axis, target,
                                          self.high_extrude, feed)

    def _measure_pa_point(self, high_speed, k, last_bpv):
        cs = self.cs1237
        self._run("M83")
        self._set_pa(saturate(k, 0.0, 1.0))
        cs.calibration(START_CALIBRATION, START_LOW_SPEED)
        self._run(self._low_move())
        self._run("M400")
        cs.calibration(START_CALIBRATION, STOP_LOW_SPEED)
        cs.calibration(START_CALIBRATION, START_HIGH_SPEED)
        self._run(self._high_move())
        self._run("M400")
        cs.calibration(START_CALIBRATION, STOP_HIGH_SPEED)
        # Hold while the MCU observes the post-peak pressure decay so it can set
        # RealVal distinct from TargetVal (otherwise dP=0). The sensor keeps
        # sampling during the dwell; M400 makes the host wait in real time.
        if self.measure_settle_ms > 0:
            self._run("G4 P%d" % self.measure_settle_ms)
            self._run("M400")
        bpv, tv, rv = cs.query_calibration_val()
        if bpv is None:
            # Response timeout — treat as an abnormal (no data) sample.
            last_bpv[0] = 0
            logger.warning("measure_pa_point k=%.4f: calibration_Val timeout", k)
            if self._verbose:
                self.gcode.respond_info(
                    "  k=%.4f  TIMEOUT — no cs1237_calibration_Val response "
                    "(tunnel/sensor?)" % k)
            return True, 0.0
        last_bpv[0] = bpv
        abnormal = (tv == 0 or rv == 0)
        logger.debug("measure_pa_point k=%.4f bpv=%d tv=%d rv=%d dP=%.1f speed=%.1f",
                     k, bpv, tv, rv, float(rv - tv), high_speed / 60.0)
        if self._verbose:
            flag = "  <abnormal: no pressure data>" if abnormal else ""
            self.gcode.respond_info(
                "  k=%.4f  dP=%d  (target=%d real=%d block=%d)%s"
                % (k, rv - tv, tv, rv, bpv, flag))
        if abnormal:
            return True, 0.0
        return False, float(rv - tv)

    def _fitting_solution(self, current_speed, temp, pa_sets, dps, robust,
                          eff_pa_min, nozzle_d):
        if len(pa_sets) < 3:
            raise self.cmderr(
                "Flow cal: not enough fit data (%d points, need >=3). "
                "Check filament is loaded/primed and the nozzle is hot."
                % len(pa_sets))
        coeff = robust_fit(robust, pa_sets)
        base = interpolate_2d(saturate(current_speed / 60.0, 100., 400.),
                              saturate(temp, 210., 250.),
                              INTERP_SPEEDS, INTERP_TEMPS,
                              self.interp_fit_factor)
        fit_val = saturate(base * (max(dps) - min(dps)), 500., self.fit_val_max)
        k_raw = saturate(coeff[0] * fit_val + coeff[1], eff_pa_min, self.pa_max)
        # Per-nozzle calibration gain anchoring the absolute output to a real PA
        # test print (corrects the measurement-geometry scale).
        scale = self._pa_scale_for(nozzle_d)
        best_k = saturate(k_raw * scale, eff_pa_min, self.pa_max)
        if math.isnan(best_k):
            raise self.cmderr("Flow cal: PA computation returned NaN")
        logger.info("Flow cal: FitVal=%.2f K=%.6f (%.2fmm nozzle, %.0f mm/s)",
                    fit_val, best_k, nozzle_d, current_speed / 60.0)
        if self._verbose:
            self.gcode.respond_info(
                "Flow cal fit @%.0f mm/s: points=%d  dP=[%d..%d]  FitVal=%.0f  "
                "slope=%.3g int=%.4f  K_raw=%.4f  -> K=%.4f "
                "(pa_scale x%.2f @ %.2fmm)"
                % (current_speed / 60.0, len(pa_sets), int(min(dps)),
                   int(max(dps)), fit_val, coeff[0], coeff[1], k_raw, best_k,
                   scale, nozzle_d))
        return best_k, current_speed / 60.0

    def _pa_scale_for(self, nozzle_d):
        # Per-nozzle gain; falls back to the global pa_scale for off-grid sizes.
        return self.nozzle_pa_scale.get(round(nozzle_d, 2), self.pa_scale)

    def _setup_measure_center(self):
        # Capture the zig-zag center ONCE (toolhead is over the chute). The
        # measurement moves use absolute targets around this center, so the head
        # never drifts across repeated runs (RUNS=N).
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        self._center = pos[0] if self.measure_axis == 'X' else pos[1]
        self.gcode.respond_info(
            "Flow cal: kinematic measure on %s around %.1f (+/-%.1f), "
            "high move %d ms" % (self.measure_axis, self._center,
                                 self.measure_travel / 2.0, self.high_move_ms))
        self._run("G90")  # absolute XYZ so the zig-zag targets are correct

    def flow_calibration_operation(self, high_speed, temp, nozzle_d):
        cs = self.cs1237
        self._check_can_extrude()
        eff_pa_min = self.pa_min
        if nozzle_d > 0.4:
            eff_pa_min = self.pa_min - self.pa_min_nozzled_comp

        cs.enable()
        best_Ks = []
        speeds = []
        try:
            # First read after enable can be a settling transient — discard it.
            cs.query_diff()
            _, init_press = cs.query_diff()
            if init_press is None:
                raise self.cmderr(
                    "Flow cal: CS1237 sensor not responding (check the "
                    "nozzle_mcu socat tunnel). Aborting.")
            logger.debug("Flow cal: init_press=%d", init_press)
            if self._verbose:
                self.gcode.respond_info(
                    "Flow cal: sensor baseline (init_press)=%d" % init_press)
            cs.calibration(START_CALIBRATION, ZERO_SPEED)

            if self.enable_dyna_PA:
                speed = INIT_SPEED_300
                cnt = 3
            else:
                speed = high_speed
                cnt = 1

            for c in range(cnt):
                cali_speed = speed - INIT_SPEED_100 * c
                block_cnt = 0
                abn_cnt = 0
                set_ks = []
                dps = []
                robust = []
                last_bpv = [0]
                # Throwaway pass to init DeltaP
                self._measure_pa_point(cali_speed, PA_SWEEP_MIN - PA_STEP,
                                       last_bpv)
                k = PA_SWEEP_MIN
                while k <= PA_SWEEP_MAX:
                    abnormal, delta_p = self._measure_pa_point(cali_speed, k,
                                                               last_bpv)
                    bpv = last_bpv[0]
                    if abs(init_press - bpv) > BLOCK_THRESHOLD:
                        block_cnt += 1
                    if block_cnt > BLOCK_MAX_CNT:
                        raise self.cmderr(
                            "Flow cal: nozzle appears blocked (filament stuck)")
                    if not abnormal:
                        if (k != PA_SWEEP_MAX and delta_p > 1e-6
                                and delta_p < self.delta_press_max):
                            set_ks.append(k - PA_STEP)
                            dps.append(delta_p)
                            robust.append([delta_p, 1.0])
                    else:
                        if k >= 0.03:
                            abn_cnt += 1
                    if abn_cnt > ((PA_SWEEP_MAX - PA_SWEEP_MIN)
                                  / PA_STEP / 2.0):
                        raise self.cmderr(
                            "Flow cal: too many abnormal measurements — "
                            "check filament/temperature/sensor")
                    k += PA_STEP
                best_k, cur = self._fitting_solution(
                    cali_speed, temp, set_ks, dps, robust, eff_pa_min, nozzle_d)
                best_Ks.append(best_k)
                speeds.append(cur)
            best_Ks.reverse()
            speeds.reverse()
        finally:
            cs.calibration(STOP_CALIBRATION, ZERO_SPEED)
            cs.disable()
        return best_Ks, speeds

    def _apply_result(self, best_Ks, speeds, filament_index):
        if not best_Ks:
            raise self.cmderr("Flow cal: no result produced")
        if self.enable_dyna_PA and len(best_Ks) >= 3:
            self.inter_bestk_map[filament_index] = list(best_Ks)
            self.inter_speeds = speeds
            # A strictly increasing K-vs-speed set is considered an
            # UNSTABLE/FAILED dynamic calibration.
            if best_Ks[0] < best_Ks[1] < best_Ks[2]:
                self.calib_status_map[filament_index] = CALIBRATION_FAILED
                srt = sorted(best_Ks)
                if abs(srt[0] - srt[2]) > 0.002:
                    raise self.cmderr(
                        "Flow cal: unstable dynamic result (spread %.4f > "
                        "0.002), cannot derive a PA value." % abs(srt[0] - srt[2]))
                # Small spread -> use the median K as a static fallback.
                pa = srt[1]
                self.gcode.respond_info(
                    "Flow cal: WARNING unstable dynamic result, applying median "
                    "K=%.4f as a static fallback." % pa)
            else:
                # Smoothing corrections (clamp a single-point dip/spike to its
                # neighbour), then median as the applied static PA (vanilla
                # cannot vary PA per-move).
                if best_Ks[0] > best_Ks[1] and best_Ks[1] < best_Ks[2]:
                    best_Ks[2] = best_Ks[1]
                if best_Ks[0] < best_Ks[1] and best_Ks[1] > best_Ks[2]:
                    best_Ks[0] = best_Ks[1]
                pa = sorted(best_Ks)[len(best_Ks) // 2]
                self.calib_status_map[filament_index] = CALIBRATION_SUCCESS
        else:
            pa = round(best_Ks[0] * 1000.0) / 1000.0
            self.calib_status_map[filament_index] = CALIBRATION_SUCCESS
        self.calib_pa_map[str(filament_index)] = pa
        self._set_pa(pa)
        return pa

    # ------------------------------------------------------------------
    # Cleanup — poop drop, wipe, cooldown (goal requirement)
    # ------------------------------------------------------------------
    def _cleanup(self):
        if self.do_poop_cleanup:
            self._try_run(self.fan_on_tmpl.format(speed=255), "nozzle fan on")
            self._try_run(self.flush_poop_macro, "flush poop")
            self._try_run(self.wipe_gcode, "wipe")
        # Cool down only when standalone — KEEP_HOT (e.g. from a print-start
        # macro) leaves the nozzle at temperature so the print can continue.
        if not self._keep_hot:
            self._try_run("M104 S%.0f" % self.cooldown_temp, "cooldown")
        # restore absolute extrusion (we used M83 during the sweep)
        self._try_run("M82", "restore E mode")
        if self.do_poop_cleanup:
            self._try_run(self.fan_off_macro, "nozzle fan off")
        # Turn the ACE feed assist back off for the tool we calibrated.
        if self._fa_tool is not None:
            self._try_run(self.ace_feed_assist_off.format(tool=self._fa_tool),
                          "ace feed assist off")
            self._fa_tool = None

    # ------------------------------------------------------------------
    # Temperature resolution
    # ------------------------------------------------------------------
    def _resolve_temp(self, gcmd, ace_temp):
        t = gcmd.get_float('TEMPERATURE', None, minval=0.)
        if t is None:
            t = gcmd.get_float('EXTRUDERTEMP', None, minval=0.)
        if (t is None or t <= 0) and ace_temp:
            if float(ace_temp) >= 150.:
                t = float(ace_temp)
            else:
                self.gcode.respond_info(
                    "Flow cal: ACE reported an implausible temp (%s C) for this "
                    "slot — falling back. Pass TEMPERATURE= to override."
                    % ace_temp)
        if t is None or t <= 0:
            cur = self._nozzle_temp()
            if cur >= 210.:
                t = cur
                self.gcode.respond_info(
                    "Flow cal: no temperature given, using current %.0f C" % cur)
            else:
                t = 220.
                self.gcode.respond_info(
                    "Flow cal: no temperature given, defaulting to 220 C")
        if t < 210. or t > 250.:
            self.gcode.respond_info(
                "Flow cal: WARNING temp %.0f C is outside the interpolation "
                "range [210-250], result may be inaccurate" % t)
        return t

    # ------------------------------------------------------------------
    # GCode command
    # ------------------------------------------------------------------
    cmd_FLOW_CALIBRATE_help = (
        "Auto-calibrate pressure advance using the CS1237 nozzle pressure "
        "sensor. Params: TOOL= (ACE slot), TEMPERATURE=, SPEED= (mm/s), "
        "NOZZLE_DIAMETER=, BEDTEMP=")

    def cmd_FLOW_CALIBRATE(self, gcmd):
        # Always close any open selection popup first.
        self._close_popup()
        if self.cs1237 is None:
            self.cs1237 = self._resolve_cs1237()
        if self.cs1237 is None:
            raise gcmd.error("Flow cal: [cs1237] section not configured")
        if (isinstance(self.cs1237, _ForeignCS1237Adapter)
                and not self.cs1237.stock_calibration):
            raise gcmd.error(
                "Flow cal: the installed [cs1237] module has no stock "
                "calibration commands. This needs the original Anycubic "
                "nozzle-MCU firmware (the open-source toolhead firmware does "
                "not expose them).")

        tool = gcmd.get_int('TOOL', None)
        nozzle_d = gcmd.get_float('NOZZLE_DIAMETER',
                                  getattr(self._extruder(), 'nozzle_diameter',
                                          0.4), above=0.)
        speed = gcmd.get_float('SPEED', self.cali_speed_default, above=0.)
        high_speed = saturate(speed, 90., 400.) * 60.0
        save = gcmd.get_int('SAVE', 1 if self.save_to_config else 0)
        self._verbose = gcmd.get_int('VERBOSE', 1 if self.verbose else 0) > 0
        # KEEP_HOT=1 skips the final cooldown (for print-start macro use).
        self._keep_hot = gcmd.get_int('KEEP_HOT', 0) > 0
        runs = gcmd.get_int('RUNS', self.measure_runs, minval=1)

        # USE_ACE=0 forces rack mode even if an ACE is connected (Anycubic compat)
        use_ace = gcmd.get_int('USE_ACE', None)
        if use_ace == 0:
            units = []
        else:
            units = self._connected_units()
        ace_active = len(units) > 0
        orig_pa = self._current_pa()
        filament_index = RACK_FILAMENT_INDEX
        ace_slot = None

        # --- Pre-flight (no motion / no cleanup): resolve ACE slot & temp ---
        if ace_active:
            slots = self._occupied_slots(units)
            if not slots:
                raise gcmd.error(
                    "ACE connected but no slot is loaded — insert filament "
                    "into the ACE for flow calibration.")
            if tool is None:
                # No tool given -> ask the user via Mainsail/Fluidd popup.
                # Propagate the params the user passed so the buttons keep them.
                extra = []
                for key in ('SAVE', 'VERBOSE', 'USE_ACE', 'KEEP_HOT',
                            'TEMPERATURE', 'EXTRUDERTEMP', 'SPEED',
                            'NOZZLE_DIAMETER', 'BEDTEMP', 'RUNS'):
                    val = gcmd.get(key, None)
                    if val is not None:
                        extra.append("%s=%s" % (key, val))
                extra_args = (" " + " ".join(extra)) if extra else ""
                self._show_slot_popup(slots, extra_args)
                self.gcode.respond_info(
                    "Flow cal: select a slot in the popup (or run "
                    "FLOW_CALIBRATE TOOL=<n>).")
                return
            match = [s for s in slots if s['tool'] == tool]
            if not match:
                avail = ", ".join("T%s" % s['tool'] for s in slots)
                raise gcmd.error(
                    "Flow cal: T%d is not a loaded ACE slot. Available: %s"
                    % (tool, avail))
            ace_slot = match[0]
            temp = self._resolve_temp(gcmd, ace_slot['temp'])
            filament_index = tool
        else:
            temp = self._resolve_temp(gcmd, None)

        # --- Calibration (motion + sensor): cleanup guaranteed afterwards ---
        try:
            # Make sure the part-cooling fan is OFF before heating, so the nozzle
            # doesn't heat against a fan left running by a previous cooldown.
            self._try_run(self.fan_off_macro, "fan off (pre-heat)")
            # Always home first (goal: "immer erst G28"). Required before any
            # TO_THROW_POSITION / CUT_TIP / wipe move.
            if not self._is_homed('xyz'):
                self.gcode.respond_info("Flow cal: homing axes...")
                self._run(self.home_macro)
            if ace_active:
                self._ensure_loaded(tool, temp)
                if not self._is_homed('xyz'):
                    self._run(self.home_macro)
                self._try_run(self.throw_position_macro, "throw position")
                self._run("M109 S%.0f" % temp)
                # Pump the ACE buffer so the extruder doesn't fight the bowden.
                if self.ace_feed_assist:
                    self._try_run(self.ace_feed_assist_on.format(tool=tool),
                                  "ace feed assist on")
                    self._fa_tool = tool
                # Fresh prime + poop drop for a defined, pressurized start state.
                self._pre_cal_prime()
            else:
                self._rack_prep(gcmd, temp)

            self.calib_status_map[filament_index] = CALIBRATION_UNCALIBRATED
            self.gcode.respond_info(
                "Flow calibration: temp=%.0f C, speed=%.0f mm/s, nozzle=%.2f mm"
                "%s%s" % (temp, high_speed / 60.0, nozzle_d,
                          (", T%d" % filament_index) if ace_active else " (rack)",
                          (", %d runs (median)" % runs) if runs > 1 else ""))

            # Capture the zig-zag center once (head is over the chute now).
            self._setup_measure_center()
            results = []
            speeds = []
            for r in range(runs):
                best_Ks, speeds = self.flow_calibration_operation(
                    high_speed, temp, nozzle_d)
                results.append(best_Ks[0])
                if runs > 1:
                    self.gcode.respond_info(
                        "Flow cal: run %d/%d -> K=%.4f"
                        % (r + 1, runs, best_Ks[0]))
            # Median across runs (rejects the unstable first-run-after-swap).
            results.sort()
            median_k = results[len(results) // 2]
            pa = self._apply_result([median_k], speeds, filament_index)
            self.gcode.respond_info(
                "Flow calibration COMPLETE: pressure_advance = %.4f%s%s"
                % (pa, (" (T%d)" % filament_index) if ace_active else "",
                   (" [median of %d]" % runs) if runs > 1 else ""))
            if save:
                self._save_to_config(pa)
        except self.cmderr as e:
            self.calib_status_map[filament_index] = CALIBRATION_FAILED
            if orig_pa is not None:
                try:
                    self._set_pa(orig_pa)
                except Exception:
                    pass
            raise
        except Exception as e:
            self.calib_status_map[filament_index] = CALIBRATION_FAILED
            if orig_pa is not None:
                try:
                    self._set_pa(orig_pa)
                except Exception:
                    pass
            raise gcmd.error("Flow calibration failed: %s" % e)
        finally:
            self._cleanup()

    def cmd_PA_AUTO_CALIBRATE(self, gcmd):
        # Anycubic-compatible entry point. USE_ACE is auto-detected, but honour
        # an explicit USE_ACE=0 to force rack mode.
        self.cmd_FLOW_CALIBRATE(gcmd)


def load_config(config):
    return FlowCalibration(config)
