# CS1237 strain-gauge ADC driver for the Anycubic Kobra S1 toolhead MCU
#
# Makes the CS1237 nozzle pressure-sensor protocol available to
# vanilla Klipper. It talks to the `nozzle_mcu` (reached through the
# socat tunnel, see vanilla-klipper-swu/tunneled-klipper.md).
#
# It registers config section [cs1237] and printer object 'cs1237', and is
# consumed by extras/flow_calibration.py.
#
# Config section (matches the existing [cs1237] block in printer.cfg):
#   [cs1237]
#   level_pin: nozzle_mcu:PA7
#   dout_pin:  nozzle_mcu:PA6
#   sclk_pin:  nozzle_mcu:PA5
#   register:  60
#   sensitivity: -2500
#   (head_block_sensitivity / scratch_sensitivity / self_check_sensitivity /
#    block_filament_sensitivity are read but only relevant for the
#    self-check / block-detection paths, not for flow calibration.)

import logging

logger = logging.getLogger("klippy.extras.cs1237")

# Calibration phase states
STOP_CALIBRATION = 0
START_CALIBRATION = 1

# Speed phase states -- NOTE: value 1 is unused!
ZERO_SPEED = 0
STOP_LOW_SPEED = 2
START_LOW_SPEED = 3
START_HIGH_SPEED = 4
STOP_HIGH_SPEED = 5

QUERY_TIMEOUT = 2.0  # seconds


def _i32(data):
    """Signed-32-bit conversion of a raw MCU value.

    Converts an unsigned 32-bit MCU value to signed ONLY when the top nibble is
    0xF (i.e. value in 0xF0000000..0xFFFFFFFF). This is NOT a full signed-32
    cast. We use mathematically-correct two's complement here; every consumer of
    these values uses *differences* of converted values.
    """
    data &= 0xffffffff
    if (data & 0xf0000000) == 0xf0000000:
        return -((data ^ 0xffffffff) + 1)
    return data


class CS1237Sensor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name()

        # --- Pin config -------------------------------------------------
        self.dout_pin = config.get('dout_pin')
        self.sclk_pin = config.get('sclk_pin')
        self.level_pin = config.get('level_pin')
        self.register = config.getint('register', 60)
        self.sensitivity = config.getint('sensitivity', -2500)
        # Read-but-unused-by-flow-cal (kept so the [cs1237] section loads
        # cleanly without "Option ... is not valid" errors).
        self.head_block_sensitivity = config.getint('head_block_sensitivity',
                                                     -300000)
        self.scratch_sensitivity = config.getint('scratch_sensitivity',
                                                  -100000)
        self.self_check_sensitivity = config.getint('self_check_sensitivity',
                                                     -400)
        self.block_filament_sensitivity = config.getint(
            'block_filament_sensitivity', -3000)
        self.data_bit = config.getint('data_bit', 24)

        # --- Resolve pins -> mcu + bare pin names -----------------------
        ppins = self.printer.lookup_object('pins')
        dout = ppins.lookup_pin(self.dout_pin)
        sclk = ppins.lookup_pin(self.sclk_pin)
        level = ppins.lookup_pin(self.level_pin)
        self.mcu = sclk['chip']
        self._dout_name = dout['pin']
        self._sclk_name = sclk['pin']
        self._level_name = level['pin']
        if not (dout['chip'] is sclk['chip'] is level['chip']):
            raise config.error(
                "cs1237: dout_pin, sclk_pin and level_pin must be on the "
                "same MCU")

        self.oid = None
        self._cq = None
        # Command handles
        self.cmd_reset = None
        self.cmd_enable = None
        self.cmd_calib_phase = None
        self.cmd_calib_val = None
        self.cmd_query_diff = None

        # Async query state
        self._completion = None
        self._pending = None          # 'cal' or 'diff' — which reply we await
        self.enable_count = 0

        # Latest cached values
        self.last_diff = 0
        self.last_raw = 0
        # Raw params of the most recent responses (for diagnostics)
        self._last_diff_params = {}
        self._last_cal_params = {}

        # NB: do NOT add_object('cs1237') here — Klipper auto-registers the
        # object returned by load_config under the section name 'cs1237'.
        self.mcu.register_config_callback(self._build_config)

        # Safe diagnostic: read the sensor over the tunnel (no motion/heat).
        # NB: the command name must NOT contain digits — Klipper's gcode parser
        # mis-parses names like "CS1237_QUERY" (it sees the command "CS1237").
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("QUERY_FLOW_SENSOR", self.cmd_QUERY_FLOW_SENSOR,
                               desc="Read the CS1237 nozzle sensor once "
                                    "(diagnostic, no motion/heating)")

    # ------------------------------------------------------------------
    def _build_config(self):
        self.oid = self.mcu.create_oid()
        self.mcu.add_config_cmd(
            "config_cs1237 oid=%d level_pin=%s dout_pin=%s sclk_pin=%s"
            " register=%d sensitivity=%d"
            % (self.oid, self._level_name, self._dout_name, self._sclk_name,
               self.register, self.sensitivity))

        self._cq = self.mcu.alloc_command_queue()
        self.cmd_reset = self.mcu.lookup_command(
            "reset_cs1237 oid=%c count=%c", cq=self._cq)
        self.cmd_enable = self.mcu.lookup_command(
            "enable_cs1237 oid=%c state=%c", cq=self._cq)
        self.cmd_calib_phase = self.mcu.lookup_command(
            "cs1237_calibration_phase oid=%c cali_state=%c speed_state=%c",
            cq=self._cq)
        self.cmd_calib_val = self.mcu.lookup_command(
            "cs1237_calibration_DataProcess oid=%c", cq=self._cq)
        self.cmd_query_diff = self.mcu.lookup_command(
            "query_cs1237_diff oid=%c", cq=self._cq)

        # Response handlers (API differs between mainline Klipper and the
        # klipper-kobra-s1 fork — see _register_response).
        self._register_response(self._handle_calibration_val,
                                "cs1237_calibration_Val")
        self._register_response(self._handle_diff, "cs1237_diff")
        # No-op handlers so stray reports don't trigger "unknown message".
        self._register_response(self._handle_state, "cs1237_state")
        self._register_response(self._handle_checkself, "cs1237_checkself_flag")

    # ------------------------------------------------------------------
    # MCU response registration (portable across Klipper variants)
    # ------------------------------------------------------------------
    def _get_serial(self):
        # Mainline: mcu._serial. klipper-kobra-s1 fork: mcu._conn_helper.get_serial()
        ch = getattr(self.mcu, '_conn_helper', None)
        if ch is not None and hasattr(ch, 'get_serial'):
            try:
                return ch.get_serial()
            except Exception:
                pass
        return getattr(self.mcu, '_serial', None)

    def _register_response(self, cb, name):
        serial = self._get_serial()
        msgformat = name
        # Skip messages the MCU does not declare (avoids a connect-time error
        # on the fork, whose register_serial_response validates the format).
        if serial is not None:
            try:
                msgs = serial.get_msgparser().messages_by_name
                if name not in msgs:
                    logger.info("cs1237: MCU has no '%s' message; "
                                "handler not registered", name)
                    return
                msgformat = msgs[name].msgformat
            except Exception:
                pass
        if hasattr(self.mcu, 'register_response'):
            # Mainline Klipper: takes the message NAME.
            self.mcu.register_response(cb, name, self.oid)
        elif hasattr(self.mcu, 'register_serial_response'):
            # klipper-kobra-s1 fork: takes the full message FORMAT string.
            self.mcu.register_serial_response(cb, msgformat, self.oid)
        elif serial is not None:
            serial.register_response(cb, name, self.oid)
        else:
            raise self.printer.command_error(
                "cs1237: cannot register MCU response '%s' — unsupported "
                "Klipper MCU API" % name)

    # ------------------------------------------------------------------
    # Low-level operations
    # ------------------------------------------------------------------
    def reset(self, count=3):
        """Reset the CS1237 ADC."""
        if count <= 0:
            count = 3
        self.cmd_reset.send([self.oid, count])
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.dwell(0.1)

    def enable(self):
        """Enable sensor sampling. Refcounted; the G4 P500
        settle dwell runs on every call (outside the refcount guard, as in Go).
        """
        if self.enable_count == 0:
            self.cmd_enable.send([self.oid, 1])
            logger.debug("cs1237 enabled")
        self.enable_count += 1
        gcode = self.printer.lookup_object('gcode')
        gcode.run_script_from_command("G4 P500")

    def disable(self):
        """Disable sensor sampling."""
        if self.enable_count > 0:
            self.enable_count -= 1
        if self.enable_count == 0:
            self.cmd_enable.send([self.oid, 0])
            logger.debug("cs1237 disabled")

    def force_disable(self):
        """Unconditionally disable (used in error cleanup)."""
        self.enable_count = 0
        self.cmd_enable.send([self.oid, 0])

    def calibration(self, cali_state, speed_state):
        """Mark a calibration phase boundary."""
        self.cmd_calib_phase.send([self.oid, cali_state, speed_state])

    def query_calibration_val(self):
        """Request and wait for (block_pre_val, target_val, real_val).

        Sends cs1237_calibration_DataProcess; the MCU answers
        with cs1237_calibration_Val. Returns (-1,-1,-1) on timeout.
        """
        self._completion = self.reactor.completion()
        self._pending = 'cal'
        self.cmd_calib_val.send([self.oid])
        params = self._completion.wait(self.reactor.monotonic()
                                       + QUERY_TIMEOUT)
        self._completion = None
        self._pending = None
        if params is None:
            logger.warning("cs1237_calibration_Val response timeout")
            return None, None, None
        return (params['block_pre_val'], params['target_val'],
                params['real_val'])

    def query_diff(self):
        """Request and wait for (diff, raw).

        Returns (-1,-1) on timeout. NOTE: init pressure uses the 2nd element
        (raw).
        """
        self._completion = self.reactor.completion()
        self._pending = 'diff'
        self.cmd_query_diff.send([self.oid])
        params = self._completion.wait(self.reactor.monotonic()
                                       + QUERY_TIMEOUT)
        self._completion = None
        self._pending = None
        if params is None:
            logger.warning("cs1237_diff response timeout")
            return None, None
        return params['diff'], params['raw']

    # ------------------------------------------------------------------
    # Response handlers
    # ------------------------------------------------------------------
    def _handle_calibration_val(self, params):
        # Params named BlockPreVal/TargetVal/RealVal,
        # with defensive None->0 handling for old MCUs.
        self._last_cal_params = dict(params)
        result = {
            'block_pre_val': _i32(params.get('BlockPreVal', 0) or 0),
            'target_val': _i32(params.get('TargetVal', 0) or 0),
            'real_val': _i32(params.get('RealVal', 0) or 0),
        }
        c = self._completion
        if c is not None and self._pending == 'cal':
            self._completion = None
            self._pending = None
            c.complete(result)

    def _handle_diff(self, params):
        # Params 'diff' and 'raw'.
        self._last_diff_params = dict(params)
        result = {
            'diff': _i32(params.get('diff', 0) or 0),
            'raw': _i32(params.get('raw', 0) or 0),
        }
        self.last_diff = result['diff']
        self.last_raw = result['raw']
        c = self._completion
        if c is not None and self._pending == 'diff':
            self._completion = None
            self._pending = None
            c.complete(result)

    def _handle_state(self, params):
        # Continuous report packet -- not used during flow calibration.
        pass

    def _handle_checkself(self, params):
        # Self-check flag -- not used during flow calibration.
        pass

    def cmd_QUERY_FLOW_SENSOR(self, gcmd):
        """Diagnostic: enable the sensor, read one (diff, raw) sample, disable.

        No toolhead motion and no heating — use this to confirm the socat tunnel
        and the CS1237 are alive before running a full FLOW_CALIBRATE.
        """
        if self.oid is None:
            raise self.printer.command_error(
                "cs1237: MCU not configured yet (is nozzle_mcu connected?)")
        # Show the MCU's declared message format so we can confirm field names.
        try:
            msgs = self._get_serial().get_msgparser().messages_by_name
            fmt = msgs['cs1237_diff'].msgformat
        except Exception as e:
            fmt = "(could not read format: %s)" % e
        gcmd.respond_info("cs1237_diff format: %s" % fmt)

        self.reset()
        self.enable()
        ok = False
        try:
            for i in range(4):
                diff, raw = self.query_diff()
                if raw is None:
                    gcmd.respond_info("sample %d: NO RESPONSE (timeout)"
                                      % (i + 1))
                else:
                    ok = True
                    gcmd.respond_info(
                        "sample %d: diff=%d raw=%d" % (i + 1, diff, raw))
                self.printer.lookup_object('gcode').run_script_from_command(
                    "G4 P200")
        finally:
            self.disable()
        if not ok:
            raise self.printer.command_error(
                "cs1237: NO RESPONSE — check the nozzle_mcu socat tunnel and "
                "that the nozzle_mcu exposes the CS1237 commands.")
        gcmd.respond_info(
            "CS1237 OK: sensor responding. 'raw' is the live ADC value "
            "(negative is normal); 'diff' stays 0 until a calibration phase.")

    def get_status(self, eventtime):
        return {
            'last_diff': self.last_diff,
            'last_raw': self.last_raw,
            'enabled': self.enable_count > 0,
        }


def load_config(config):
    return CS1237Sensor(config)
