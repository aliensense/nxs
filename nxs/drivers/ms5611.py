"""
MS5611 driver for NXS VM.

TE Connectivity MS5611-01BA03 barometric pressure sensor. A 24-bit
delta-sigma ADC digitises pressure (10..1200 mbar) and die temperature
(-40..+85 C); six factory calibration coefficients live in a 128-bit
PROM. The interface is a command protocol (not a register map): RESET,
CONVERT D1/D2 (a per-OSR opcode), ADC READ, PROM READ. The datasheet's
first- and second-order compensation polynomial runs on-device in the
VM's 64-bit fixed-point path, so the wire carries compensated SI
(pressure in pascal, temperature in kelvin) with no host-side math.

Oversampling (osr) is encoded in the CONVERT opcode, so it changes the
measure bytecode rather than a patchable register: it is a compile-time
config key with one measure variant per ratio, and each variant's
conversion sleep is sized to that ratio's maximum conversion time. The
part has no data-ready pin and each conversion is command-paced, so the
trigger is poll and sample_rate tunes the poll cadence.

Bus: I2C command protocol. The CSB strap selects address 0x76 or 0x77;
NXS scans both on load. (The silicon also has a SPI port, but the
command/compensation datapath is expressed here over the I2C command
transport via I2cCommandDriver.)

Config keys:
    osr          256 | 512 | 1024 | 2048 | 4096  (default 4096)  compile-time
    sample_rate  1 | 2 | 5 | 10 | 25 Hz          (default 10)    runtime param

mikroBUS pin mapping (I2C command part; no data-ready / reset pin):
    SDA (PA8)  -> SDA: I2C data
    SCL (PC4)  -> SCL: I2C clock
    INT/RST/AN/PWM unused (MS5611 has neither a DRDY output nor a
    hardware reset pin; RESET is the 0x1E command).

Datasheet: TE MS5611-01BA03 (ENG_DS_MS5611-01BA03_B3)
"""

from nxs import I2cCommandDriver, Sample


class Ms5611(I2cCommandDriver):
    """MS5611 barometer -- I2C command protocol, on-device int64 compensation."""

    PINS = {}                       # command/convert-paced; no DRDY line
    I2C_ADDRS = [0x76, 0x77]        # CSB strap selects the address LSB
    WHO_AM_I_REG = 0x00
    WHO_AM_I_VALUES = []            # no identity register on this part
    WHO_AM_I_SKIP_REASON = (
        "MS5611 has no WHO_AM_I / identity register; it is a command-"
        "protocol part whose presence is established by RESET reloading "
        "the PROM and the coefficient reads in configure()."
    )

    # ---- Command set (datasheet, Commands section) ----------------
    CMD_RESET = 0x1E                # reloads the calibration PROM into the chip
    PROM_C1 = 0xA2                  # C1..C6 at 0xA2,0xA4,0xA6,0xA8,0xAA,0xAC (16-bit BE)
    # ADC READ = 0x00 (24-bit result); CONVERT opcodes per OSR (D1 = pressure,
    # D2 = temperature) and each ratio's max conversion time:
    #   OSR    D1     D2     t_conv,max
    #   256   0x40   0x50    0.60 ms
    #   512   0x42   0x52    1.17 ms
    #   1024  0x44   0x54    2.28 ms
    #   2048  0x46   0x56    4.54 ms
    #   4096  0x48   0x58    9.04 ms

    def probe(self):
        # RESET reloads the factory PROM; datasheet reload time 2.8 ms.
        self.send_command(self.CMD_RESET)
        self.sleep_ms(3)

    def configure(self, config):
        # Poll-paced part with no rate register: declare sample_rate so the
        # compiler patches the poll-loop interval (no register write / tag).
        # Every listed rate fits the full convert+read cycle even at OSR 4096
        # (~20 ms, so <= 50 Hz; capped at 25 Hz for margin).
        self.declare_param("sample_rate", values=[1, 2, 5, 10, 25],
                           default=10, unit="Hz")

        # Factory calibration coefficients: read ONCE, bound to self.<attr>
        # so they persist into measure(). 16-bit big-endian PROM words.
        self.c1 = self.read(self.PROM_C1 + 0, 2)   # SENS_T1   pressure sensitivity
        self.c2 = self.read(self.PROM_C1 + 2, 2)   # OFF_T1    pressure offset
        self.c3 = self.read(self.PROM_C1 + 4, 2)   # TCS       temp. coeff. of sensitivity
        self.c4 = self.read(self.PROM_C1 + 6, 2)   # TCO       temp. coeff. of offset
        self.c5 = self.read(self.PROM_C1 + 8, 2)   # T_REF     reference temperature
        self.c6 = self.read(self.PROM_C1 + 10, 2)  # TEMPSENS  temperature sensitivity

        # Polynomial output: P in Pa (100009 -> 100009 Pa, scale 1.0 -> pascal);
        # TEMP in centi-degC (2007 -> 20.07 C, scale 0.01 + 273.15 -> kelvin).
        self.set_output([
            {'name': 'pressure',    'type': 'int32', 'byte_order': 'big',
             'scale': 1.0},
            {'name': 'temperature', 'type': 'int32', 'byte_order': 'big',
             'scale': 0.01, 'offset': 273.15},
        ])
        self.set_sample_size(8)     # two int32 fields

    # ---- Measure variants: one per OSR (compile-time `osr` key) ----
    # Each issues CONVERT D2 then D1 at its OSR, waits that ratio's max
    # conversion time, reads the 24-bit ADC, and runs the datasheet's
    # first- and second-order compensation on-device (int64 path).

    @I2cCommandDriver.measure_loop(trigger="poll", sample_rate=10,
                                   when=("osr", 256))
    def measure_osr256(self):
        self.send_command(0x50)      # convert D2 (temperature), OSR=256
        self.sleep_ms(1)              # max conversion 0.60 ms
        d2 = self.read(0x00, 3)         # 24-bit temperature ADC
        self.send_command(0x40)      # convert D1 (pressure), OSR=256
        self.sleep_ms(1)              # max conversion 0.60 ms
        d1 = self.read(0x00, 3)         # 24-bit pressure ADC
        # First-order compensation (datasheet).
        dt   = d2 - self.c5 * 2**8
        temp = 2000 + dt * self.c6 // 2**23
        off  = self.c2 * 2**16 + self.c4 * dt // 2**7
        sens = self.c1 * 2**15 + self.c3 * dt // 2**8

        # Second-order temperature compensation (datasheet).
        if temp < 2000:
            t2    = dt * dt // 2**31
            f     = (temp - 2000) * (temp - 2000)
            off2  = 5 * f // 2
            sens2 = 5 * f // 4
            if temp < -1500:
                g     = (temp + 1500) * (temp + 1500)
                off2  = off2 + 7 * g
                sens2 = sens2 + 11 * g // 2
            temp = temp - t2
            off  = off - off2
            sens = sens - sens2

        p = (d1 * sens // 2**21 - off) // 2**15
        return Sample(pressure=p, temperature=temp)

    @I2cCommandDriver.measure_loop(trigger="poll", sample_rate=10,
                                   when=("osr", 512))
    def measure_osr512(self):
        self.send_command(0x52)      # convert D2 (temperature), OSR=512
        self.sleep_ms(2)              # max conversion 1.17 ms
        d2 = self.read(0x00, 3)         # 24-bit temperature ADC
        self.send_command(0x42)      # convert D1 (pressure), OSR=512
        self.sleep_ms(2)              # max conversion 1.17 ms
        d1 = self.read(0x00, 3)         # 24-bit pressure ADC
        # First-order compensation (datasheet).
        dt   = d2 - self.c5 * 2**8
        temp = 2000 + dt * self.c6 // 2**23
        off  = self.c2 * 2**16 + self.c4 * dt // 2**7
        sens = self.c1 * 2**15 + self.c3 * dt // 2**8

        # Second-order temperature compensation (datasheet).
        if temp < 2000:
            t2    = dt * dt // 2**31
            f     = (temp - 2000) * (temp - 2000)
            off2  = 5 * f // 2
            sens2 = 5 * f // 4
            if temp < -1500:
                g     = (temp + 1500) * (temp + 1500)
                off2  = off2 + 7 * g
                sens2 = sens2 + 11 * g // 2
            temp = temp - t2
            off  = off - off2
            sens = sens - sens2

        p = (d1 * sens // 2**21 - off) // 2**15
        return Sample(pressure=p, temperature=temp)

    @I2cCommandDriver.measure_loop(trigger="poll", sample_rate=10,
                                   when=("osr", 1024))
    def measure_osr1024(self):
        self.send_command(0x54)      # convert D2 (temperature), OSR=1024
        self.sleep_ms(3)              # max conversion 2.28 ms
        d2 = self.read(0x00, 3)         # 24-bit temperature ADC
        self.send_command(0x44)      # convert D1 (pressure), OSR=1024
        self.sleep_ms(3)              # max conversion 2.28 ms
        d1 = self.read(0x00, 3)         # 24-bit pressure ADC
        # First-order compensation (datasheet).
        dt   = d2 - self.c5 * 2**8
        temp = 2000 + dt * self.c6 // 2**23
        off  = self.c2 * 2**16 + self.c4 * dt // 2**7
        sens = self.c1 * 2**15 + self.c3 * dt // 2**8

        # Second-order temperature compensation (datasheet).
        if temp < 2000:
            t2    = dt * dt // 2**31
            f     = (temp - 2000) * (temp - 2000)
            off2  = 5 * f // 2
            sens2 = 5 * f // 4
            if temp < -1500:
                g     = (temp + 1500) * (temp + 1500)
                off2  = off2 + 7 * g
                sens2 = sens2 + 11 * g // 2
            temp = temp - t2
            off  = off - off2
            sens = sens - sens2

        p = (d1 * sens // 2**21 - off) // 2**15
        return Sample(pressure=p, temperature=temp)

    @I2cCommandDriver.measure_loop(trigger="poll", sample_rate=10,
                                   when=("osr", 2048))
    def measure_osr2048(self):
        self.send_command(0x56)      # convert D2 (temperature), OSR=2048
        self.sleep_ms(5)              # max conversion 4.54 ms
        d2 = self.read(0x00, 3)         # 24-bit temperature ADC
        self.send_command(0x46)      # convert D1 (pressure), OSR=2048
        self.sleep_ms(5)              # max conversion 4.54 ms
        d1 = self.read(0x00, 3)         # 24-bit pressure ADC
        # First-order compensation (datasheet).
        dt   = d2 - self.c5 * 2**8
        temp = 2000 + dt * self.c6 // 2**23
        off  = self.c2 * 2**16 + self.c4 * dt // 2**7
        sens = self.c1 * 2**15 + self.c3 * dt // 2**8

        # Second-order temperature compensation (datasheet).
        if temp < 2000:
            t2    = dt * dt // 2**31
            f     = (temp - 2000) * (temp - 2000)
            off2  = 5 * f // 2
            sens2 = 5 * f // 4
            if temp < -1500:
                g     = (temp + 1500) * (temp + 1500)
                off2  = off2 + 7 * g
                sens2 = sens2 + 11 * g // 2
            temp = temp - t2
            off  = off - off2
            sens = sens - sens2

        p = (d1 * sens // 2**21 - off) // 2**15
        return Sample(pressure=p, temperature=temp)

    @I2cCommandDriver.measure_loop(trigger="poll", sample_rate=10,
                                   when=("osr", 4096),
                                   default=True)
    def measure_osr4096(self):
        self.send_command(0x58)      # convert D2 (temperature), OSR=4096
        self.sleep_ms(10)              # max conversion 9.04 ms
        d2 = self.read(0x00, 3)         # 24-bit temperature ADC
        self.send_command(0x48)      # convert D1 (pressure), OSR=4096
        self.sleep_ms(10)              # max conversion 9.04 ms
        d1 = self.read(0x00, 3)         # 24-bit pressure ADC
        # First-order compensation (datasheet).
        dt   = d2 - self.c5 * 2**8
        temp = 2000 + dt * self.c6 // 2**23
        off  = self.c2 * 2**16 + self.c4 * dt // 2**7
        sens = self.c1 * 2**15 + self.c3 * dt // 2**8

        # Second-order temperature compensation (datasheet).
        if temp < 2000:
            t2    = dt * dt // 2**31
            f     = (temp - 2000) * (temp - 2000)
            off2  = 5 * f // 2
            sens2 = 5 * f // 4
            if temp < -1500:
                g     = (temp + 1500) * (temp + 1500)
                off2  = off2 + 7 * g
                sens2 = sens2 + 11 * g // 2
            temp = temp - t2
            off  = off - off2
            sens = sens - sens2

        p = (d1 * sens // 2**21 - off) // 2**15
        return Sample(pressure=p, temperature=temp)
