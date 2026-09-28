"""
MS5611 driver for NXS VM.

TE Connectivity MS5611-01BA03 barometer; datasheet ENG_DS_MS5611-01BA03_B3.
Bus: I2C command protocol at 0x76/0x77 (CSB strap): RESET, CONVERT D1/D2, ADC READ, PROM READ.
Config: osr 256|512|1024|2048|4096 (4096), compile-time, one measure variant per ratio;
    sample_rate 1|2|5|10|25 Hz (10), poll cadence.
Outputs: pressure (Pa), temperature (K), compensated on-device (first and second order).
Pins: none (no data-ready or reset pin; RESET is command 0x1E).
"""

from nxs import I2cCommandDriver, Sample


class Ms5611(I2cCommandDriver):
    """MS5611 barometer, I2C command protocol with on-device int64 compensation."""

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
    # ADC READ = 0x00 (24-bit); the CONVERT opcodes and max conversion times
    # per OSR are at each measure variant below.

    def probe(self):
        # RESET reloads the factory PROM; datasheet reload time 2.8 ms.
        self.send_command(self.CMD_RESET)
        self.sleep_ms(3)

    def configure(self, config):
        # No rate register: sample_rate patches the poll interval. At OSR 4096 a
        # full convert+read cycle is ~20 ms, so the ladder stops at 25 Hz.
        self.declare_params_from_descriptor()

        # Factory PROM coefficients (16-bit big-endian), read once and bound to
        # self so they persist into measure().
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
    # CONVERT D2 then D1, sleep the ratio's max conversion time, read the
    # 24-bit ADC, and run the datasheet's compensation on-device (int64).

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
