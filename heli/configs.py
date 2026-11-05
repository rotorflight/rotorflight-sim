"""
Load a Rotorflight CLI dump (`dump all` / `diff all` from the Configurator or CLI) for the
SITL: keeps the flight-relevant settings (profiles, rates, mixer, servos, governor,
filters, modes), drops what is tied to the real board or radio link.

Only the Python standard library is used.
"""

# whole commands that describe the board, its pins or its peripherals
SKIP_COMMANDS = {
    'resource', 'timer', 'dma', 'serial', 'board_name', 'board_design', 'manufacturer_id',
    'mcu_id', 'signature', 'led', 'mode_color', 'color', 'beacon', 'beeper', 'batch', 'save',
    'defaults', 'map', 'rxfail', 'vtxtable', 'vtx', 'osd_layout', 'version', 'exit',
    # in-flight adjustments (profile select, gain knobs) read channels the bridge does not
    # drive like the pilot's radio did; the dump's active profiles stay selected instead
    'adjfunc',
}
# settings for the hardware link (the SITL talks MSP RX, TCP UARTs, fake sensors)
SKIP_SETTINGS = (
    'serialrx_', 'motor_pwm_', 'dshot_', 'rx_spi_', 'spektrum_', 'sbus_', 'crsf_',
    'gyro_to_use', 'gyro_1_', 'gyro_2_', 'acc_hardware', 'baro_hardware', 'mag_hardware',
    'blackbox_device', 'sdcard_', 'flash_', 'pinio_', 'vcd_', 'osd_', 'vtx_', 'led_',
    'battery_voltage_meter_source', 'battery_current_meter_source', 'ibata_', 'vbat_scale',
    'vbat_divider', 'vbat_multiplier', 'esc_sensor_', 'telemetry_', 'frsky_', 'ledstrip_',
    'name', 'mco_', 'system_hse_mhz', 'i2c', 'spi_',
    # how the board sits in the airframe: the SITL's sensors already come in body axes
    # (a dump with align_board_yaw = 180 would otherwise invert roll and pitch)
    'align_', 'mag_align_', 'acc_align', 'gyro_align',
)


def load_dump(path):
    """Return the CLI commands to apply, in order."""
    out = []
    for raw in open(path, encoding='utf-8', errors='replace'):
        line = raw.strip()
        if not line or line.startswith('#'):
            continue
        word = line.split()[0].lower()
        if word in SKIP_COMMANDS:
            continue
        if word == 'set':
            name = line.split()[1].lower() if len(line.split()) > 1 else ''
            if any(name.startswith(p) for p in SKIP_SETTINGS):
                continue
        if word == 'feature' and line.split()[1].lstrip('-').upper() in (
                'RX_SERIAL', 'RX_PPM', 'RX_PARALLEL_PWM', 'RX_SPI', 'RX_MSP', 'SOFTSERIAL',
                'ESC_SENSOR', 'TELEMETRY', 'GPS', 'LED_STRIP', 'OSD', 'CMS', 'DASHBOARD'):
            continue
        out.append(line)
    return out
