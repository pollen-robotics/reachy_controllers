"""Check if Orbita's fan is working."""
import logging
import time
from reachy_pyluos_hal.reachy import Reachy


def main():
    """Connect to reachy using reachy_pyluos_hal and check Orbita's fan."""
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger()

    try:
        r = Reachy('orbita', logging.getLogger())
        logger.info('Success opening Reachy.')
    except (TimeoutError, OSError, IOError):
        logger.error('Connection failed, make sure that there are no other scripts or notebooks accessing the robot.')
        return

    r.start()
    r.set_orbita_values('fan_trigger_temperature_threshold', 'neck', {'disk_top': 15})
    logger.info("Changing Orbita's fan trigger temperature for 5 seconds, check if you here the fan.")
    time.sleep(5.0)
    r.set_orbita_values('fan_trigger_temperature_threshold', 'neck', {'disk_top': 35})
    r.stop()


if __name__ == '__main__':
    main()
