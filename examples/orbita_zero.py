"""Set the zero of Orbita actuator."""
import logging
from reachy_pyluos_hal.reachy import Reachy


def main():
    """Connect to reachy using reachy_pyluos_hal and set zero values."""
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger()
    logger.info('Starting zero setting of Orbita, make sure that the head is correctly positioned.')

    try:
        r = Reachy(logging.getLogger())
        logger.info('Success opening Reachy.')
    except (TimeoutError, OSError, IOError):
        logger.error('Zero setting failed, make sure that there are no other scripts or notebooks accessing the robot.')

    r.start()
    r.set_orbita_values('zero', 'neck', {})
    logger.info('New zeros of Orbita are set!')
    r.stop()


if __name__ == '__main__':
    main()
