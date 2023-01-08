import argparse
from datetime import datetime, timezone
from textwrap import dedent

from scd4x import SCD41


def single_shot_reading(verbose=False, debug=False):
    quiet = not debug
    device = SCD41(quiet=quiet)
    device.wake_up(quiet=quiet)

    co2, temperature, relative_humidity, timestamp = device.measure_single_shot()
    date = datetime.fromtimestamp(timestamp).strftime("%Y-%m-%d %H:%M:%S")
    msg = dedent(f"""
        Time:        {date}
        CO2:         {co2:.0f} PPM
        Temperature: {temperature:.1f}°C
        Humidity:    {relative_humidity:.1f}% RH""")
    if debug or verbose:
        print(msg)

    log = f'"{date}","{co2:.0f}","{temperature:.1f}","{relative_humidity:.1f}"\n'
    with open("/home/henry/.logs/SCD41.csv", 'a') as f:
        f.write(log)

    device.power_down()


def main():
    parser = argparse.ArgumentParser(description="Henry's CO2 sensor reader")
    parser.add_argument('-d', dest='debug', action='store_true', help='Debug')
    parser.add_argument('-v', dest='verbose', action='store_true', help='Verbose')
    args = parser.parse_args()

    single_shot_reading(verbose=args.verbose, debug=args.debug)


if __name__ == '__main__':
    main()
