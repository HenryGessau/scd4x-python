from datetime import datetime, timezone
from textwrap import dedent

from library.scd4x import SCD41


def single_shot_reading():
    device = SCD41(quiet=False)
    device.wake_up()

    co2, temperature, relative_humidity, timestamp = device.measure_single_shot()
    date = datetime.fromtimestamp(timestamp, timezone.utc)
    msg = dedent(f"""
        Time:        {date.strftime("%Y/%m/%d %H:%M:%S:%f %Z %z")}
        CO2:         {co2:.2f}PPM
        Temperature: {temperature:.4f}c
        Humidity:    {relative_humidity:.2f}%RH""")
    print(msg)

    device.power_down()


def main():
    # TODO: argparse and stuff
    single_shot_reading()


if __name__ == '__main__':
    main()
