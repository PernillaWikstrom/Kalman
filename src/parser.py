import argparse


def get_parser():
    parser = argparse.ArgumentParser(description="Set optional parameters")

    parser.add_argument(
        "--std_measurements",
        type=float,
        default=3.0,
        help="Measuremetns standard deviation of positions [m] [default: %(default)s ]",
    )
    parser.add_argument(
        "--std_process",
        type=float,
        default=0.2,
        help="Acceleration standard deviation [m/s²] [default: %(default)s ]",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=1,
        help="Sampling time of measuremetns [s] [default: %(default)s ]",
    )
    return parser.parse_args()
