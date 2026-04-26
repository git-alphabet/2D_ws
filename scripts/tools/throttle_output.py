#!/usr/bin/env python3
import argparse
import re
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Throttle noisy line-oriented process output."
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=30.0,
        help="Minimum seconds between matching lines with the same throttle key.",
    )
    parser.add_argument(
        "--pattern",
        action="append",
        default=[],
        help="Regex pattern to throttle. Can be specified multiple times.",
    )
    args = parser.parse_args()

    patterns = [re.compile(pattern) for pattern in args.pattern]
    last_seen: dict[str, float] = {}

    for line in sys.stdin:
        now = time.monotonic()
        throttle_key = None
        for pattern in patterns:
            match = pattern.search(line)
            if match:
                throttle_key = match.group(0)
                break

        if throttle_key is None or args.interval <= 0.0:
            print(line, end="", flush=True)
            continue

        # Keep launch prefixes such as "[ign gazebo-1]" independent by using
        # the whole line as the key. Identical warnings from the same process
        # are printed at most once per interval.
        throttle_key = line.rstrip("\n")
        previous = last_seen.get(throttle_key)
        if previous is None or now - previous >= args.interval:
            last_seen[throttle_key] = now
            print(line, end="", flush=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
