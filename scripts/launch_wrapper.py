#!/usr/bin/env python3
import sys
from pathlib import Path

from tools.wrapper_main import main


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv))
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exc:
        print(f"[{Path(sys.argv[0]).name}] ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
