"""Package entry point: `python3 -m nxs ...` is an alias for the `nxs`
command."""
import sys

from nxs.cli import main

sys.exit(main())
