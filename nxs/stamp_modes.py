"""ROS 2 `header.stamp` source names.

A leaf module with no imports: the CLI parser and the launch description
name these modes without paying for the bridge (and its yaml and client
dependencies) on every invocation.
"""

STAMP_SYNCED = "synced"
STAMP_DEVICE = "device"
STAMP_ARRIVAL = "arrival"
STAMP_ITOW = "itow"

# Every mode the bridge accepts — the CLI and launch surfaces derive
# their choice lists from this tuple.
STAMP_MODES = (STAMP_SYNCED, STAMP_DEVICE, STAMP_ARRIVAL, STAMP_ITOW)
