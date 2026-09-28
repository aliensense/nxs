"""ROS 2 `header.stamp` source names. A leaf module with no imports, so the CLI
parser and launch description name the modes without importing the bridge."""

STAMP_SYNCED = "synced"
STAMP_DEVICE = "device"
STAMP_ARRIVAL = "arrival"
STAMP_ITOW = "itow"

# Every mode the bridge accepts; the CLI and launch surfaces derive their
# choice lists from this tuple.
STAMP_MODES = (STAMP_SYNCED, STAMP_DEVICE, STAMP_ARRIVAL, STAMP_ITOW)
