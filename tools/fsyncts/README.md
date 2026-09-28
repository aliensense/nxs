# fsyncts: the frame-sync proof by hardware timestamps

One libargus capture session over two capture ids; per frame, the capture
metadata carries both sensors' start-of-frame stamps on the Tegra TSC
(`Ext::ISensorTimestampTsc`, `getSensorSofTimestampTsc` for the first device
of the session, `getSensorSofTimestampTsc2` for the second). Their difference
is the sensor-side skew, independent of delivery: the buffer PTS a GStreamer
consumer sees are delivery times (millisecond scheduling jitter) and cannot
judge sync.

Build on the Jetson against the multimedia API headers (the package
`nvidia-l4t-jetson-multimedia-api` at the booted L4T build, installed or
unpacked with `dpkg-deb -x`):

```sh
g++ -std=c++11 -O2 -I "$MMAPI/argus/include" fsyncts.cpp \
    -L/usr/lib/aarch64-linux-gnu/nvidia -lnvargus_socketclient -lEGL -lpthread -o fsyncts
```

Run with the tool's choreography, consumer first and the CSI gate second, on a
world `nxs <port> on` and `trigger fsync` brought up (stop the viewers first):

```sh
./fsyncts <idA> <idB> <mode index> <frames> [exposure_us] [fps] > sof.tsv
```

Columns: frame counter, both frame numbers, both stamps as seen from each
stream's metadata, both end-of-frame stamps; nanoseconds. Bench 2026-09-16,
1920x1080 pair at 70 fps under the hub's generator: skew −5..+5 µs over 200
frames, frame period 14.2852 ms with 3 µs standard deviation on both heads.
