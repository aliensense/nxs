# gainpair: one gain on a synced pair, read on both heads

A frame-synced pair runs one gain: the leading link's capture session decides
it, and `nxsd` copies it to the following head each frame under the sensor's
register hold. `gainpair` reads both heads' analog gain registers once a frame
at the port's rate and says how closely the following head keeps up. It is a
bench tool: no verb calls it, and the wheel does not carry it.

It samples what the follower copies: the heads and the gain register of the
pack's `build_follow` for the port, the leader and the follower the port
record names, or the port's first two camera links on a pair a declared
`camera.gain_db` locks. `--leader`, `--follower` and `--reg` replace the
addresses and the register; `--bus` replaces the port's bus. Each sample takes
the bus lock every tool run shares in one attempt, so a frame another run holds
the bus in is skipped, and the tool writes nothing.

Copy `gainpair.py` to the host and run it with the interpreter the installed
`nxs` runs on, while a capture keeps both heads streaming:

```sh
nxs cam0 capture --frames 4200 &
"$(dirname "$(readlink -f "$(command -v nxs)")")/python" gainpair.py cam0 --seconds 60
```

It prints what it samples, a line whenever either head's gain changes, and
the summary:

```
sampling cam0: A at 0x1e, B at 0x1b, gain register 0x3514, 30 fps for 60 s
t=0.000 A=21.4dB B=21.4dB
t=7.233 A=22.9dB B=21.4dB
t=7.267 A=22.9dB B=22.9dB
...
summary: 1798 samples, 2 skipped, A changed 24 times, B one frame behind 22 times, out of step 0 samples (longest 0.0 ms, max 0.0 dB)
```

A sample is skipped while another run holds the bus, or when the leader's gain
changed and two reads of it disagree, since its driver writes the bytes under
its own hold.

The follower copies the leader's gain within the frame after the leader's
driver writes it, so a sample finds B at A's gain, or at the gain A held one
sample before. Both are in step: while A's loop converges and moves its gain
every frame, B trails it by one frame at every sample, and the summary counts
those samples as `B one frame behind`. A sample in which B holds neither gain
is out of step. The summary counts the out-of-step samples, the longest
stretch of them (from its first sample to the first in-step one, or to the end
of the run) and the widest gap among them in dB. A sample taken right after a
skipped slot (another run held the bus, or two reads of the leader disagreed)
that differs from its leader is not judged, since the leader's gain of the
missing frame is unknown; the summary counts those apart as `after skipped
slots not judged`. On a matched pair no
out-of-step stretch lasts longer than one frame period and the sampler's few
milliseconds of jitter, 33.3 ms at 30 fps.

B's capture session starts locked at the gain nxsd's heartbeat last named,
and the kernel driver writes that gain to B's head as the session starts. The
heartbeat is rewritten once a second. Where A's gain moved since, or no
heartbeat names a gain (the session then starts at 0 dB), B holds the start
gain until the follower's next copy: one out-of-step sample, one frame long.

Under a declared gain neither head moves, and the summary reads
`A changed 0 times, B one frame behind 0 times, out of step 0 samples (longest 0.0 ms, max 0.0 dB)`.
