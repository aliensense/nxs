# NXS host bridge: decode samples over any wire, print pretty or JSON.
# One Dockerfile, slim per-transport variants via --build-arg EXTRA; the
# package vendors its own DSDL, so the build context is this directory.
# The cyphal image needs the vendored DSDL in the context (git-ignored):
#   scripts/vendor-dsdl.sh
#   docker build -t nxs:i2c .
#   docker build --build-arg EXTRA='[cyphal]' -t nxs:cyphal .
# Multi-arch (arm64 Jetson + amd64):
#   docker buildx build --platform linux/arm64,linux/amd64 \
#       --build-arg EXTRA='[cyphal]' -t <registry>/nxs:cyphal --push .
# Run:
#   docker run --rm --device /dev/i2c-2 nxs:i2c --transport i2c --bus /dev/i2c-2
#   docker run --rm --device /dev/ttyUSB0 nxs:cyphal --transport cyphal-serial \
#       --port /dev/ttyUSB0 --format json
# This image ships no ROS: pipe `--format json` into a ROS container, or
# pip-install nxs into a ROS image and run `nxs ros2` there.
FROM python:3.12-slim

# Empty for the I2C/serial image; '[cyphal]' pulls pycyphal + the DSDL
# toolchain. The i2c build apt-installs nothing.
ARG EXTRA=

WORKDIR /opt/nxs
COPY . .
# Install, then pre-compile the vendored DSDL so the first cyphal run is
# instant. No-op for the i2c image (EXTRA empty).
RUN pip install --no-cache-dir ".${EXTRA}" \
    && if [ -n "$EXTRA" ]; then \
        python -c "from nxs.transports.cyphal_source import _ensure_dsdl; _ensure_dsdl()"; \
    fi

# Bake the bundled-DSDL env for any in-container Cyphal tool; nxs auto-resolves it anyway.
ENV CYPHAL_PATH=/opt/nxs/nxs/dsdl \
    CYPHAL_ALLOW_UNREGULATED_FIXED_PORT_ID=1 \
    UAVCAN__NODE__ID=127

ENTRYPOINT ["python", "-m", "nxs.container"]
CMD ["--help"]
