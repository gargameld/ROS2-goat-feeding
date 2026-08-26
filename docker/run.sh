#!/usr/bin/env bash
# Start the goat-feeding container with whatever GPU this host actually has.
#
# The GPU reaches a container differently on each kind of host, so detect it
# here instead of keeping one set of flags that only works on one machine:
#   - WSL2:         /dev/dxg plus the host's D3D12 user-mode driver from
#                   /usr/lib/wsl (works for Intel, AMD and NVIDIA alike).
#   - NVIDIA:       --gpus all, via the nvidia container runtime.
#   - Native Linux: /dev/dri, plus the video/render groups so the device nodes
#                   are readable by the unprivileged desktop user.
# Driver selection *inside* the container is then handled by gl-autodetect.sh.
#
# Usage: docker/run.sh [extra docker run args...]
# Override with GOAT_IMAGE, GOAT_NAME or GOAT_WORKSPACE.
set -euo pipefail

IMAGE=${GOAT_IMAGE:-yotambar123/ros-goat-feeding:l1}
NAME=${GOAT_NAME:-goat}
WORKSPACE=${GOAT_WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/workspace}

args=(
    --name "$NAME"
    --detach
    --publish 3000:3000
    --publish 3001:3001
    --publish 2222:22
    --volume "$WORKSPACE:/config/workspace"
    # Fast DDS puts its shared-memory transport segments in /dev/shm, and a
    # single 640x480 XYZRGB PointCloud2 is ~10 MB. Docker's 64 MB default runs
    # out once a few camera topics are queued, at which point DDS silently
    # falls back to loopback UDP.
    --shm-size "${GOAT_SHM_SIZE:-2g}"
)

gpus=()

if [ -e /dev/dxg ] && [ -d /usr/lib/wsl ]; then
    gpus+=(WSL/D3D12)
    args+=(
        --device /dev/dxg
        --volume /usr/lib/wsl:/usr/lib/wsl:ro
        --env LD_LIBRARY_PATH=/usr/lib/wsl/lib
    )
fi

if docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"' &&
   { [ -e /dev/nvidiactl ] || command -v nvidia-smi >/dev/null 2>&1; }; then
    gpus+=(NVIDIA)
    args+=(--gpus all --env NVIDIA_DRIVER_CAPABILITIES=all)
fi

if [ -d /dev/dri ]; then
    gpus+=(DRI)
    args+=(--device /dev/dri)
    for group in video render; do
        gid=$(getent group "$group" | cut -d: -f3)
        [ -n "$gid" ] && args+=(--group-add "$gid")
    done
fi

if [ ${#gpus[@]} -eq 0 ]; then
    echo "No GPU interface found on this host; the container will render with llvmpipe." >&2
else
    echo "Exposing GPU interfaces: ${gpus[*]}" >&2
fi

exec docker run "${args[@]}" "$@" "$IMAGE"
