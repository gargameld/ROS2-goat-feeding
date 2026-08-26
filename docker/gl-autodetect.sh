# Pick the best available OpenGL driver for this host, once per container start.
#
# Sourced from /etc/bash.bashrc, so every interactive shell (and anything
# launched from one, e.g. `ros2 launch ... mujoco`) inherits the result.
#
# The image is the same everywhere, but the GPU underneath it is not:
#   - NVIDIA via the nvidia container runtime -> libglvnd picks libGLX_nvidia
#     on its own, and environment overrides only get in the way.
#   - Native Linux with /dev/dri passed in    -> Mesa picks i915/radeonsi itself.
#   - WSL2 with /dev/dxg + /usr/lib/wsl       -> needs GALLIUM_DRIVER=d3d12 and
#     libd3d12core.so on the library path, or Mesa silently falls back to the
#     llvmpipe software rasteriser.
# Rather than hardcoding one of those, probe glxinfo and keep the first
# candidate that reports a non-software renderer.
#
# Set GL_AUTODETECT=0 before starting a shell to skip this entirely.

_gl_autodetect() {
    [ "${GL_AUTODETECT:-1}" = "0" ] && return 0

    local cache="/tmp/.gl-env-$(id -u).sh"
    if [ -r "$cache" ]; then
        . "$cache"
        return 0
    fi

    # Probing needs a live X display and glxinfo. If either is missing this is
    # not the desktop session, so leave the environment alone and retry in the
    # next shell instead of caching a bogus answer.
    [ -n "${DISPLAY:-}" ] || return 0
    command -v glxinfo >/dev/null 2>&1 || return 0

    # Some WSL/X11 guides export these; either one forces software rendering no
    # matter which driver we select below, so clear them first.
    unset LIBGL_ALWAYS_INDIRECT
    [ "${LIBGL_ALWAYS_SOFTWARE:-0}" = "0" ] || unset LIBGL_ALWAYS_SOFTWARE

    local candidate renderer
    local -a overrides
    # Order matters: on a hybrid laptop the Mesa default is the integrated GPU,
    # which is hardware and would win, so offer the discrete NVIDIA card first.
    for candidate in nvidia-prime mesa-default wsl-d3d12; do
        case "$candidate" in
            nvidia-prime)
                # Only when the NVIDIA container runtime actually handed us the
                # device nodes and the GLX vendor library that goes with them;
                # a compute-only (--gpus) setup has no libGLX_nvidia.
                [ -e /dev/nvidiactl ] || continue
                ldconfig -p 2>/dev/null | grep -q libGLX_nvidia || continue
                overrides=(
                    __NV_PRIME_RENDER_OFFLOAD=1
                    __GLX_VENDOR_LIBRARY_NAME=nvidia
                )
                ;;
            mesa-default)
                overrides=()
                ;;
            wsl-d3d12)
                # Only meaningful on WSL, where the GPU arrives as /dev/dxg and
                # the D3D12 user-mode driver is bind-mounted from the host.
                [ -e /dev/dxg ] && [ -d /usr/lib/wsl/lib ] || continue
                overrides=(GALLIUM_DRIVER=d3d12)
                # The launcher may already put the host's WSL libraries on the
                # path; only prepend them when it did not.
                case ":${LD_LIBRARY_PATH:-}:" in
                    *:/usr/lib/wsl/lib:*) ;;
                    *) overrides+=("LD_LIBRARY_PATH=/usr/lib/wsl/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}") ;;
                esac
                ;;
        esac

        renderer=$(env "${overrides[@]}" timeout 20 glxinfo -B 2>/dev/null |
            sed -n 's/^OpenGL renderer string: //p')

        case "$renderer" in
            ""|*llvmpipe*|*softpipe*|*swrast*|*"Software Rasterizer"*) continue ;;
        esac

        {
            echo "# gl-autodetect: $candidate -> $renderer"
            local assignment
            for assignment in "${overrides[@]}"; do
                echo "export ${assignment%%=*}='${assignment#*=}'"
            done
        } > "$cache"
        . "$cache"
        return 0
    done

    echo "# gl-autodetect: no hardware renderer available, using llvmpipe" > "$cache"
    return 0
}

_gl_autodetect
unset -f _gl_autodetect
