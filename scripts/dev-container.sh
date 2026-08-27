#!/usr/bin/env bash
set -euo pipefail

root="$(git rev-parse --show-toplevel)"
host_uid="$(stat -c '%u' "$root")"
host_gid="$(stat -c '%g' "$root")"
compose=(
  docker compose
  -f "$root/.devcontainer/compose.yaml"
  --project-directory "$root/.devcontainer"
  --project-name rotator-dev
)

export DEV_UID="$host_uid"
export DEV_GID="$host_gid"

usage() {
  cat <<'EOF'
Usage: scripts/dev-container.sh COMMAND [ARGS...]

Commands:
  up              Build and start the persistent development container
  down            Stop the container (named caches and Claude state survive)
  shell           Open an ESP-IDF shell in the running container
  build           Build firmware and the LittleFS image with idf.py
  web             Run web tests and build the Svelte UI into main/data
  clean           Run idf.py fullclean inside the container
  size            Show ESP-IDF image and component sizes
  claude [ARGS]   Run Claude Code interactively in the container
  exec CMD...     Run an arbitrary command in the container
  status          Show container status
  config          Render and validate the effective Compose configuration
EOF
}

ensure_up() {
  "${compose[@]}" up -d --build
}

case "${1:-}" in
  up)
    ensure_up
    ;;
  down)
    "${compose[@]}" down
    ;;
  shell)
    ensure_up
    exec "${compose[@]}" exec dev bash
    ;;
  build)
    ensure_up
    "${compose[@]}" exec -T dev bash -lc '
      npm test
      npm run build
      if [[ -f sdkconfig && sdkconfig.defaults -nt sdkconfig ]]; then
        printf "sdkconfig.defaults changed; regenerating sdkconfig\n"
        rm -f sdkconfig sdkconfig.old
        idf.py fullclean
      fi
      idf.py build
    '
    ;;
  web)
    ensure_up
    "${compose[@]}" exec -T dev bash -lc 'npm test && npm run build'
    ;;
  clean)
    ensure_up
    "${compose[@]}" exec -T dev idf.py fullclean
    ;;
  size)
    ensure_up
    "${compose[@]}" exec -T dev idf.py size
    ;;
  claude)
    shift
    ensure_up
    exec "${compose[@]}" exec dev claude "$@"
    ;;
  exec)
    shift
    if (( $# == 0 )); then usage; exit 2; fi
    ensure_up
    exec "${compose[@]}" exec dev "$@"
    ;;
  status)
    "${compose[@]}" ps
    ;;
  config)
    "${compose[@]}" config
    ;;
  *)
    usage
    exit 2
    ;;
esac
