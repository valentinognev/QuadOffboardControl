#!/bin/bash
# Report the git version of a component directory to the CatSwarm GUI backend.
# Called at component launch time so each drone records the commit it actually started with.
# Fails silently — never blocks startup.

DRONE_ID=""
COMPONENT=""
DIR=""
GUI_URL="${CATSWARM_GUI_URL:-http://localhost:3001}"

for arg in "$@"; do
    case "$arg" in
        --drone-id=*) DRONE_ID="${arg#--drone-id=}" ;;
        --component=*) COMPONENT="${arg#--component=}" ;;
        --dir=*) DIR="${arg#--dir=}" ;;
        --gui-url=*) GUI_URL="${arg#--gui-url=}" ;;
    esac
done

if [[ -z "$DRONE_ID" || -z "$COMPONENT" || -z "$DIR" ]]; then
    exit 0
fi

if ! git -C "$DIR" rev-parse HEAD &>/dev/null; then
    exit 0
fi

COMMIT=$(git -C "$DIR" rev-parse --short=10 HEAD 2>/dev/null || echo "unknown")
BRANCH=$(git -C "$DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")

curl -s -X POST "${GUI_URL}/api/drone/${DRONE_ID}/version" \
    -H "Content-Type: application/json" \
    -d "{\"component\":\"${COMPONENT}\",\"commit\":\"${COMMIT}\",\"branch\":\"${BRANCH}\"}" \
    >/dev/null 2>&1 || true

exit 0
