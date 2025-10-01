#!/usr/bin/env bash
set -euo pipefail

# ---------- argument parser -------------------------------------------------
while [[ $# -gt 0 ]]; do
  case $1 in
    --device)   DEVICE="$2";   shift 2 ;;
    --width)    WIDTH="$2";    shift 2 ;;
    --height)   HEIGHT="$2";   shift 2 ;;
    --bitrate)  BITRATE="$2";  shift 2 ;;
    --host)     HOST="$2";     shift 2 ;;
    --port)     PORT="$2";     shift 2 ;;
    --flip)     FLIP="$2";     shift 2 ;;
    *) echo "Unknown option $1"; exit 1 ;;
  esac
done

# ---------- fall back to environment variables if any flag is missing -------
: "${DEVICE:=${CAM_DEVICE:-/dev/video0}}"
: "${WIDTH:=${CAM_WIDTH:-1280}}"
: "${HEIGHT:=${CAM_HEIGHT:-720}}"
: "${BITRATE:=${CAM_BITRATE:-300000}}"
: "${HOST:=${CAM_HOST:-192.168.1.6}}"
: "${PORT:=${CAM_PORT:-5000}}"
: "${FLIP:=${CAM_FLIP:-2}}"

echo "▶️  Starting stream $DEVICE  ${WIDTH}x${HEIGHT}  ${BITRATE} bps → $HOST:$PORT"

exec gst-launch-1.0 -e \
  v4l2src device="$DEVICE" do-timestamp=true ! \
  video/x-raw,width="$WIDTH",height="$HEIGHT" ! \
  queue leaky=downstream ! \
  watchdog timeout=2000000000 name=wd ! \
  nvvidconv flip-method="$FLIP" ! \
  'video/x-raw(memory:NVMM),format=NV12' ! \
  nvv4l2h265enc bitrate="$BITRATE" ! \
  rtph265pay pt=96 config-interval=1 ! \
  udpsink host="$HOST" port="$PORT" sync=false
