#!/usr/bin/env bash

set -euo pipefail

# ------------------------------------------------------------
# Sand Rake Vehicle - ROS 2 demo bag recorder
# ------------------------------------------------------------

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
BAG_ROOT="${WORKSPACE_DIR}/bags"

# Date and time, e.g. 20260815_171500
TIMESTAMP="$(date +'%Y%m%d_%H%M%S')"

# Git version. Fall back to "nogit" when unavailable.
if git -C "${WORKSPACE_DIR}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    GIT_VERSION="$(git -C "${WORKSPACE_DIR}" rev-parse --short HEAD)"
else
    GIT_VERSION="nogit"
fi

BAG_NAME="demo_${TIMESTAMP}_${GIT_VERSION}"
BAG_PATH="${BAG_ROOT}/${BAG_NAME}"

# ------------------------------------------------------------
# Candidate topics
#
# Only topics that currently exist will actually be recorded.
# This allows the script to work during staged integration.
# ------------------------------------------------------------

CANDIDATE_TOPICS=(
    "/teleop/cmd_vel"
    "/safety/cmd_vel"
    "/safety/event"

    "/front_camera/image_raw"
    "/front_camera/camera_info"
    "/rear_camera/image_raw"
    "/rear_camera/camera_info"

    "/vision/ball_detections"

    "/chassis/wheel_rpm_cmd"
    "/mock/wheel_rpm_feedback"
    "/mock/chassis_state"
    "/chassis/odom_raw"

    "/diagnostics"
)

mkdir -p "${BAG_ROOT}"

echo "============================================================"
echo " Sand Rake ROS 2 Demo Recorder"
echo "============================================================"
echo "Workspace : ${WORKSPACE_DIR}"
echo "Git       : ${GIT_VERSION}"
echo "Output    : ${BAG_PATH}"
echo

# ------------------------------------------------------------
# Find topics that currently exist
# ------------------------------------------------------------

mapfile -t AVAILABLE_TOPICS < <(ros2 topic list)

RECORD_TOPICS=()

echo "[INFO] Checking candidate topics..."

for topic in "${CANDIDATE_TOPICS[@]}"; do
    found=false

    for available_topic in "${AVAILABLE_TOPICS[@]}"; do
        if [[ "${topic}" == "${available_topic}" ]]; then
            found=true
            break
        fi
    done

    if [[ "${found}" == true ]]; then
        echo "[RECORD] ${topic}"
        RECORD_TOPICS+=("${topic}")
    else
        echo "[SKIP]   ${topic} (not currently available)"
    fi
done

echo

# ------------------------------------------------------------
# Do not start an empty rosbag
# ------------------------------------------------------------

if [[ "${#RECORD_TOPICS[@]}" -eq 0 ]]; then
    echo "[ERROR] No candidate topics are currently available."
    echo "[ERROR] rosbag recording will not start."
    exit 1
fi

echo "[INFO] Recording ${#RECORD_TOPICS[@]} topic(s)."
echo "[INFO] Press Ctrl+C to stop recording."
echo

# ------------------------------------------------------------
# Record
# ------------------------------------------------------------

ros2 bag record \
    -o "${BAG_PATH}" \
    "${RECORD_TOPICS[@]}"
