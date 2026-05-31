#!/usr/bin/env bash
# Build the image on any NVIDIA AGX Orin (JetPack 6.x).
# Usage: ./build.sh [image-tag]
set -euo pipefail
IMAGE="${1:-misys:3d_xvn_rebuilt}"
echo ">> Building $IMAGE (this pulls a multi-GB base image and takes a while)..."
docker build -t "$IMAGE" .
echo ""
echo ">> Done. Skipped packages (if any):"
docker run --rm "$IMAGE" sh -c 'cat /tmp/apt_skipped.log /tmp/pip_skipped.log 2>/dev/null || echo none'
echo ">> Run it with:  ./run.sh $IMAGE"
