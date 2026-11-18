#!/usr/bin/env bash
# Prepare a pool-test clip for publication review: trim, drop audio, discard all source
# container/stream metadata (device, location, timestamps) and remove the encoder-settings string embedded in the stream.
# Only generic muxer labels remain (e.g. encoder "Lavc libx264", handler "VideoHandler"); they are not
# personal data. Does not blur anything - review the output for people and location identifiers.
# Usage: tools/prepare_clip.sh INPUT.mp4 OUTPUT.mp4 [START_SECONDS] [DURATION_SECONDS] [POSTER_SECONDS]
set -euo pipefail
in=${1:?input}; out=${2:?output}; ss=${3:-0}; dur=${4:-10}; poster=${5:-2.6}
ffmpeg -v error -y -ss "$ss" -t "$dur" -i "$in" \
  -an -map 0:v:0 -map_metadata -1 -map_chapters -1 \
  -vf "fps=30" -c:v libx264 -preset slow -crf 22 -pix_fmt yuv420p \
  -fflags +bitexact -flags:v +bitexact -movflags +faststart "$out.tmp.mp4"
# remove the encoder-settings SEI (NAL type 6) that libx264 embeds in the stream
ffmpeg -v error -y -i "$out.tmp.mp4" -c copy -bsf:v filter_units=remove_types=6 -map_metadata -1 \
  -fflags +bitexact -movflags +faststart "$out"
rm -f "$out.tmp.mp4"
# poster still (PNG, no metadata)
ffmpeg -v error -y -ss "$poster" -i "$out" -frames:v 1 -map_metadata -1 "${out%.*}_poster.png"
echo "wrote $out and ${out%.*}_poster.png"
