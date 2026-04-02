#!/bin/bash
# Install GeographicLib datasets from local GeographicLib/ directory

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$SCRIPT_DIR/GeographicLib"
DEST_DIR="/usr/share/GeographicLib"

if [[ $UID != 0 ]]; then
    echo "This script requires root privileges!" 1>&2
    exit 1
fi

if [[ ! -d "$SRC_DIR" ]]; then
    echo "Source directory not found: $SRC_DIR" 1>&2
    exit 1
fi

mkdir -p "$DEST_DIR"

for subdir in geoids gravity magnetic; do
    if [[ -d "$SRC_DIR/$subdir" ]]; then
        echo "Installing $subdir ..."
        cp -r "$SRC_DIR/$subdir" "$DEST_DIR/"
        echo "  -> $DEST_DIR/$subdir"
    else
        echo "Warning: $SRC_DIR/$subdir not found, skipping"
    fi
done

echo "Done."
