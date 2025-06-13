#!/bin/bash
echo "Running init_container.sh, use this script to install libraries etc."

set -e

# Export uv dependencies to requirements.txt without hashes
uv export --all-extras --format requirements-txt --no-hashes --output-file /tmp/requirements.txt

# Install dependencies using pip (directly to system Python)
pip install -r /tmp/requirements.txt
