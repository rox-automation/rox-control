#!/bin/bash
set -e

cd /workspace
echo "Running CI pipeline..."

echo "Step 1: Running linting checks..."
uv run invoke lint

echo "Step 2: Running tests..."
uv run invoke test

echo "CI pipeline completed successfully!"