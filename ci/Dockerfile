FROM python:3.11-slim

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install uv
RUN pip install uv

# Set working directory to /app for setup
WORKDIR /app

# Copy dependency files first (for better caching)
COPY pyproject.toml uv.lock ./

# Install dependencies in /app
RUN uv sync --all-extras

# Set the working directory where code will be mounted
WORKDIR /workspace

# Default command - run CI script
CMD ["./ci/scripts/run-ci.sh"]