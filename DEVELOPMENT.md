# Development

### Local Development
```bash
# Create virtual environment
invoke create-venv

# Run tests
invoke test

# Lint and format code
invoke lint


```

### VS Code DevContainer
Open in VS Code and select "Reopen in Container" for a pre-configured development environment with all tools and extensions.

## CI/CD

* **Local CI**: Run `invoke ci` to test in Docker container

## Tooling

* **Dependencies**: `uv` for fast package management
* **Automation**: `invoke` - run `invoke -l` to list available commands
* **Versioning**: `setuptools_scm` with git tags
* **Linting**: `ruff` for fast linting and formatting
* **Type checking**: `mypy` for static analysis

## Project Structure

* `src/rox_control/` - Application code
* `tasks.py` - Automation tasks via invoke
* `pyproject.toml` - Modern Python packaging configuration
