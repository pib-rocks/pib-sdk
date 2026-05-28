# Development Setup

Complete guide for setting up a development environment.

## Prerequisites

- Python 3.8 or higher
- Git
- A code editor (VS Code, PyCharm, etc.)

## Fork and Clone

```bash
# Fork the repository on GitHub, then clone your fork
git clone https://github.com/YOUR_USERNAME/pib3.git
cd pib3

# Add upstream remote
git remote add upstream https://github.com/mamrehn/pib3.git
```

## Virtual Environment

### Linux / macOS

```bash
python -m venv venv
source venv/bin/activate
```

### Windows

```cmd
python -m venv venv
venv\Scripts\activate
```

## Install Dependencies

```bash
# Install in editable mode with all dev dependencies
pip install -e ".[dev]"

# Or install specific extras
pip install -e ".[image,viz,robot,dev]"
```

## Verify Installation

```bash
# Check import works
python -c "import pib3; print(pib3.__version__)"

# Run tests
pytest

# Run with coverage
pytest --cov=pib3 --cov-report=html
```

## Project Structure

```
pib3/
├── pib3/              # Source code
│   ├── __init__.py      # Public API
│   ├── config.py        # Configuration classes
│   ├── types.py         # Core types
│   ├── image.py         # Image processing
│   ├── trajectory.py    # Trajectory generation
│   ├── backends/        # Robot backends
│   └── resources/       # Bundled assets (URDF, STL)
├── tests/               # Test files
├── docs/                # Documentation
├── pyproject.toml       # Package configuration
└── CLAUDE.md            # AI assistant instructions
```

## Development Workflow

### 1. Create a Branch

```bash
# Sync with upstream
git fetch upstream
git checkout main
git merge upstream/main

# Create feature branch
git checkout -b feature/your-feature-name
```

### 2. Make Changes

Edit code, add tests, update documentation as needed.

### 3. Run Tests

```bash
# Run all tests
pytest

# Run specific test file
pytest tests/test_trajectory.py

# Run with verbose output
pytest -v

# Run tests matching a pattern
pytest -k "test_sketch"
```

### 4. Check Code Quality

```bash
# Format code (if using black)
black pib3 tests

# Check types (if using mypy)
mypy pib3

# Lint (if using ruff)
ruff check pib3
```

### 5. Commit and Push

```bash
git add .
git commit -m "feat: Add your feature description"
git push origin feature/your-feature-name
```

### 6. Submit Pull Request

Go to GitHub and create a pull request from your branch.

## Running Documentation Locally

```bash
# Install docs dependencies
pip install mkdocs-material mkdocstrings[python]

# Serve documentation locally
mkdocs serve

# Open http://localhost:8000 in browser
```

## Testing with Hardware



### Webots Simulator

1. Install [Webots](https://cyberbotics.com/)
2. Open a PIB world file
3. Set controller to your test script

### Real Robot

```bash
# Test connection (requires robot on network)
python -c "
from pib3 import Robot
robot = Robot(host='172.26.34.149', timeout=5.0)
try:
    robot.connect()
    print('Connected!')
    robot.disconnect()
except Exception as e:
    print(f'Connection failed: {e}')
"
```

## Common Tasks

### Adding a New Feature

1. Create feature branch
2. Write tests first (TDD encouraged)
3. Implement feature
4. Add documentation
5. Submit PR

### Fixing a Bug

1. Write a failing test that reproduces the bug
2. Fix the bug
3. Verify test passes
4. Submit PR with test and fix

### Updating Documentation

```bash
# Edit docs/*.md files
# Preview changes
mkdocs serve

# Documentation is automatically deployed on merge to main
```

### Regenerating URDF

If you modify the Webots proto file:

```bash
python -m pib3.tools.proto_converter
```

## Environment Variables

| Variable | Purpose | Default |
|----------|---------|---------|
| `PIB_ROBOT_HOST` | Default robot IP | 172.26.34.149 |
| `PIB_ROBOT_PORT` | Default rosbridge port | 9090 |
| `PIB_DEBUG` | Enable debug logging | false |

## Troubleshooting

### Import Errors

```bash
# Ensure you're in the virtual environment
which python  # Should show venv path

# Reinstall in editable mode
pip install -e ".[dev]"
```

### Test Failures

```bash
# Run with verbose output
pytest -v --tb=long

# Run single failing test
pytest tests/test_file.py::test_function -v
```

### Documentation Build Errors

```bash
# Check mkdocs config
mkdocs build --strict

# Common issue: missing docstring
# Solution: Add docstrings to all public functions
```
