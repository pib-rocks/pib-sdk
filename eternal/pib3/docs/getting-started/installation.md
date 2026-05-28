# Installation

## Requirements

- **Python**: 3.8+
- **pip**: Latest version recommended

## Quick Install

```bash
pip install -U "pib3 @ git+https://github.com/mamrehn/pib3.git"
```

---

## Linux

```bash
# Create project directory
mkdir ~/pib_project && cd ~/pib_project

# Create and activate virtual environment
python3 -m venv venv
source venv/bin/activate

# Install pib3
pip install "pib3 @ git+https://github.com/mamrehn/pib3.git"

# Verify
python -c "import pib3; print(f'pib3 {pib3.__version__}')"

# Deactivate when done
deactivate
```

## Windows

1. Install Python from [python.org](https://www.python.org/downloads/) - check **"Add Python to PATH"**
2. Open PowerShell:

```powershell
# Create project directory
mkdir $env:USERPROFILE\pib_project
cd $env:USERPROFILE\pib_project

# Create and activate virtual environment
python -m venv venv
.\venv\Scripts\Activate.ps1

# Install pib3
pip install "pib3 @ git+https://github.com/mamrehn/pib3.git"

# Verify
python -c "import pib3; print(f'pib3 {pib3.__version__}')"
```

!!! tip "PowerShell Script Error?"
    Run as Administrator: `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser`

---

## Installation Options

| Option | Dependencies | Use Case |
|--------|--------------|----------|
| `image` | scikit-image | Advanced image processing |

| `robot` | roslibpy | Real robot via rosbridge |
| `dev` | pytest, black, ruff, mypy | Development/testing |
| `all` | All above (except dev) | Full functionality |

```bash
# Install specific options
pip install "pib3[image,viz] @ git+https://github.com/mamrehn/pib3.git"
```

---

## Development Installation

```bash
git clone https://github.com/mamrehn/pib3.git
cd pib3
python3 -m venv venv
source venv/bin/activate  # Linux/macOS
pip install -e ".[all,dev]"
```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `pip: command not found` | Use `pip3` or `python3 -m pip install` |
| `No module named 'pib3'` | Activate venv: `source venv/bin/activate` |
| Permission errors (Linux) | Use venv or `pip install --user` |
| PowerShell script error | `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser` |
| SSL/Certificate errors | Add `--trusted-host pypi.org --trusted-host files.pythonhosted.org` |

| Robot connection fails | Install with `[robot]` option, check rosbridge running |

---

## Upgrading / Uninstalling

```bash
pip install --upgrade git+https://github.com/mamrehn/pib3.git
pip uninstall pib3
```

---

**Next:** [Quick Start](quickstart.md) | [Calibration](calibration.md)
