# Running Tests for Pegasus Simulator

## Overview

The Pegasus Simulator test suite is organized into:
- **Unit tests** (`tests/unit/`) - Pure Python tests that don't require Isaac Sim
- **Integration tests** (`tests/integration/`) - Tests that require Isaac Sim environment

## Running Tests with Isaac Sim's python.sh

Since our code depends on Isaac Sim modules (like `pxr`), we need to run pytest using Isaac Sim's Python environment.

### Basic Usage

```bash
# Navigate to the extension directory
cd /home/ubuntu/workspace/EmissarePegasusSimulator/extensions/pegasus.simulator

# Run all unit tests
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh -m pytest

# Or if pytest is not installed in Isaac Sim environment, install it first:
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh -m pip install pytest pytest-cov

# Then run tests
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh -m pytest
```

### Specific Test Commands

```bash
# Run only unit tests
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh -m pytest pegasus/simulator/tests/unit/

# Run specific test file
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh -m pytest pegasus/simulator/tests/unit/test_coordinate_transforms.py

# Run tests matching a pattern (e.g., yaw tests)
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh -m pytest -k "yaw"

# Run with verbose output
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh -m pytest -v

# Run with coverage report
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh -m pytest --cov=pegasus.simulator.logic.sensors.calculations --cov-report=term

# Test the coordinate transformation fix specifically
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh -m pytest pegasus/simulator/tests/unit/test_coordinate_transforms.py::TestYawOffsetFix -v
```

### Creating an Alias (Optional)

For convenience, you can add an alias to your shell configuration:

```bash
# Add to ~/.bashrc or ~/.zshrc
alias isaac-pytest="~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh -m pytest"

# Then use:
isaac-pytest pegasus/simulator/tests/unit/
```

## Test Structure

```
tests/
├── unit/                              # Tests without Isaac Sim dependencies
│   ├── test_coordinate_transforms.py  # Tests for coordinate system fixes
│   ├── test_imu_calculations.py       # IMU sensor calculations
│   └── test_gps_calculations.py       # GPS sensor calculations
├── integration/                       # Tests requiring Isaac Sim
│   └── test_hello_world.py           # Example Isaac Sim test
└── conftest.py                        # Pytest fixtures and configuration
```

## Key Tests for Coordinate Fix

The most important test for verifying the 90° yaw offset fix:

```bash
# This specific test verifies the fix works
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh -m pytest \
    pegasus/simulator/tests/unit/test_coordinate_transforms.py::TestYawOffsetFix::test_identity_quaternion_gives_zero_yaw -v
```

Expected output:
```
test_coordinate_transforms.py::TestYawOffsetFix::test_identity_quaternion_gives_zero_yaw PASSED
✓ Identity quaternion correctly gives yaw=0° (bug is fixed!)
```

## Troubleshooting

### ImportError: No module named 'pxr'
This means you're not using Isaac Sim's Python environment. Make sure to use `python.sh` instead of regular `python` or `pytest`.

### Permission Denied errors
Some directories in the Isaac Sim installation may have restricted permissions. The tests should only run on the unit test directory.

### Tests not discovered
Make sure you're in the correct directory:
```bash
cd /home/ubuntu/workspace/EmissarePegasusSimulator/extensions/pegasus.simulator
```

## Continuous Testing

During development, you can run tests in watch mode:

```bash
# Install pytest-watch first
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh -m pip install pytest-watch

# Run tests on file changes
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh -m ptw pegasus/simulator/tests/unit/
```