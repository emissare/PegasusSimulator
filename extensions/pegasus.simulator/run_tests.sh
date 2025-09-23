#!/bin/bash
# Test runner script for Pegasus Simulator

echo "=================================="
echo "Pegasus Simulator Test Suite"
echo "=================================="
echo ""

# Change to the extension directory
cd "$(dirname "$0")"

# Different test commands
case "${1:-all}" in
    unit)
        echo "Running unit tests only..."
        pytest pegasus/simulator/tests/unit/ -v
        ;;

    integration)
        echo "Running integration tests (requires Isaac Sim)..."
        pytest pegasus/simulator/tests/integration/ -v
        ;;

    coordinate)
        echo "Running coordinate transformation tests..."
        pytest pegasus/simulator/tests/unit/test_coordinate_transforms.py -v -k "test_yaw"
        ;;

    coverage)
        echo "Running tests with coverage..."
        pytest --cov=pegasus.simulator.logic.sensors.calculations \
               --cov-report=html \
               --cov-report=term \
               pegasus/simulator/tests/unit/
        echo "Coverage report generated in htmlcov/index.html"
        ;;

    quick)
        echo "Running quick smoke tests..."
        pytest pegasus/simulator/tests/unit/ -v -k "not slow"
        ;;

    fix)
        echo "Testing the coordinate transformation fix..."
        pytest pegasus/simulator/tests/unit/ -v -k "yaw_offset or identity_quaternion"
        ;;

    all)
        echo "Running all unit tests..."
        pytest pegasus/simulator/tests/unit/ -v

        echo ""
        echo "Running coverage analysis..."
        pytest --cov=pegasus.simulator.logic.sensors.calculations \
               --cov-report=term-missing \
               pegasus/simulator/tests/unit/
        ;;

    *)
        echo "Usage: $0 [unit|integration|coordinate|coverage|quick|fix|all]"
        echo ""
        echo "  unit        - Run unit tests only"
        echo "  integration - Run integration tests (requires Isaac Sim)"
        echo "  coordinate  - Run coordinate transformation tests"
        echo "  coverage    - Run tests with coverage report"
        echo "  quick       - Run quick smoke tests"
        echo "  fix         - Test the 90° yaw offset fix"
        echo "  all         - Run all unit tests with coverage (default)"
        exit 1
        ;;
esac

# Check test result
if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Tests passed!"
else
    echo ""
    echo "✗ Tests failed!"
    exit 1
fi