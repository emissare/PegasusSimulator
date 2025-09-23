#!/isaac-sim/python.sh
"""
Simple test runner for Pegasus Simulator tests.
Run this from inside the Isaac Sim shell using: /isaac-sim/python.sh test_runner.py

Usage:
    /isaac-sim/python.sh test_runner.py              # Run all tests
    /isaac-sim/python.sh test_runner.py coordinate   # Run coordinate transformation tests
    /isaac-sim/python.sh test_runner.py yaw_fix      # Test the 90° yaw offset fix specifically
    /isaac-sim/python.sh test_runner.py imu          # Run IMU tests
    /isaac-sim/python.sh test_runner.py gps          # Run GPS tests
"""

import sys
import unittest
import os


def print_header(title):
    """Print a nice header for test output."""
    print("\n" + "="*60)
    print(f"  {title}")
    print("="*60 + "\n")


def run_tests(test_type="all"):
    """Run specific tests using unittest."""

    # Initialize test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    try:
        if test_type == "yaw_fix":
            # THE critical test for the 90° yaw offset fix
            from pegasus.simulator.tests.unit.test_coordinate_transforms_kit import TestCoordinateTransforms
            suite.addTest(TestCoordinateTransforms('test_identity_quaternion_gives_zero_yaw'))

        elif test_type == "coordinate":
            # All coordinate transformation tests
            from pegasus.simulator.tests.unit import test_coordinate_transforms_kit
            suite.addTests(loader.loadTestsFromModule(test_coordinate_transforms_kit))

        elif test_type == "imu":
            # All IMU calculation tests
            from pegasus.simulator.tests.unit import test_imu_calculations_kit
            suite.addTests(loader.loadTestsFromModule(test_imu_calculations_kit))

        elif test_type == "gps":
            # All GPS calculation tests
            from pegasus.simulator.tests.unit import test_gps_calculations_kit
            suite.addTests(loader.loadTestsFromModule(test_gps_calculations_kit))

        elif test_type == "summary":
            # Only summary tests
            from pegasus.simulator.tests.unit.test_coordinate_transforms_kit import TestCoordinateTransforms
            from pegasus.simulator.tests.unit.test_imu_calculations_kit import TestIMUCalculations
            from pegasus.simulator.tests.unit.test_gps_calculations_kit import TestGPSCalculations

            suite.addTest(TestCoordinateTransforms('test_complete_transformation_summary'))
            suite.addTest(TestIMUCalculations('test_imu_summary'))
            suite.addTest(TestGPSCalculations('test_gps_summary'))

        elif test_type == "all":
            # Load all tests from unit directory
            from pegasus.simulator.tests.unit import test_coordinate_transforms_kit
            from pegasus.simulator.tests.unit import test_imu_calculations_kit
            from pegasus.simulator.tests.unit import test_gps_calculations_kit

            suite.addTests(loader.loadTestsFromModule(test_coordinate_transforms_kit))
            suite.addTests(loader.loadTestsFromModule(test_imu_calculations_kit))
            suite.addTests(loader.loadTestsFromModule(test_gps_calculations_kit))

        else:
            print(f"Unknown test type: {test_type}")
            return False

    except ImportError as e:
        print(f"Error importing test modules: {e}")
        print("Make sure you're in the correct directory and the modules exist.")
        return False

    # Run the tests
    if suite.countTestCases() == 0:
        print("No tests found to run!")
        return False

    runner = unittest.TextTestRunner(verbosity=2, stream=sys.stdout)
    result = runner.run(suite)

    return result.wasSuccessful()


def main():
    """Main test runner function."""

    # Get test type from command line
    test_type = sys.argv[1].lower() if len(sys.argv) > 1 else "all"

    # Define test configurations
    test_configs = {
        "all": "Running ALL Tests",
        "coordinate": "Running Coordinate Transformation Tests",
        "yaw_fix": "Testing 90° Yaw Offset Fix",
        "imu": "Running IMU Calculation Tests",
        "gps": "Running GPS Calculation Tests",
        "summary": "Running Summary Tests"
    }

    # Show help if requested
    if test_type in ["-h", "--help", "help"]:
        print(__doc__)
        print("\nAvailable test types:")
        for key, title in test_configs.items():
            print(f"  {key:12} - {title}")
        return 0

    # Check if test type is valid
    if test_type not in test_configs:
        print(f"Unknown test type: {test_type}")
        print("Available types:", ", ".join(test_configs.keys()))
        print("Run '/isaac-sim/python.sh test_runner.py help' for more information")
        return 1

    # Print header
    print_header(test_configs[test_type])

    # Special message for yaw_fix test
    if test_type == "yaw_fix":
        print("This is THE critical test that verifies the 90° yaw offset is fixed.")
        print("If this passes, the vehicle will point the correct direction!\n")

    # Run the tests
    success = run_tests(test_type)

    # Print result
    print("\n" + "="*60)
    if success:
        print("✓ TESTS PASSED!")
        if test_type == "yaw_fix":
            print("✓ The 90° yaw offset bug is FIXED!")
            print("✓ Vehicle pointing along X-axis now shows yaw=0° (North)")
    else:
        print("✗ TESTS FAILED!")
        if test_type == "yaw_fix":
            print("✗ The 90° yaw offset bug is NOT fixed")
    print("="*60)

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())