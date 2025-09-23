"""
Pure barometer calculation functions.
No dependencies on Isaac Sim or State class.
"""

import numpy as np
from typing import Dict, Optional, Tuple


def calculate_barometer_measurements(
    position_z_flu: float,
    origin_alt: float,
    temperature: float = 20.0,
    noise_params: Optional[Dict] = None
) -> Dict[str, float]:
    """
    Calculate barometer measurements from vehicle state.

    Args:
        position_z_flu: Z position in FLU world frame (meters)
        origin_alt: Altitude of world origin (meters)
        temperature: Temperature in Celsius (default 20°C)
        noise_params: Optional noise parameters

    Returns:
        Dictionary containing pressure and temperature
    """

    # Calculate altitude above sea level
    altitude_asl = position_z_flu + origin_alt

    # Add noise if provided
    if noise_params and 'altitude_noise' in noise_params:
        altitude_asl += noise_params['altitude_noise']

    # Calculate pressure using barometric formula
    pressure = calculate_pressure_from_altitude(altitude_asl, temperature)

    # Add noise to pressure if provided
    if noise_params and 'pressure_noise' in noise_params:
        pressure += noise_params['pressure_noise']

    # Add noise to temperature if provided
    if noise_params and 'temperature_noise' in noise_params:
        temperature += noise_params['temperature_noise']

    return {
        "pressure": pressure,
        "temperature": temperature,
        "altitude": altitude_asl
    }


def calculate_pressure_from_altitude(
    altitude: float,
    temperature: float = 20.0
) -> float:
    """
    Calculate atmospheric pressure from altitude using barometric formula.

    Args:
        altitude: Altitude above sea level in meters
        temperature: Temperature in Celsius

    Returns:
        Pressure in Pascals
    """
    # Constants
    P0 = 101325.0  # Sea level standard atmospheric pressure (Pa)
    L = 0.0065  # Temperature lapse rate (K/m)
    T0 = 288.15  # Sea level standard temperature (K)
    g = 9.80665  # Gravitational acceleration (m/s^2)
    M = 0.0289644  # Molar mass of dry air (kg/mol)
    R = 8.31432  # Universal gas constant (N·m/(mol·K))

    # Convert temperature to Kelvin
    T_kelvin = temperature + 273.15

    # Calculate pressure using barometric formula
    # P = P0 * (1 - L*h/T0)^(g*M/(R*L))
    if altitude < 11000:  # Troposphere
        pressure = P0 * np.power(1.0 - L * altitude / T0, g * M / (R * L))
    else:  # Simplified for stratosphere
        # At 11km, pressure is about 22632 Pa
        P11 = 22632.0
        T11 = T0 - L * 11000
        # Isothermal layer
        pressure = P11 * np.exp(-g * M * (altitude - 11000) / (R * T11))

    return pressure


def calculate_barometer_noise(
    dt: float,
    pressure_noise_density: float = 0.01,  # Pa/sqrt(Hz)
    altitude_noise_density: float = 0.1,   # m/sqrt(Hz)
    temperature_drift: float = 0.01        # °C/sqrt(Hz)
) -> Dict[str, float]:
    """
    Calculate barometer noise.

    Args:
        dt: Time step in seconds
        pressure_noise_density: Pressure noise density
        altitude_noise_density: Altitude noise density
        temperature_drift: Temperature drift rate

    Returns:
        Dictionary containing noise values
    """

    # Generate white noise scaled by sqrt(dt)
    sqrt_dt = np.sqrt(dt)

    return {
        "pressure_noise": pressure_noise_density * sqrt_dt * np.random.randn(),
        "altitude_noise": altitude_noise_density * sqrt_dt * np.random.randn(),
        "temperature_noise": temperature_drift * sqrt_dt * np.random.randn()
    }