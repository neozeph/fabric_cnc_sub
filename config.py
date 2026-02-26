#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Configuration management for the fabric CNC system.
Provides motor configurations, work area settings, and motion parameters.
All configuration is now consolidated in this single file.
"""

import logging
import os
import platform
from dataclasses import dataclass
from typing import Dict

logger = logging.getLogger(__name__)

@dataclass
class WorkArea:
    """Work area dimensions in inches."""
    x: float
    y: float

@dataclass
class MotionConfig:
    """Motion control configuration."""
    default_speed_inch_s: float
    default_accel_inch_s2: float
    lift_height_inch: float

class Config:
    """Configuration manager for the fabric CNC system."""
    
    def __init__(self):
        """Initialize configuration with all values directly in code."""
        # GPIO Pin Configuration
        self.gpio_pins = {
            'X': {'DIR': 1, 'STEP': 0, 'EN': 8},  # Pico GP0/1
            'Y': {'DIR': 3, 'STEP': 2, 'EN': 8},  # Pico GP2/3
            'Z_LIFT': {'DIR': 5, 'STEP': 4, 'EN': 8},  # Pico GP4/5
            'A': {'DIR': 7, 'STEP': 6, 'EN': 8},  # Pico GP6/7
        }
        
        # Motor Steps Configuration (steps per inch)
        self.steps_per_inch = {
            'X': 2032,  # 80 steps/mm * 25.4 mm/inch
            'Y': 2032,  # 80 steps/mm * 25.4 mm/inch
            'Z_LIFT': 10160,  # 400 steps/mm * 25.4 mm/inch
            'A': 254,  # 10 steps/mm * 25.4 mm/inch
        }
        
        # Motor Direction Configuration
        self.direction_inverted = {
            'X': True,  # Invert X direction to fix flipped axis
            'Y': True,  # Invert Y direction
            'Z_LIFT': False,
            'A': True,  # Invert direction to fix one-way rotation issue
        }
        
        # Work Area Configuration
        self.work_area = WorkArea(x=30, y=30)  # 30" x 30" work area (matches APP_CONFIG)
        
        # Motion Configuration
        self.motion = MotionConfig(
            default_speed_inch_s=0.787,  # 20 mm/s = 0.787 inches/s
            default_accel_inch_s2=3.937,  # 100 mm/s² = 3.937 inches/s²
            lift_height_inch=1.0  # 25.4mm = 1.0 inches
        )
        
        # System Configuration
        self.simulation_mode = self._get_bool_env('FABRIC_CNC_SIMULATION', False)
        self.step_pulse_duration = float(
            os.getenv('FABRIC_CNC_STEP_PULSE_DURATION', '0.001')
        )
        
        # Sensor debounce configuration (in milliseconds)
        self.sensor_debounce_times = {
            'X': 15,      # 15ms for X sensor
            'Y': 10,      # 10ms for Y sensor
            'Z_LIFT': 40, # 40ms for Z sensor
            'A': 25 # 25ms for A sensor
        }
        
        # Sensor reading count for multi-reading debounce
        self.sensor_reading_count = 2
        
        self._validate_config()
        
    def _get_bool_env(self, name: str, default: bool) -> bool:
        """Get boolean value from environment variable."""
        value = os.getenv(name)
        if value is None:
            return default
        return value.lower() in ('true', '1', 'yes')
        
    def _validate_config(self) -> None:
        """Validate configuration values."""
        # Check that all motors have required pins
        required_pins = {'DIR', 'STEP', 'EN'}
        for motor, pins in self.gpio_pins.items():
            missing = required_pins - set(pins.keys())
            if missing:
                raise ValueError(
                    f"Motor {motor} missing required pins: {missing}"
                )
                
        # Check that all motors have steps_per_inch defined
        for motor in self.gpio_pins:
            if motor not in self.steps_per_inch:
                raise ValueError(
                    f"Motor {motor} missing steps_per_inch configuration"
                )
                
        # Check that all motors have direction_inverted defined
        for motor in self.gpio_pins:
            if motor not in self.direction_inverted:
                raise ValueError(
                    f"Motor {motor} missing direction_inverted configuration"
                )
                
        # Validate work area dimensions
        if self.work_area.x <= 0 or self.work_area.y <= 0:
            raise ValueError("Work area dimensions must be positive")
            
        # Validate motion parameters
        if self.motion.default_speed_inch_s <= 0:
            raise ValueError("Default speed must be positive")
        if self.motion.default_accel_inch_s2 <= 0:
            raise ValueError("Default acceleration must be positive")
        if self.motion.lift_height_inch <= 0:
            raise ValueError("Lift height must be positive")
            
        logger.info("Configuration validation successful")

# System detection
ON_RPI = platform.system() == 'Linux' and (os.uname().machine.startswith('arm') or os.uname().machine.startswith('aarch'))

# Create global configuration instance
config = Config()

# Export commonly used values for backward compatibility
GPIO_PINS = config.gpio_pins
STEPS_PER_INCH = config.steps_per_inch
DIRECTION_INVERTED = config.direction_inverted
WORKAREA_INCH = {'X': config.work_area.x, 'Y': config.work_area.y}
DEFAULT_SPEED_INCH_S = config.motion.default_speed_inch_s
DEFAULT_ACCEL_INCH_S2 = config.motion.default_accel_inch_s2
LIFT_HEIGHT_INCH = config.motion.lift_height_inch
USE_SIMULATION_MODE = config.simulation_mode
STEP_PULSE_DURATION = config.step_pulse_duration

# Simulation mode detection
SIMULATION_MODE = not ON_RPI or config.simulation_mode

# Motor configuration - All motor parameters consolidated here
MOTOR_CONFIG = {
    'X': {
        'PULSES_PER_REV': 800,  # DIP switches set for 800 steps per revolution
        'INCH_PER_REV': 0.787,  # 20mm per revolution = 0.787 inches per revolution
        'STEP_DELAY': 0.00025,  # 0.25ms between pulses = 2000 steps/sec
        'STEP': 0,   # GP0
        'DIR': 1,    # GP1
        'EN': 8,     # GP8
        'HOME_DIRECTION': 1,  # Positive direction for homing
        'HOME_SPEED': 0.0005,  # Original homing speed (0.5ms between pulses)
        'VERIFY_SPEED': 0.002  # Original verification speed (2ms between pulses)
    },
    'Y': {
        'PULSES_PER_REV': 800,
        'INCH_PER_REV': 0.787,  # 20mm per revolution = 0.787 inches per revolution
        'STEP_DELAY': 0.00025,  # 0.25ms between pulses = 2000 steps/sec
        'STEP': 2,   # GP2
        'DIR': 3,    # GP3
        'EN': 8,     # GP8
        'HOME_DIRECTION': 1,
        'HOME_SPEED': 0.0005,  # Original homing speed
        'VERIFY_SPEED': 0.002  # Original verification speed
    },
    'Z_LIFT': {
        'PULSES_PER_REV': 800,
        'INCH_PER_REV': 0.197,  # 5mm per revolution = 0.197 inches per revolution
        'STEP_DELAY': 0.00025,  # 0.25ms between pulses = 2000 steps/sec
        'STEP': 4,   # GP4
        'DIR': 5,    # GP5
        'EN': 8,     # GP8
        'HOME_DIRECTION': -1,
        'HOME_SPEED': 0.001,  # Original homing speed
        'VERIFY_SPEED': 0.004  # Original verification speed
    },
    'A': {
        'PULSES_PER_REV': 3200,  # TB6600 driver setting (3200 pulses/rev)
        'INCH_PER_REV': 360,  # 360 degrees per revolution
        'STEP_DELAY': 0.00025,  # 0.25ms between pulses = 2000 steps/sec
        'STEP': 6,   # GP6
        'DIR': 7,    # GP7
        'EN': 8,     # GP8
        'HOME_DIRECTION': -1,
        'HOME_SPEED': 0.001,  # Original homing speed
        'VERIFY_SPEED': 0.004  # Original verification speed
    }
}

# Machine configuration - All machine limits and parameters
MACHINE_CONFIG = {
    'MAX_X': 30,  # Maximum X travel in inches (matches work area)
    'MAX_Y': 30,  # Maximum Y travel in inches (matches work area)
    'HOMING_OFFSET': 0.225,  # Distance to move after hitting home sensor (5.715mm = 0.225 inches)
    'VERIFICATION_DISTANCE': 0.394  # Distance to move for verification (10mm = 0.394 inches)
}



# Application configuration
APP_CONFIG = {
    'X_MAX_INCH': 30,  # 30 inches
    'Y_MAX_INCH': 30,  # 30 inches
    'Z_MAX_INCH': 2.5,  # 2.5 inches
    'Z_UP_INCH': -0.75,  # -0.75 inches
    'Z_DOWN_INCH': -0.75,  # Same as hover height for testing
    'PLOT_BUFFER_IN': 1.0,
    'ANGLE_CHANGE_THRESHOLD_DEG': 2.0,
    'STEP_SIZE_INCHES': 0.05,
    'ARROW_KEY_REPEAT_DELAY': 100,
    'JOG_SLIDER_SCALE': 0.1,
    'CANVAS_WIDTH': 800,
    'CANVAS_HEIGHT': 600,
    'CANVAS_SCALE': 1.0,
    'TOOL_HEAD_RADIUS': 10,
    'LIVE_TOOL_HEAD_RADIUS': 7,
    'LIVE_TOOL_HEAD_DIR_RADIUS': 0.5,
    'ANIMATION_STEPS_PER_TICK': 1,
    'ANIMATION_TOOL_RADIUS': 0.5,
    'PLOT_BUFFER_PX': 50,  # Buffer around the plot in pixels
}

# UI Color scheme
UI_COLORS = {
    'PRIMARY_COLOR': '#800000',  # Maroon 800
    'PRIMARY_VARIANT': '#660000',  # Dark Maroon 900
    'SECONDARY_COLOR': '#A52A2A',  # Brown (lighter maroon variant)
    'BACKGROUND': '#F5F5F5',
    'SURFACE': '#F5F5F5',
    'ON_PRIMARY': '#ffffff',
    'ON_SURFACE': '#222222',
    'ERROR_COLOR': '#b00020',
    # Modern button styling
    'BUTTON_PRIMARY': '#800000',  # Maroon
    'BUTTON_PRIMARY_HOVER': '#660000',  # Darker maroon on hover
    'BUTTON_SECONDARY': '#6B7280',  # Modern gray
    'BUTTON_SECONDARY_HOVER': '#4B5563',  # Darker gray on hover
    'BUTTON_SUCCESS': '#10B981',  # Modern green
    'BUTTON_SUCCESS_HOVER': '#059669',  # Darker green on hover
    'BUTTON_WARNING': '#F59E0B',  # Modern orange
    'BUTTON_WARNING_HOVER': '#D97706',  # Darker orange on hover
    'BUTTON_DANGER': '#EF4444',  # Modern red
    'BUTTON_DANGER_HOVER': '#DC2626',  # Darker red on hover
    'BUTTON_TEXT': '#FFFFFF',  # White text on buttons
    'BUTTON_SHADOW': '#E5E7EB',  # Light shadow color
}

# UI Padding constants for consistent spacing
UI_PADDING = {
    'SMALL': 12,
    'MEDIUM': 16,
    'LARGE': 20,
    'XLARGE': 24,
    'XXLARGE': 32,
    'SECTION_SPACING': 20,
    'BUTTON_SPACING': 10,
    'FRAME_PADDING': 20,
    'CANVAS_PADDING': 24
}

# Toolpath configuration
TOOLPATH_CONFIG = {
    'DEFAULT_FEED_RATE': 100,
    'DEFAULT_Z_UP': 5,
    'DEFAULT_Z_DOWN': -1,
    'DEFAULT_STEP_SIZE': 0.05,
    'SPLINE_FLATTENING_PRECISION': 0.005,
    'CIRCLE_STEPS_MIN': 32,
    'CIRCLE_STEPS_MAX': 256,
    'SPLINE_STEPS_MIN': 64,
    'SPLINE_STEPS_MAX': 512,
    'MAX_ANGLE_STEP_RADIANS': 0.026179,  # 1.5 degrees
} 