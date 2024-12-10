#!/usr/bin/env python3
"""Test script for DexHand with steady-state error analysis and feedback tracking."""

import numpy as np
import time
import logging
import argparse
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import matplotlib.pyplot as plt
import pandas as pd

from pyzlg_dexhand.dexhand_interface import (
    LeftDexHand,
    RightDexHand,
    ControlMode,
    ZCANWrapper,
    HandFeedback,
)
from pyzlg_dexhand.dexhand_logger import DexHandLogger

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class TestResult:
    """Store test metrics for a joint movement"""

    timestamp: float
    command_angle: float
    actual_angle: float
    steady_state_angle: float  # Average angle after settling
    steady_state_error: float  # Error after settling
    settling_time: float  # Time to reach steady state
    error_msg: Optional[str] = None
    command_time: Optional[float] = None


@dataclass
class JointConfig:
    """Configuration for a joint including limits"""

    min_angle: float = 0.0
    max_angle: float = 90.0  # Default for most joints


JOINT_CONFIGS = {
    "th_rot": JointConfig(max_angle=150.0),  # Thumb rotation has extended range
    "th_mcp": JointConfig(),  # Default 0-90
    "th_dip": JointConfig(),
    "ff_spr": JointConfig(max_angle=30.0),  # Finger spread is limited
    "ff_mcp": JointConfig(),
    "ff_dip": JointConfig(),
    "mf_mcp": JointConfig(),
    "mf_dip": JointConfig(),
    "rf_mcp": JointConfig(),
    "rf_dip": JointConfig(),
    "lf_mcp": JointConfig(),
    "lf_dip": JointConfig(),
}


class DexHandTester:
    """Test sequence runner for dexterous hands with steady-state analysis"""

    def __init__(
        self,
        hand_names: List[str],
        log_dir: str = "dexhand_logs",
        settling_time: float = 1.0,
        n_samples: int = 5,
    ):
        """Initialize test runner

        Args:
            hand_names: List of hands to test ('left', 'right')
            log_dir: Directory for test logs
            settling_time: Time to wait for settling (seconds)
            n_samples: Number of samples to collect for steady state
        """
        self.settling_time = settling_time
        self.n_samples = n_samples

        # Validate hand selection
        valid_hands = {"left", "right"}
        if not all(hand in valid_hands for hand in hand_names):
            raise ValueError(f"Invalid hand selection. Must be 'left' or 'right'")

        # Create ZCAN interface
        self.zcan = ZCANWrapper() if len(hand_names) > 1 else None

        # Initialize hands
        self.hands = {}
        for name in hand_names:
            hand_class = LeftDexHand if name == "left" else RightDexHand
            self.hands[name] = hand_class(self.zcan)
            if not self.hands[name].init():
                raise RuntimeError(f"Failed to initialize {name} hand")
            logger.info(f"Initialized {name} hand")

        # Create results directory
        self.results_dir = Path(log_dir) / f"test_{time.strftime('%Y%m%d_%H%M%S')}"
        self.results_dir.mkdir(parents=True, exist_ok=True)

        # Initialize logger
        self.logger = DexHandLogger(str(self.results_dir))

        # Initialize test results storage
        self.results = {
            "individual": {joint: [] for joint in JOINT_CONFIGS},
            "simultaneous": [],
            "consecutive": {"commands": [], "feedback": []},
        }

    def run_tests(self):
        """Run all test sequences"""
        try:
            # Test 1: Reset all joints
            logger.info("Test 1: Reset all joints with steady-state analysis")
            self._test_reset()

            # Test 2: Test each joint individually
            logger.info("Test 2: Individual joint tests with steady-state analysis")
            self._test_individual_joints()

            # Test 3: Move all joints together
            logger.info("Test 3: Simultaneous movement test")
            self._test_simultaneous()

            # Test 4: Consecutive small movements
            logger.info("Test 4: Consecutive movement test with feedback tracking")
            self._test_consecutive()

            # Generate report
            # self._generate_report()

        except Exception as e:
            logger.error(f"Test sequence failed: {e}", exc_info=True)
            raise

    def _collect_steady_state(
        self, target_angles: Dict[str, float], settling_time: Optional[float] = None
    ) -> List[HandFeedback]:
        """Collect multiple feedback samples after settling

        Args:
            target_angles: Dictionary of joint angles
            settling_time: Optional override for settling time

        Returns:
            List of feedback samples
        """
        # Wait for settling
        time.sleep(settling_time or self.settling_time)

        # Collect samples
        samples = []
        for _ in range(self.n_samples):
            for name, hand in self.hands.items():
                hand.move_joints(**target_angles)
                feedback = hand.get_feedback()
                self.logger.log_feedback(feedback, name)
                samples.append(feedback)

        return samples

    def _compute_steady_state_metrics(
        self, joint_name: str, target_angle: float, samples: List[HandFeedback]
    ) -> Tuple[float, float]:
        """Compute steady-state metrics for a joint

        Args:
            joint_name: Name of joint
            target_angle: Commanded angle
            samples: List of feedback samples

        Returns:
            Tuple of (steady_state_angle, steady_state_error)
        """
        # Extract angles for the joint from samples
        angles = []
        for feedback in samples:
            if joint_name in feedback.joints:
                angles.append(feedback.joints[joint_name].angle)

        if not angles:
            return float("nan"), float("nan")

        # Compute metrics
        ss_angle = angles[-1]  # Last sample is assumed to be steady state
        ss_error = target_angle - ss_angle

        return ss_angle, ss_error

    def _test_reset(self):
        """Reset all joints and analyze steady-state"""
        start_time = time.time()

        # Send reset command
        for name, hand in self.hands.items():
            hand.reset_joints()
            feedback = hand.get_feedback()
            self.logger.log_command(
                "reset_joints",
                {joint: 0.0 for joint in JOINT_CONFIGS},
                ControlMode.CASCADED_PID,
                name,
                feedback,
            )

        # Collect steady state data
        samples = self._collect_steady_state({})

        # Analyze each joint
        for joint in JOINT_CONFIGS:
            ss_angle, ss_error = self._compute_steady_state_metrics(joint, 0.0, samples)

            self.results["individual"][joint].append(
                TestResult(
                    timestamp=start_time,
                    command_angle=0.0,
                    actual_angle=ss_angle,
                    steady_state_angle=ss_angle,
                    steady_state_error=ss_error,
                    settling_time=self.settling_time,
                    error_msg=None,
                )
            )

    def _test_individual_joints(self):
        """Test each joint's full range with steady-state analysis"""
        for joint_name, config in JOINT_CONFIGS.items():
            logger.info(f"Testing joint: {joint_name}")

            try:
                # Move to maximum position
                max_angle = config.max_angle
                start_time = time.time()

                for name, hand in self.hands.items():
                    hand.move_joints(**{joint_name: max_angle})
                    feedback = hand.get_feedback()
                    hand.clear_errors(clear_all=True)
                    self.logger.log_command(
                        "move_joint",
                        {joint_name: max_angle},
                        ControlMode.CASCADED_PID,
                        name,
                        feedback,
                    )

                # Collect steady state data
                samples = self._collect_steady_state({joint_name: max_angle})
                ss_angle, ss_error = self._compute_steady_state_metrics(
                    joint_name, max_angle, samples
                )

                self.results["individual"][joint_name].append(
                    TestResult(
                        timestamp=start_time,
                        command_angle=max_angle,
                        actual_angle=samples[0].joints[joint_name].angle,
                        steady_state_angle=ss_angle,
                        steady_state_error=ss_error,
                        settling_time=self.settling_time,
                    )
                )

                # Return to zero
                start_time = time.time()
                for name, hand in self.hands.items():
                    hand.move_joints(**{joint_name: 0.0})
                    feedback = hand.get_feedback()
                    hand.clear_errors(clear_all=True)
                    self.logger.log_command(
                        "move_joint",
                        {joint_name: 0.0},
                        ControlMode.CASCADED_PID,
                        name,
                        feedback,
                    )

                # Collect steady state data
                samples = self._collect_steady_state({joint_name: 0.0})
                ss_angle, ss_error = self._compute_steady_state_metrics(
                    joint_name, 0.0, samples
                )

                self.results["individual"][joint_name].append(
                    TestResult(
                        timestamp=start_time,
                        command_angle=0.0,
                        actual_angle=samples[0].joints[joint_name].angle,
                        steady_state_angle=ss_angle,
                        steady_state_error=ss_error,
                        settling_time=self.settling_time,
                    )
                )

            except Exception as e:
                logger.error(f"Failed testing {joint_name}: {e}")

    def _test_simultaneous(self):
        """Test simultaneous movement of all joints with steady-state analysis"""
        logger.info("Testing simultaneous movement of all joints")
        target_angle = 30.0  # Use moderate angle for all joints

        try:
            # First movement: All joints to target
            start_time = time.time()
            joint_commands = {
                name: min(target_angle, config.max_angle)
                for name, config in JOINT_CONFIGS.items()
            }

            # Send commands
            for name, hand in self.hands.items():
                hand.move_joints(**joint_commands)
                feedback = hand.get_feedback()
                hand.clear_errors(clear_all=True)
                self.logger.log_command(
                    "simultaneous",
                    joint_commands,
                    ControlMode.CASCADED_PID,
                    name,
                    feedback,
                )

            # Collect steady state data
            logger.info(f"Waiting {self.settling_time}s for settling...")
            samples = self._collect_steady_state(joint_commands)

            # Analyze steady state for each joint
            for joint_name in JOINT_CONFIGS:
                ss_angle, ss_error = self._compute_steady_state_metrics(
                    joint_name, joint_commands[joint_name], samples
                )

                result = TestResult(
                    timestamp=start_time,
                    command_angle=joint_commands[joint_name],
                    actual_angle=samples[0].joints[joint_name].angle,
                    steady_state_angle=ss_angle,
                    steady_state_error=ss_error,
                    settling_time=self.settling_time,
                )
                self.results["simultaneous"].append(result)

                logger.info(
                    f"{joint_name}: Target={joint_commands[joint_name]:.1f}°, "
                    f"Steady-state={ss_angle:.1f}°, Error={ss_error:.1f}°"
                )

            # Second movement: Return to zero
            logger.info("Returning all joints to zero")
            start_time = time.time()
            zero_commands = {name: 0.0 for name in JOINT_CONFIGS}

            for name, hand in self.hands.items():
                hand.move_joints(**zero_commands)
                feedback = hand.get_feedback()
                hand.clear_errors(clear_all=True)
                self.logger.log_command(
                    "simultaneous",
                    zero_commands,
                    ControlMode.CASCADED_PID,
                    name,
                    feedback,
                )

            # Collect steady state data for return to zero
            logger.info(f"Waiting {self.settling_time}s for settling...")
            samples = self._collect_steady_state(zero_commands)

            # Analyze return to zero for each joint
            for joint_name in JOINT_CONFIGS:
                ss_angle, ss_error = self._compute_steady_state_metrics(
                    joint_name, 0.0, samples
                )

                result = TestResult(
                    timestamp=start_time,
                    command_angle=0.0,
                    actual_angle=samples[0].joints[joint_name].angle,
                    steady_state_angle=ss_angle,
                    steady_state_error=ss_error,
                    settling_time=self.settling_time,
                )
                self.results["simultaneous"].append(result)

                logger.info(
                    f"{joint_name}: Target=0.0°, "
                    f"Steady-state={ss_angle:.1f}°, Error={ss_error:.1f}°"
                )

            # Add simultaneous movement plots to report
            self._plot_simultaneous_results()

        except Exception as e:
            logger.error(f"Error in simultaneous movement test: {e}")
            raise

    def _test_consecutive(self):
        """Test rapid consecutive movements with feedback tracking and timing"""
        feedback_data = []
        timing_data = []

        # Forward sequence (0 to 30 degrees)
        for angle in range(31):
            step_start = time.perf_counter()  # Use perf_counter for precise timing
            joint_commands = {
                name: min(float(angle), config.max_angle)
                for name, config in JOINT_CONFIGS.items()
            }

            # Record command
            self.results["consecutive"]["commands"].append(
                {"timestamp": time.time(), "angle": angle}
            )

            # Send command and measure time
            command_times = []
            feedback_times = []

            for name, hand in self.hands.items():
                # Time command sending
                cmd_start = time.perf_counter()
                hand.clear_errors(clear_all=True)
                hand.move_joints(**joint_commands)
                cmd_end = time.perf_counter()
                command_times.append(cmd_end - cmd_start)

                # Time feedback collection
                fb_start = time.perf_counter()
                feedback = hand.get_feedback()  # Explicitly get feedback
                fb_end = time.perf_counter()
                feedback_times.append(fb_end - fb_start)

                self.logger.log_command(
                    "consecutive",
                    joint_commands,
                    ControlMode.CASCADED_PID,
                    name,
                    feedback,
                )

                # Record feedback
                feedback_data.append(
                    {
                        "timestamp": time.time(),
                        "command": angle,
                        "direction": "up",
                        "joints": {
                            name: fb.angle for name, fb in feedback.joints.items()
                        },
                    }
                )

            step_end = time.perf_counter()

            # Record timing data
            timing_data.append(
                {
                    "angle": angle,
                    "direction": "up",
                    "total_time": step_end - step_start,
                    "command_time": np.mean(command_times),
                    "feedback_time": np.mean(feedback_times),
                    "command_time_std": np.std(command_times),
                    "feedback_time_std": np.std(feedback_times),
                }
            )

            time.sleep(0.02)  # Small delay between commands

        # Reverse sequence (30 to 0 degrees)
        for angle in range(30, -1, -1):
            step_start = time.perf_counter()
            joint_commands = {
                name: min(float(angle), config.max_angle)
                for name, config in JOINT_CONFIGS.items()
            }

            # Record command
            self.results["consecutive"]["commands"].append(
                {"timestamp": time.time(), "angle": angle}
            )

            # Send command and measure time
            command_times = []
            feedback_times = []

            for name, hand in self.hands.items():
                # Time command sending
                cmd_start = time.perf_counter()
                hand.clear_errors(clear_all=True)
                hand.move_joints(**joint_commands)
                cmd_end = time.perf_counter()
                command_times.append(cmd_end - cmd_start)

                # Time feedback collection
                fb_start = time.perf_counter()
                feedback = hand.get_feedback()
                fb_end = time.perf_counter()
                feedback_times.append(fb_end - fb_start)

                self.logger.log_command(
                    "consecutive",
                    joint_commands,
                    ControlMode.CASCADED_PID,
                    name,
                    feedback,
                )

                # Record feedback
                feedback_data.append(
                    {
                        "timestamp": time.time(),
                        "command": angle,
                        "direction": "down",
                        "joints": {
                            name: fb.angle for name, fb in feedback.joints.items()
                        },
                    }
                )

            step_end = time.perf_counter()

            # Record timing data
            timing_data.append(
                {
                    "angle": angle,
                    "direction": "down",
                    "total_time": step_end - step_start,
                    "command_time": np.mean(command_times),
                    "feedback_time": np.mean(feedback_times),
                    "command_time_std": np.std(command_times),
                    "feedback_time_std": np.std(feedback_times),
                }
            )

            time.sleep(0.02)  # Small delay between commands

        # Store results
        self.results["consecutive"].update(
            {"feedback": feedback_data, "timing": timing_data}
        )

    def _generate_report(self):
        """Generate comprehensive test report with all test results"""
        report_path = self.results_dir / "test_report.txt"

        with open(report_path, "w") as f:
            f.write("DexHand Test Report\n")
            f.write("=" * 80 + "\n\n")

            # 1. Individual joint results
            f.write("1. Individual Joint Tests\n")
            f.write("-" * 40 + "\n")
            for joint_name, results in self.results["individual"].items():
                f.write(f"\n{joint_name}:\n")

                for i, result in enumerate(results):
                    movement = "Maximum" if i == 0 else "Return to Zero"
                    f.write(f"\n  {movement} Movement:\n")
                    f.write(f"    Command: {result.command_angle:.1f}°\n")
                    f.write(
                        f"    Steady-state angle: {result.steady_state_angle:.2f}°\n"
                    )
                    f.write(
                        f"    Steady-state error: {result.steady_state_error:.2f}°\n"
                    )
                    f.write(f"    Settling time: {result.settling_time:.2f}s\n")
                    if result.error_msg:
                        f.write(f"    Error: {result.error_msg}\n")

            # 2. Simultaneous movement results
            f.write("\n\n2. Simultaneous Movement Test\n")
            f.write("-" * 40 + "\n")

            # Group results by joint
            joint_results = {}
            for result in self.results["simultaneous"]:
                joint_name = next(
                    name
                    for name in JOINT_CONFIGS
                    if abs(result.command_angle - 30.0) < 0.1
                    or abs(result.command_angle) < 0.1
                )
                if joint_name not in joint_results:
                    joint_results[joint_name] = []
                joint_results[joint_name].append(result)

            for joint_name, results in joint_results.items():
                f.write(f"\n{joint_name}:\n")

                # Forward movement
                f.write("  Forward Movement:\n")
                f.write(f"    Command: {results[0].command_angle:.1f}°\n")
                f.write(
                    f"    Steady-state angle: {results[0].steady_state_angle:.2f}°\n"
                )
                f.write(
                    f"    Steady-state error: {results[0].steady_state_error:.2f}°\n"
                )
                f.write(f"    Settling time: {results[0].settling_time:.2f}s\n")

                # Return movement
                f.write("  Return Movement:\n")
                f.write(f"    Command: {results[1].command_angle:.1f}°\n")
                f.write(
                    f"    Steady-state angle: {results[1].steady_state_angle:.2f}°\n"
                )
                f.write(
                    f"    Steady-state error: {results[1].steady_state_error:.2f}°\n"
                )
                f.write(f"    Settling time: {results[1].settling_time:.2f}s\n")

            # 3. Consecutive movement analysis
            f.write("\n\n3. Consecutive Movement Analysis\n")
            f.write("-" * 40 + "\n")

            # Convert feedback data to numpy arrays for analysis
            feedback_data = self.results["consecutive"]["feedback"]
            if feedback_data:
                # Group by direction
                up_data = [d for d in feedback_data if d["direction"] == "up"]
                down_data = [d for d in feedback_data if d["direction"] == "down"]

                # Analyze each joint
                joints = feedback_data[0]["joints"].keys()
                for joint in joints:
                    f.write(f"\n{joint}:\n")

                    # Upward movement
                    up_errors = [d["joints"][joint] - d["command"] for d in up_data]
                    f.write("  Upward Movement:\n")
                    f.write(f"    Mean error: {np.mean(up_errors):.2f}°\n")
                    f.write(f"    Max error: {np.max(np.abs(up_errors)):.2f}°\n")
                    f.write(
                        f"    RMS error: {np.sqrt(np.mean(np.array(up_errors)**2)):.2f}°\n"
                    )

                    # Downward movement
                    down_errors = [d["joints"][joint] - d["command"] for d in down_data]
                    f.write("  Downward Movement:\n")
                    f.write(f"    Mean error: {np.mean(down_errors):.2f}°\n")
                    f.write(f"    Max error: {np.max(np.abs(down_errors)):.2f}°\n")
                    f.write(
                        f"    RMS error: {np.sqrt(np.mean(np.array(down_errors)**2)):.2f}°\n"
                    )

            # 4. Generate performance comparison
            f.write("\n\n4. Performance Comparison\n")
            f.write("-" * 40 + "\n")

            # Compare steady-state errors across test types
            for joint_name in JOINT_CONFIGS:
                f.write(f"\n{joint_name}:\n")

                # Individual test errors
                ind_errors = [
                    r.steady_state_error for r in self.results["individual"][joint_name]
                ]
                f.write("  Individual Movement:\n")
                f.write(f"    Forward Error: {ind_errors[0]:.2f}°\n")
                f.write(f"    Return Error: {ind_errors[1]:.2f}°\n")

                # Simultaneous test errors
                sim_results = joint_results[joint_name]
                f.write("  Simultaneous Movement:\n")
                f.write(
                    f"    Forward Error: {sim_results[0].steady_state_error:.2f}°\n"
                )
                f.write(f"    Return Error: {sim_results[1].steady_state_error:.2f}°\n")

            # Generate error comparison plots
            self._plot_steady_state_errors()
            self._plot_simultaneous_results()
            self._plot_consecutive_movement()
            self._plot_control_timing()

    def _plot_steady_state_errors(self):
        """Plot steady-state errors for all joints"""
        plt.figure(figsize=(15, 10))

        joints = list(JOINT_CONFIGS.keys())
        errors = []

        for joint in joints:
            # Get maximum angle test result
            max_result = [
                r for r in self.results["individual"][joint] if r.command_angle > 0
            ][0]
            errors.append(abs(max_result.steady_state_error))

        plt.bar(joints, errors)
        plt.xticks(rotation=45)
        plt.ylabel("Absolute Steady-State Error (degrees)")
        plt.title("Steady-State Errors by Joint")
        plt.grid(True)
        plt.tight_layout()

        plt.savefig(self.results_dir / "steady_state_errors.png")
        plt.close()

    def _plot_consecutive_movement(self):
        """Plot commands vs feedback for consecutive movements"""
        feedback_df = pd.DataFrame(self.results["consecutive"]["feedback"])
        commands_df = pd.DataFrame(self.results["consecutive"]["commands"])

        plt.figure(figsize=(15, 10))

        # Plot commands
        base_time = feedback_df["timestamp"].iloc[0]
        feedback_df["time"] = feedback_df["timestamp"] - base_time
        commands_df["time"] = commands_df["timestamp"] - base_time

        # Plot command trajectory
        plt.plot(
            commands_df["time"],
            commands_df["angle"],
            "k--",
            label="Command",
            linewidth=2,
        )

        # Plot each joint's response
        joint_cols = [col for col in feedback_df["joints"].iloc[0].keys()]
        for joint in joint_cols:
            angles = [joints[joint] for joints in feedback_df["joints"]]
            plt.plot(
                feedback_df["time"],
                angles,
                ".-",
                label=f"{joint}",
                alpha=0.7,
                markersize=2,
            )

        plt.xlabel("Time (s)")
        plt.ylabel("Angle (degrees)")
        plt.title("Joint Responses to Consecutive Commands")
        plt.grid(True)
        plt.legend(bbox_to_anchor=(1.05, 1), loc="upper left")
        plt.tight_layout()

        plt.savefig(self.results_dir / "consecutive_movement.png")
        plt.close()

        # Plot tracking errors
        plt.figure(figsize=(15, 10))
        for joint in joint_cols:
            angles = np.array([joints[joint] for joints in feedback_df["joints"]])
            # Interpolate commands to match feedback timestamps
            cmd_interp = np.interp(
                feedback_df["time"], commands_df["time"], commands_df["angle"]
            )
            error = angles - cmd_interp
            plt.plot(
                feedback_df["time"],
                error,
                ".-",
                label=f"{joint}",
                alpha=0.7,
                markersize=2,
            )

        plt.xlabel("Time (s)")
        plt.ylabel("Tracking Error (degrees)")
        plt.title("Joint Tracking Errors During Consecutive Commands")
        plt.grid(True)
        plt.legend(bbox_to_anchor=(1.05, 1), loc="upper left")
        plt.tight_layout()

        plt.savefig(self.results_dir / "tracking_errors.png")
        plt.close()

        # Generate tracking error statistics
        stats_path = self.results_dir / "tracking_statistics.txt"
        with open(stats_path, "w") as f:
            f.write("Tracking Error Statistics\n")
            f.write("=" * 80 + "\n\n")

            for joint in joint_cols:
                f.write(f"\n{joint}:\n")
                angles = np.array([joints[joint] for joints in feedback_df["joints"]])
                cmd_interp = np.interp(
                    feedback_df["time"], commands_df["time"], commands_df["angle"]
                )
                error = angles - cmd_interp

                # Calculate statistics
                rms_error = np.sqrt(np.mean(error**2))
                max_error = np.max(np.abs(error))
                mean_error = np.mean(error)
                std_error = np.std(error)

                f.write(f"  RMS Error: {rms_error:.2f}°\n")
                f.write(f"  Max Absolute Error: {max_error:.2f}°\n")
                f.write(f"  Mean Error: {mean_error:.2f}°\n")
                f.write(f"  Standard Deviation: {std_error:.2f}°\n")

    def _plot_simultaneous_results(self):
        """Generate plots for simultaneous movement test results"""

        # Plot steady-state errors for forward and return movements
        plt.figure(figsize=(15, 10))

        # Group results by joint
        joint_results = {}
        for result in self.results["simultaneous"]:
            joint_name = next(
                name
                for name in JOINT_CONFIGS
                if abs(result.command_angle - 30.0) < 0.1
                or abs(result.command_angle) < 0.1
            )
            if joint_name not in joint_results:
                joint_results[joint_name] = []
            joint_results[joint_name].append(result)

        # Plot errors for each joint
        x = np.arange(len(JOINT_CONFIGS))
        width = 0.35

        forward_errors = [
            abs(results[0].steady_state_error) for results in joint_results.values()
        ]
        return_errors = [
            abs(results[1].steady_state_error) for results in joint_results.values()
        ]

        plt.bar(x - width / 2, forward_errors, width, label="Forward Movement")
        plt.bar(x + width / 2, return_errors, width, label="Return Movement")

        plt.xlabel("Joints")
        plt.ylabel("Absolute Steady-State Error (degrees)")
        plt.title("Steady-State Errors During Simultaneous Movement")
        plt.xticks(x, list(JOINT_CONFIGS.keys()), rotation=45)
        plt.legend()
        plt.grid(True)
        plt.tight_layout()

        plt.savefig(self.results_dir / "simultaneous_errors.png")
        plt.close()

    def _plot_control_timing(self):
        """Plot control loop timing statistics"""
        timing_data = self.results["consecutive"].get("timing", [])
        if not timing_data:
            return

        plt.figure(figsize=(15, 10))

        # Create arrays for plotting
        angles = [d["angle"] for d in timing_data]
        total_times = [d["total_time"] * 1000 for d in timing_data]  # Convert to ms
        command_times = [d["command_time"] * 1000 for d in timing_data]
        feedback_times = [d["feedback_time"] * 1000 for d in timing_data]

        # Plot timing components
        plt.plot(angles, total_times, "k-", label="Total Step Time")
        plt.plot(angles, command_times, "b-", label="Command Time")
        plt.plot(angles, feedback_times, "r-", label="Feedback Time")

        # Add error bars for command and feedback times
        command_stds = [d["command_time_std"] * 1000 for d in timing_data]
        feedback_stds = [d["feedback_time_std"] * 1000 for d in timing_data]

        plt.fill_between(
            angles,
            np.array(command_times) - np.array(command_stds),
            np.array(command_times) + np.array(command_stds),
            color="b",
            alpha=0.2,
        )
        plt.fill_between(
            angles,
            np.array(feedback_times) - np.array(feedback_stds),
            np.array(feedback_times) + np.array(feedback_stds),
            color="r",
            alpha=0.2,
        )

        plt.xlabel("Command Angle (degrees)")
        plt.ylabel("Time (ms)")
        plt.title("Control Loop Timing Analysis")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        plt.savefig(self.results_dir / "control_timing.png")
        plt.close()

        # Also create timing statistics file
        stats_path = self.results_dir / "timing_statistics.txt"
        with open(stats_path, "w") as f:
            f.write("Control Loop Timing Statistics\n")
            f.write("=" * 80 + "\n\n")

            # Overall statistics
            total_times = np.array([d["total_time"] * 1000 for d in timing_data])
            command_times = np.array([d["command_time"] * 1000 for d in timing_data])
            feedback_times = np.array([d["feedback_time"] * 1000 for d in timing_data])

            f.write("Overall Statistics (milliseconds):\n")
            f.write("-" * 40 + "\n")
            f.write(f"Total Step Time:\n")
            f.write(f"  Mean: {np.mean(total_times):.2f} ms\n")
            f.write(f"  Std:  {np.std(total_times):.2f} ms\n")
            f.write(f"  Min:  {np.min(total_times):.2f} ms\n")
            f.write(f"  Max:  {np.max(total_times):.2f} ms\n\n")

            f.write(f"Command Time:\n")
            f.write(f"  Mean: {np.mean(command_times):.2f} ms\n")
            f.write(f"  Std:  {np.std(command_times):.2f} ms\n")
            f.write(f"  Min:  {np.min(command_times):.2f} ms\n")
            f.write(f"  Max:  {np.max(command_times):.2f} ms\n\n")

            f.write(f"Feedback Time:\n")
            f.write(f"  Mean: {np.mean(feedback_times):.2f} ms\n")
            f.write(f"  Std:  {np.std(feedback_times):.2f} ms\n")
            f.write(f"  Min:  {np.min(feedback_times):.2f} ms\n")
            f.write(f"  Max:  {np.max(feedback_times):.2f} ms\n\n")

            # Compare up vs down movement timing
            f.write("\nUp vs Down Movement Comparison:\n")
            f.write("-" * 40 + "\n")

            up_data = [d for d in timing_data if d["direction"] == "up"]
            down_data = [d for d in timing_data if d["direction"] == "down"]

            up_times = np.array([d["total_time"] * 1000 for d in up_data])
            down_times = np.array([d["total_time"] * 1000 for d in down_data])

            f.write(f"Up Movement:\n")
            f.write(f"  Mean: {np.mean(up_times):.2f} ms\n")
            f.write(f"  Std:  {np.std(up_times):.2f} ms\n\n")

            f.write(f"Down Movement:\n")
            f.write(f"  Mean: {np.mean(down_times):.2f} ms\n")
            f.write(f"  Std:  {np.std(down_times):.2f} ms\n")

    def close(self):
        """Clean up resources"""
        try:
            # Close each hand
            for hand in self.hands.values():
                try:
                    hand.close()
                except Exception as e:
                    logger.error(f"Error closing hand: {e}")

            # Close logger
            try:
                self.logger.close()
            except Exception as e:
                logger.error(f"Error closing logger: {e}")

        except Exception as e:
            logger.error(f"Error during cleanup: {e}")


def main():
    parser = argparse.ArgumentParser(description="Test dexterous hand control")
    parser.add_argument(
        "--hands",
        nargs="+",
        choices=["left", "right"],
        default=["left"],
        help="Which hands to test",
    )
    parser.add_argument(
        "--log-dir", type=str, default="dexhand_logs", help="Directory for test logs"
    )
    parser.add_argument(
        "--settling-time",
        type=float,
        default=1.0,
        help="Time to wait for settling (seconds)",
    )
    parser.add_argument(
        "--n-samples",
        type=int,
        default=5,
        help="Number of samples for steady-state analysis",
    )
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")
    args = parser.parse_args()

    # Set logging level
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    tester = None
    try:
        # Initialize and run tests
        tester = DexHandTester(
            args.hands, args.log_dir, args.settling_time, args.n_samples
        )
        tester.run_tests()

    except KeyboardInterrupt:
        logger.info("Test interrupted by user")

    except Exception as e:
        logger.error(f"Test failed: {str(e)}", exc_info=args.debug)

    finally:
        if tester is not None:
            try:
                tester.close()
            except Exception as e:
                logger.error(f"Error during cleanup: {str(e)}", exc_info=args.debug)
        logger.info("Test completed")


if __name__ == "__main__":
    main()
