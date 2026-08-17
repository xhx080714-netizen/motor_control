#!/usr/bin/env python3

"""D3-C7 black-box integration test for the ROS 2 safety command chain."""

from collections import deque
from dataclasses import dataclass
from datetime import datetime
import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import tempfile
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sand_rake_interfaces.msg import SafetyEvent
from std_srvs.srv import Trigger


COMMAND_PERIOD_SEC = 0.05
FLOAT_TOLERANCE = 1.0e-6
WATCHDOG_LIMIT_SEC = 0.500


@dataclass
class CaseResult:
    """One human-readable integration-test result."""

    number: int
    name: str
    passed: bool
    expected: str = ""
    actual: str = ""
    details: str = ""


class CaseFailure(RuntimeError):
    """Assertion failure with report-ready diagnostic fields."""

    def __init__(self, expected, actual, details=""):
        super().__init__(actual)
        self.expected = expected
        self.actual = actual
        self.details = details


class SafetyBlackBoxDriver(Node):
    """ROS test driver which only uses public topics and services."""

    def __init__(self):
        super().__init__("d3_c7_safety_black_box_test")
        self.output_samples = deque(maxlen=1000)
        self.cmd_pub = self.create_publisher(
            Twist, "/teleop/cmd_vel", 10)
        self.event_pub = self.create_publisher(
            SafetyEvent, "/safety/event", 10)
        self.output_sub = self.create_subscription(
            Twist, "/safety/cmd_vel", self._output_callback, 10)
        self.reset_client = self.create_client(Trigger, "/safety/reset")

    def _output_callback(self, message):
        self.output_samples.append((time.monotonic(), message))

    def spin_for(self, duration_sec):
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(self, timeout_sec=min(0.02, remaining))

    def wait_for_output(self, predicate, timeout_sec, after=None):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            for stamp, message in list(self.output_samples):
                if (after is None or stamp >= after) and predicate(message):
                    return stamp, message
            rclpy.spin_once(self, timeout_sec=0.01)
        return None

    def publish_for(self, message, duration_sec):
        deadline = time.monotonic() + duration_sec
        next_publish = time.monotonic()
        last_publish = None
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                self.cmd_pub.publish(message)
                last_publish = time.monotonic()
                next_publish += COMMAND_PERIOD_SEC
            timeout = max(0.0, min(0.01, deadline - time.monotonic()))
            rclpy.spin_once(self, timeout_sec=timeout)
        return last_publish

    def send_event(self, stop, reason):
        message = SafetyEvent()
        message.stamp = self.get_clock().now().to_msg()
        message.stop = stop
        message.reason = reason
        self.event_pub.publish(message)
        return time.monotonic()

    def call_reset(self, keep_publishing=None, timeout_sec=2.0):
        future = self.reset_client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_sec
        next_publish = time.monotonic()
        while time.monotonic() < deadline and not future.done():
            now = time.monotonic()
            if keep_publishing is not None and now >= next_publish:
                self.cmd_pub.publish(keep_publishing)
                next_publish += COMMAND_PERIOD_SEC
            rclpy.spin_once(self, timeout_sec=0.01)
        if not future.done():
            raise CaseFailure(
                "The /safety/reset service responds within 2.0 s",
                "Service call timed out",
            )
        return future.result()


def make_twist(linear_x=0.0, angular_z=0.0):
    message = Twist()
    message.linear.x = linear_x
    message.angular.z = angular_z
    return message


def twist_values(message):
    return (
        message.linear.x,
        message.linear.y,
        message.linear.z,
        message.angular.x,
        message.angular.y,
        message.angular.z,
    )


def is_zero(message):
    return all(abs(value) < FLOAT_TOLERANCE for value in twist_values(message))


def twist_matches(actual, expected):
    return all(
        math.isclose(a, e, abs_tol=FLOAT_TOLERANCE)
        for a, e in zip(twist_values(actual), twist_values(expected))
    )


def format_twist(message):
    values = twist_values(message)
    return "linear=(%.3f, %.3f, %.3f), angular=(%.3f, %.3f, %.3f)" % values


class D3C7SafetyIntegration:
    """Runs the ten required scenarios against a real controller process."""

    CASE_NAMES = {
        1: "Normal command forwarding",
        2: "LASER emergency stop",
        3: "Latched stop blocks command",
        4: "Clear event stays latched",
        5: "Reset with non-zero cmd rejected",
        6: "Reset after zero command",
        7: "Old command does not recover",
        8: "New command resumes motion",
        9: "TIMEOUT event latches stop",
        10: "Command stream watchdog",
    }

    def __init__(self):
        self.results = []
        self.watchdog_latency = None
        self.driver = None
        self.controller = None
        self.controller_log_path = None
        self.controller_log_file = None
        self.controller_log = ""
        self.ros_log_dir = None
        self.workspace = Path(__file__).resolve().parents[3]
        self.report_path = Path(tempfile.gettempdir()) / (
            "sand_rake_safety_integration_result.md"
        )

    def run(self):
        try:
            self._start()
            self._run_case(1, self._test_normal_forwarding)
            self._run_case(2, self._test_laser_stop)
            self._run_case(3, self._test_latched_block)
            self._run_case(4, self._test_clear_stays_latched)
            self._run_case(5, self._test_nonzero_reset_rejected)
            reset_ok = self._run_case(6, self._test_zero_reset_succeeds)
            if reset_ok:
                self._run_case(7, self._test_old_command_not_restored)
                self._run_case(8, self._test_new_command_resumes)
            else:
                reason = "Not run because TEST 6 did not restore READY"
                self._record_dependency_failure(7, reason)
                self._record_dependency_failure(8, reason)

            ready_for_10 = self._run_case(9, self._test_timeout_event)
            if ready_for_10:
                self._run_case(10, self._test_command_watchdog)
            else:
                self._record_dependency_failure(
                    10, "Not run because TEST 9 cleanup did not restore READY")
        except Exception as error:  # Startup failures need a complete result table.
            missing = {item.number for item in self.results}
            for number in range(1, 11):
                if number not in missing:
                    self._record_dependency_failure(
                        number, "Test harness failure: %s" % error)
        finally:
            self._stop()
            self._print_summary()
            self._write_report()
        return all(item.passed for item in self.results)

    def _start(self):
        domain_id = os.environ.get("D3_C7_ROS_DOMAIN_ID")
        if domain_id is None:
            domain_id = str(100 + os.getpid() % 100)
        os.environ["ROS_DOMAIN_ID"] = domain_id
        self.ros_log_dir = Path(tempfile.mkdtemp(prefix="d3_c7_ros_logs_"))
        os.environ["ROS_LOG_DIR"] = str(self.ros_log_dir)

        rclpy.init()
        self.driver = SafetyBlackBoxDriver()

        command = self._controller_command()
        log_file = tempfile.NamedTemporaryFile(
            prefix="d3_c7_safety_controller_", suffix=".log", delete=False)
        self.controller_log_path = Path(log_file.name)
        self.controller_log_file = log_file
        child_env = os.environ.copy()
        self.controller = subprocess.Popen(
            command,
            env=child_env,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        if not self.driver.reset_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/safety/reset was not discovered within 10 s")

        discovery_deadline = time.monotonic() + 5.0
        while time.monotonic() < discovery_deadline:
            if (
                self.driver.cmd_pub.get_subscription_count() > 0
                and self.driver.event_pub.get_subscription_count() > 0
            ):
                break
            self.driver.spin_for(0.05)
        else:
            raise RuntimeError("Controller topic endpoints were not discovered")

        self.driver.output_samples.clear()
        self.driver.publish_for(make_twist(), 0.25)
        observed = self.driver.wait_for_output(is_zero, 1.0)
        if observed is None:
            raise RuntimeError("No /safety/cmd_vel output was observed")

    def _controller_command(self):
        configured = os.environ.get("SAND_RAKE_SAFETY_CONTROLLER")
        candidates = []
        if configured:
            candidates.append(Path(configured))
        candidates.extend(
            [
                self.workspace / "install/sand_rake_control/lib/"
                "sand_rake_control/safety_controller",
                self.workspace / "build/sand_rake_control/safety_controller",
            ]
        )
        for candidate in candidates:
            if candidate.is_file() and os.access(candidate, os.X_OK):
                return [str(candidate)]
        return ["ros2", "run", "sand_rake_control", "safety_controller"]

    def _stop(self):
        if self.controller is not None and self.controller.poll() is None:
            try:
                os.killpg(self.controller.pid, signal.SIGINT)
                self.controller.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.controller.pid, signal.SIGTERM)
                try:
                    self.controller.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    os.killpg(self.controller.pid, signal.SIGKILL)
                    self.controller.wait(timeout=2.0)

        if self.controller_log_file is not None:
            self.controller_log_file.flush()
            self.controller_log_file.close()
        if self.controller_log_path is not None:
            try:
                self.controller_log = self.controller_log_path.read_text(
                    encoding="utf-8", errors="replace")
            finally:
                self.controller_log_path.unlink(missing_ok=True)

        if self.driver is not None:
            self.driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if self.ros_log_dir is not None:
            shutil.rmtree(self.ros_log_dir, ignore_errors=True)

    def _run_case(self, number, method):
        try:
            details = method() or ""
            self.results.append(
                CaseResult(number, self.CASE_NAMES[number], True, details=details)
            )
            return True
        except CaseFailure as error:
            self.results.append(
                CaseResult(
                    number,
                    self.CASE_NAMES[number],
                    False,
                    error.expected,
                    error.actual,
                    error.details,
                )
            )
        except Exception as error:
            self.results.append(
                CaseResult(
                    number,
                    self.CASE_NAMES[number],
                    False,
                    "Scenario completes without a test-driver error",
                    "%s: %s" % (type(error).__name__, error),
                )
            )
        return False

    def _record_dependency_failure(self, number, reason):
        self.results.append(
            CaseResult(
                number,
                self.CASE_NAMES[number],
                False,
                "Required predecessor restores a known READY state",
                reason,
            )
        )

    def _require_output(self, predicate, timeout, expected, after=None):
        observed = self.driver.wait_for_output(predicate, timeout, after=after)
        if observed is None:
            latest = "No output sample"
            if self.driver.output_samples:
                latest = format_twist(self.driver.output_samples[-1][1])
            raise CaseFailure(expected, latest)
        return observed

    def _test_normal_forwarding(self):
        commands = {
            "W": make_twist(0.10, 0.0),
            "S": make_twist(-0.10, 0.0),
            "A": make_twist(0.10, 0.50),
            "D": make_twist(0.10, -0.50),
            "Q": make_twist(0.0, 0.50),
            "E": make_twist(0.0, -0.50),
        }
        failures = []
        for key, command in commands.items():
            self.driver.output_samples.clear()
            started = time.monotonic()
            self.driver.publish_for(command, 0.25)
            observed = self.driver.wait_for_output(
                lambda item, wanted=command: twist_matches(item, wanted),
                0.5,
                after=started,
            )
            if observed is None:
                actual = "No output"
                if self.driver.output_samples:
                    actual = format_twist(self.driver.output_samples[-1][1])
                failures.append("%s: %s" % (key, actual))
        if failures:
            raise CaseFailure(
                "W/S/A/D/Q/E are forwarded within floating-point tolerance",
                "; ".join(failures),
            )
        return "All six teleop command shapes were forwarded."

    def _test_laser_stop(self):
        command = make_twist(0.10, 0.0)
        self.driver.output_samples.clear()
        self.driver.publish_for(command, 0.35)
        self._require_output(
            lambda item: not is_zero(item), 0.5, "Non-zero W output before LASER stop"
        )

        self.driver.output_samples.clear()
        event_time = self.driver.send_event(True, SafetyEvent.REASON_LASER)
        zero_sample = None
        deadline = time.monotonic() + 1.0
        next_publish = time.monotonic()
        while time.monotonic() < deadline and zero_sample is None:
            now = time.monotonic()
            if now >= next_publish:
                self.driver.cmd_pub.publish(command)
                next_publish += COMMAND_PERIOD_SEC
            zero_sample = self.driver.wait_for_output(
                is_zero, min(0.03, max(0.0, deadline - now)), after=event_time
            )
        if zero_sample is None:
            raise CaseFailure(
                "First zero output within 1.0 s after LASER stop=true",
                "No zero output observed",
            )
        latency_ms = (zero_sample[0] - event_time) * 1000.0
        return "LASER stop-to-zero latency: %.1f ms" % latency_ms

    def _test_latched_block(self):
        command = make_twist(0.10, 0.0)
        self.driver.output_samples.clear()
        started = time.monotonic()
        self.driver.publish_for(command, 0.60)
        samples = [
            message
            for stamp, message in self.driver.output_samples
            if stamp >= started
        ]
        if not samples or any(not is_zero(item) for item in samples):
            nonzero_count = sum(not is_zero(item) for item in samples)
            raise CaseFailure(
                "All safety outputs remain zero for at least 0.5 s",
                "%d samples, %d non-zero" % (len(samples), nonzero_count),
            )
        return "%d zero samples observed over 0.6 s." % len(samples)

    def _test_clear_stays_latched(self):
        self.driver.send_event(False, SafetyEvent.REASON_LASER)
        self.driver.spin_for(0.10)
        command = make_twist(0.10, 0.0)
        self.driver.output_samples.clear()
        started = time.monotonic()
        self.driver.publish_for(command, 0.60)
        samples = [
            message
            for stamp, message in self.driver.output_samples
            if stamp >= started
        ]
        if not samples or any(not is_zero(item) for item in samples):
            raise CaseFailure(
                "LASER stop=false does not release STOP_LATCHED",
                "%d output samples, %d non-zero"
                % (len(samples), sum(not is_zero(item) for item in samples)),
            )
        return "Clear event did not restore motion."

    def _test_nonzero_reset_rejected(self):
        command = make_twist(0.10, 0.0)
        self.driver.publish_for(command, 0.15)
        self.driver.output_samples.clear()
        response = self.driver.call_reset(keep_publishing=command)
        self.driver.publish_for(command, 0.30)
        samples = [message for _, message in self.driver.output_samples]
        if response.success or not samples or any(not is_zero(item) for item in samples):
            raise CaseFailure(
                "reset success=false and safety output remains zero",
                "success=%s, samples=%d, non-zero=%d"
                % (
                    response.success,
                    len(samples),
                    sum(not is_zero(item) for item in samples),
                ),
                "Service message: %s" % response.message,
            )
        return "Reset rejected with service message: %s" % response.message

    def _test_zero_reset_succeeds(self):
        self.driver.publish_for(make_twist(), 0.20)
        response = self.driver.call_reset()
        if not response.success:
            raise CaseFailure(
                "reset success=true after an explicit zero command",
                "success=false",
                "Service message: %s" % response.message,
            )
        return "Reset accepted with service message: %s" % response.message

    def _test_old_command_not_restored(self):
        self.driver.output_samples.clear()
        started = time.monotonic()
        self.driver.spin_for(0.60)
        samples = [
            message
            for stamp, message in self.driver.output_samples
            if stamp >= started
        ]
        if not samples or any(not is_zero(item) for item in samples):
            raise CaseFailure(
                "No old non-zero command reappears for at least 0.5 s",
                "%d output samples, %d non-zero"
                % (len(samples), sum(not is_zero(item) for item in samples)),
            )
        return "%d zero samples observed without a new command." % len(samples)

    def _test_new_command_resumes(self):
        command = make_twist(0.10, 0.0)
        self.driver.output_samples.clear()
        started = time.monotonic()
        self.driver.publish_for(command, 0.30)
        self._require_output(
            lambda item: twist_matches(item, command),
            0.5,
            "A new W command is forwarded after reset",
            after=started,
        )
        return "New W command restored RUNNING output."

    def _test_timeout_event(self):
        if not self._force_running():
            raise CaseFailure(
                "Controller can be placed in RUNNING before TIMEOUT event",
                "Non-zero safety output was not observed",
            )
        command = make_twist(0.10, 0.0)
        self.driver.output_samples.clear()
        event_time = self.driver.send_event(True, SafetyEvent.REASON_TIMEOUT)
        self.driver.publish_for(command, 0.30)
        self._require_output(
            is_zero,
            0.5,
            "TIMEOUT stop=true forces zero output",
            after=event_time,
        )

        self.driver.send_event(False, SafetyEvent.REASON_TIMEOUT)
        self.driver.spin_for(0.10)
        self.driver.output_samples.clear()
        started = time.monotonic()
        self.driver.publish_for(command, 0.55)
        samples = [
            message
            for stamp, message in self.driver.output_samples
            if stamp >= started
        ]
        if not samples or any(not is_zero(item) for item in samples):
            raise CaseFailure(
                "TIMEOUT stop=false remains latched",
                "%d output samples, %d non-zero"
                % (len(samples), sum(not is_zero(item) for item in samples)),
            )

        self.driver.publish_for(make_twist(), 0.20)
        response = self.driver.call_reset()
        if not response.success:
            raise CaseFailure(
                "Zero command then reset restores READY after TIMEOUT",
                "Cleanup reset success=false",
                "Service message: %s" % response.message,
            )
        return "TIMEOUT latched stop and cleanup reset both succeeded."

    def _test_command_watchdog(self):
        command = make_twist(0.10, 0.0)
        self.driver.output_samples.clear()
        last_command_time = self.driver.publish_for(command, 1.10)
        if self.driver.wait_for_output(lambda item: not is_zero(item), 0.3) is None:
            raise CaseFailure(
                "Non-zero output while W is published for at least 1 s",
                "No non-zero output observed",
            )

        self.driver.output_samples.clear()
        zero_sample = self.driver.wait_for_output(
            is_zero, 1.20, after=last_command_time
        )
        if zero_sample is None:
            raise CaseFailure(
                "Watchdog function forces zero after command stream stops",
                "No zero output within 1.2 s",
                "No zero Twist was sent by the test driver after T_last_cmd.",
            )

        self.watchdog_latency = zero_sample[0] - last_command_time
        if self.watchdog_latency > WATCHDOG_LIMIT_SEC:
            raise CaseFailure(
                "FUNCTIONAL PASS and TIMING PASS (latency <= 500.0 ms)",
                "FUNCTIONAL PASS, TIMING FAIL",
                "Measured latency: %.1f ms" % (self.watchdog_latency * 1000.0),
            )
        return "FUNCTIONAL PASS, TIMING PASS; measured latency: %.1f ms" % (
            self.watchdog_latency * 1000.0
        )

    def _force_running(self):
        self.driver.send_event(False, SafetyEvent.REASON_UNKNOWN)
        self.driver.publish_for(make_twist(), 0.20)
        response = self.driver.call_reset()
        if not response.success:
            # A false reset can mean the public behavior is already READY.
            self.driver.spin_for(0.05)
        command = make_twist(0.10, 0.0)
        self.driver.output_samples.clear()
        started = time.monotonic()
        self.driver.publish_for(command, 0.30)
        return self.driver.wait_for_output(
            lambda item: twist_matches(item, command), 0.4, after=started
        ) is not None

    def _print_summary(self):
        print("\n========================================")
        print("D3-C7 Safety Integration Test")
        print("========================================\n")
        for result in sorted(self.results, key=lambda item: item.number):
            status = "PASS" if result.passed else "FAIL"
            print(
                "TEST %-2d %-36s %s"
                % (result.number, result.name, status)
            )
            if not result.passed:
                print("\nExpected:\n%s" % result.expected)
                print("\nActual:\n%s" % result.actual)
                if result.details:
                    print("\nRelevant values:\n%s" % result.details)
                print()
        if self.watchdog_latency is not None:
            print("Watchdog latency: %.1f ms\n" % (self.watchdog_latency * 1000.0))
        passed = sum(item.passed for item in self.results)
        failed = len(self.results) - passed
        print("========================================")
        print("TOTAL: %d PASS / %d FAIL" % (passed, failed))
        print("========================================")

    def _write_report(self):
        branch = self._git_value(["branch", "--show-current"])
        commit = self._git_value(["rev-parse", "HEAD"])
        lines = [
            "# D3-C7 Safety Integration Test Result",
            "",
            "- Test time: %s" % datetime.now().astimezone().isoformat(),
            "- Git branch: `%s`" % branch,
            "- Git commit: `%s`" % commit,
            "- ROS_DISTRO: `%s`" % os.environ.get("ROS_DISTRO", "unset"),
            "- Test script: `src/sand_rake_control/test/"
            "test_safety_integration.py`",
            "- Direct command: `source /opt/ros/humble/setup.bash && "
            "source install/setup.bash && python3 "
            "src/sand_rake_control/test/test_safety_integration.py`",
            "- Colcon command: `colcon test --packages-select "
            "sand_rake_control`",
            "",
            "## Results",
            "",
            "| Test | Scenario | Result | Details |",
            "|---:|---|:---:|---|",
        ]
        for result in sorted(self.results, key=lambda item: item.number):
            status = "PASS" if result.passed else "FAIL"
            detail = result.details or result.actual
            detail = detail.replace("|", "\\|").replace("\n", "<br>")
            lines.append(
                "| %d | %s | %s | %s |"
                % (result.number, result.name, status, detail)
            )

        if self.watchdog_latency is None:
            latency = "Not measured"
        else:
            latency = "%.1f ms" % (self.watchdog_latency * 1000.0)
        lines.extend(["", "## Watchdog", "", "Measured latency: **%s**" % latency])

        failures = [item for item in self.results if not item.passed]
        lines.extend(["", "## Failures", ""])
        if not failures:
            lines.append("None.")
        for result in failures:
            lines.extend(
                [
                    "### TEST %d: %s" % (result.number, result.name),
                    "",
                    "- Expected: %s" % result.expected,
                    "- Actual: %s" % result.actual,
                    "- Relevant values: %s" % (result.details or "None"),
                    "",
                ]
            )

        if failures and self.controller_log:
            tail = "\n".join(self.controller_log.splitlines()[-40:])
            lines.extend(
                [
                    "## Relevant controller log (tail)",
                    "",
                    "```text",
                    tail,
                    "```",
                ]
            )
        self.report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def _git_value(self, arguments):
        try:
            return subprocess.check_output(
                ["git", *arguments],
                cwd=self.workspace,
                text=True,
                stderr=subprocess.DEVNULL,
            ).strip()
        except (OSError, subprocess.CalledProcessError):
            return "unavailable"


def test_d3_c7_safety_integration():
    """Pytest/colcon entry point."""
    suite = D3C7SafetyIntegration()
    assert suite.run(), "One or more D3-C7 safety integration tests failed"


if __name__ == "__main__":
    raise SystemExit(0 if D3C7SafetyIntegration().run() else 1)
