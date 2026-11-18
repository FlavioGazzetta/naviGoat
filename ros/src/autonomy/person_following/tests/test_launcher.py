"""Run the subscriber-only launcher with fake ROS dependencies on the laptop."""

import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class FollowingLauncherTests(unittest.TestCase):
    def run_launcher(self, setup_text, arguments=()):
        runner_name = "ros_nav2_follow.py"
        launcher = Path(__file__).resolve().parents[1] / "run_nav2_follow.sh"
        # Execute the actual script against a temporary ROS setup and a stub
        # Python executable. No camera, server, model or ROS process can start.
        with tempfile.TemporaryDirectory(prefix="following-launcher-") as temporary:
            root = Path(temporary)
            (root / runner_name).touch()
            setup = root / "setup.bash"
            # This function checks options in the LAUNCHER's shell during its
            # dependency check. exec later bypasses it and uses the stub below.
            setup.write_text(setup_text + '''
python3() {
  [[ "${FOLLOWING_TEST_SETUP_LOADED:-}" == yes ]]
  [[ -o nounset && -o errexit && -o errtrace && -o pipefail ]]
  printf 'preflight completed with strict options restored\\n'
}
''')
            bin_dir = root / "bin"
            bin_dir.mkdir()
            stub = bin_dir / "python3"
            stub.write_text('#!/bin/sh\nprintf "runner-arg:%s\\n" "$@"\n')
            stub.chmod(0o755)
            script = root / "run_nav2_follow.sh"
            script.write_text(launcher.read_text().replace("/opt/ros/jazzy/setup.bash", str(setup)))
            environment = os.environ.copy()
            environment["PATH"] = str(bin_dir) + os.pathsep + environment["PATH"]
            environment.pop("AMENT_TRACE_SETUP_FILES", None)
            environment.pop("FOLLOWING_TEST_SETUP_LOADED", None)
            return subprocess.run(["bash", str(script), *arguments], env=environment,
                                  capture_output=True, text=True, timeout=5)

    def test_ros_setup_may_read_unset_variables_then_strict_checks_resume(self):
        # This optional-variable access is the pattern in Jazzy's setup scripts.
        result = self.run_launcher('''
if [ -n "$AMENT_TRACE_SETUP_FILES" ]; then :; fi
export FOLLOWING_TEST_SETUP_LOADED=yes
''')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("preflight completed", result.stdout)
        self.assertIn("ros_nav2_follow.py", result.stdout)

    def test_setup_failure_still_aborts_startup(self):
        result = self.run_launcher("return 42\n")
        self.assertEqual(result.returncode, 42, result.stderr)
        self.assertNotIn("preflight completed", result.stdout)
        self.assertNotIn("runner-arg:", result.stdout)

    def test_only_subscriber_starts_and_runner_arguments_are_forwarded(self):
        result = self.run_launcher("export FOLLOWING_TEST_SETUP_LOADED=yes\n",
                                   ("--enable-on-start", "--topic", "/custom/tracks"))
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = [line for line in result.stdout.splitlines() if line.startswith("runner-arg:")]
        self.assertEqual(len(calls), 6)  # -B, -u, script, and the three explicit arguments
        self.assertEqual(calls[:2], ["runner-arg:-B", "runner-arg:-u"])
        self.assertTrue(calls[2].endswith("/ros_nav2_follow.py"))
        self.assertEqual(calls[3:], ["runner-arg:--enable-on-start", "runner-arg:--topic",
                                    "runner-arg:/custom/tracks"])

    def test_default_does_not_enable_following_without_a_mode_command(self):
        result = self.run_launcher("export FOLLOWING_TEST_SETUP_LOADED=yes\n")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn("runner-arg:--enable-on-start", result.stdout)

    def test_nav2_wrapper_defaults_to_preview_and_forwards_opt_in(self):
        for arguments in ((), ("--execute-nav2", "--global-frame", "odom")):
            with self.subTest(arguments=arguments):
                result = self.run_launcher("export FOLLOWING_TEST_SETUP_LOADED=yes\n", arguments)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("ros_nav2_follow.py", result.stdout)
                if arguments:
                    self.assertIn("runner-arg:--execute-nav2", result.stdout)
                    self.assertIn("runner-arg:odom", result.stdout)
                else:
                    self.assertNotIn("runner-arg:--execute-nav2", result.stdout)


if __name__ == "__main__":
    unittest.main()
