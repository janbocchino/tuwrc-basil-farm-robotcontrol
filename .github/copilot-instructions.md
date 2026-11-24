# Code review instructions

Review every pull request for correctness, security, regressions, and missing tests. Check the PR against CONTRIBUTING.md and the relevant workstream interfaces.

Pay particular attention to motion and hardware safety, joint limits, calibration, URDF geometry, controller behavior, and failures that could cause unexpected movement. Flag undocumented changes to physically measured values and assumptions. Review changes to shared ROS interfaces for compatibility across workstreams. Check that tests and the ROS build cover the change, and highlight any behavior that needs physical verification or a view-mode smoke test.

Do not treat a passing build or tests as evidence that physical calibration or robot behavior is safe.
