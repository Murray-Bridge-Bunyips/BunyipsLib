# BunyipsLib Changelog
## v2.0.0 (2024-05-03)
Improvement of various subsystems for RoadRunner and Autonomous operation.
### Breaking changes
- The `onReady(OpModeSelection)` abstract method of `AutonomousBunyipsOpMode` has been replaced with a `Reference<?>`, and a second parameter of type `Controls` exists for determining the button that was pressed during the init-phase when `setOpModes()` is called
  - Instead of `protected void onReady(@Nullable OpModeSelection selectedOpMode) {}`, new code uses `protected void onReady(@Nullable Reference<?> selectedOpMode, Controls selectedButton) {}`
  - `OpModeSelection` has been deleted, as it fundamentally serves the same as a `Reference` and has more features to check nullability
  - `selectedOpMode` will return null if the user did not select an OpMode when they had the option to, whilst `selectedOpMode` will return a null _reference_ if there were no OpModes to select from
  - `selectedButton` will be `Controls.NONE` until it is changed by the selection of an OpMode
  - See the `Reference` class for utilising new methods to access objects passed back by `UserSelection`
- Removed unnecessary generic type from `RoadRunnerTask`
- Renamed `HoldableActuator`'s `withZeroHitThreshold(int)` -> `withHomingZeroHits(int)`
- Renamed `GetTeamPropTask` -> `GetTriPositionContourTask`, and removed debugging statement (class still requires testing)
- Removed `RedTeamProp`
  - This class was too specific for BunyipsLib, and teams should implement their own team prop pipelines
  - This is also why `GetTeamPropTask` was renamed, as it applies to a general case
### Non-breaking changes
- Added a safeguard to `Task` where calling `run()` on a finished task will no longer call `periodic()`
  - Ensure your code does not rely on the ignoring of `pollFinished()`
- `setInitTask()` of `AutonomousBunyipsOpMode` has been raised to `BunyipsOpMode`
  - As a result, `BunyipsOpMode` will handle running init-tasks, and don't have to be used solely in Autonomous. This is useful for running a task before starting TeleOp.
  - This does not change the API surface for AutonomousBunyipsOpMode
- `Storage.lastKnownPosition` is now updated every update loop, instead of when the robot is stopped, increasing the accuracy of the last known robot pose.
- Added ticks per second information to `HoldableActuator`
- Added telemetry to `TankDrive` to match `MecanumDrive`
### Bug fixes
- Fixed a race condition where the TelemetryPacket of `DualTelemetry` would be set to null
  - This may bring lost data that are attached to FtcDashboard packets. This will be monitored.
- Fixed a `Rotator` bug where the default "infinite" limit was using `Double.MIN_VALUE` instead of `Double.NEGATIVE_INFINITY`, causing unexpected behaviour
- Fixed telemetry in `Rotator` and `WaitTask` where WPIUnits `toString()` methods were being called instead of rounding
- Fixed an EncoderMotor bug where `holdCurrentPosition()` wouldn't actually hold the current position when called repeatedly
  - Users now must call `resetHoldPosition()` to set a new setpoint for `holdCurrentPosition()` to use, latching to ensure a single position is held
- Trajectory mirroring now works properly, removing code that did not work from the RoadRunner interface
  - RoadRunner trajectories can be mirrored with `.mirrorToRef(Reference<TrajectorySequence>)`
- Fixed flawed calculations of angle in `RoadRunnerTask`, where the angle will now represent the angle the robot is to the end pose using trigonometry
### Additions
- The RoadRunner interface now has new velocity and acceleration constraint creation methods `atVelocity` and `atAcceleration` which integrate with WPIUnits
- `HoldableActuator` HomeTask overcurrent detection algorithm based on an exceeding of current over a constant time value (for example if the current exceeds 4A for 1s straight, the HomeTask will end)
  - These methods are optional
- Added `Limit` utility class for easy construction of velocity and acceleration constraints using WPIUnits (internally used by the RoadRunner interface)
- UserSelection now exposes `selectedButton` field (used in the `onReady()` callback of ABOM)
- Added new `require()`, `ifPresent(Consumer)`, `ifNotPresent(Runnable)`, `ifPresentOrElse(Consumer, Runnable)`, `getOrElse(V)` methods to `Reference`
## v1.0.0 (2024-04-30)
Initial release and first semantic version of BunyipsLib.<br><br>
*BunyipsLib is a comprehensive robotics library tailored for FTC teams, providing a suite of development tools for efficient robot code creation, including modules for error handling, dynamic device initialization, and integrated systems like RoadRunner, FtcDashboard, Vision, Command-based paradigms, and more. Designed for ease of use and versatility, BunyipsLib aims to streamline the programming process for students of all skill levels, fostering rapid development and code reusability across different robots.*
### Breaking changes
- Previous versions of BunyipsLib were not following semantic versioning. This version is the first to do so.
- RoadRunner methods `addNewTrajectory(...)`, `newTrajectory(...)`, and `addTrajectory()`, have been removed and replaced with `makeTrajectory()`.
  - The new method is more flexible and culminates all task and trajectory creation into one method.
  - To create new trajectories for ABOM, simply use `makeTrajectory()` and call `addTask()` to represent what `build()` used to do
  - Alternatively, you can call `buildTask()` to return a task object without adding it to the task list (for grouping or manual allocation).
  - To add pre-built trajectories as tasks, you may use `makeTrajectory().runSequence()`, allowing you to have access to task properties before building.
