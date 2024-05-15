# BunyipsLib Changelog
###### BunyipsLib releases are made whenever a snapshot of the repository is taken following new features/patches that are confirmed to work.<br>All archived (removed) BunyipsLib code can be found [here](https://github.com/Murray-Bridge-Bunyips/BunyipsFTC/tree/devid-heath/TeamCode/Archived/common).

## v3.0.0 (2024-05-15)
New utilities, autonomous task allocation improvements, API surface simplification, and vision integrity improvements.
### Breaking changes
- `BunyipsOpMode`'s MovingAverageTimer has been reworked to be exposed as a public field `timer`
  - This makes it so like the gamepads and telemetry, the MovingAverageTimer can be accessed in a similar fashion
  - `getMovingAverageTimer()` now is called with `timer`
- Removed various encoder-related systems including `EncoderMotor`, `PivotMotor`, `ScopedEncoder`
  - These systems span from the pre-RoadRunner era of BunyipsLib, where we would use this to measure deadwheel distance
  - It has become far too tedious to maintain, as RoadRunner has deprecated these systems, however, we chose to continue maintaining them for other systems such as PivotMotor
  - However, at their core, they simply convert the tick reading from the encoder into human units, and can be delegated to a utility to handle
  - An appropriate util class `EncoderTicks` has been implemented to accommodate the lost functionality from these utility classes
  - This promotes using motor objects only supplied by the SDK, where the intervention of their implementations should only be handled if core functionality is modified
- Removed `Rotator` and integrated these methods into `HoldableActuator`
  - All `Rotator`s should now be exchanged `HoldableActuator`s
  - All methods from `Rotator` that were useful were transferred and reimplemented
  - Rotator was simply a HoldableActuator under the hood, and as HoldableActuator was changing to include newer features such as overcurrent and limit switches Rotator became outdated and code repetition was happening
  - Like the encoder-related systems removed in the above change, Rotator simply converted encoder ticks to human units
  - The methods available in `HoldableActuator` and `EncoderTicks` replace what `Rotator` did
- AutonomousBunyipsOpMode `removeTaskIndex` method has been renamed to `removeTaskAtIndex` to match the new method `addTaskAtIndex`
- Renamed `Scheduler`'s `finishingWhen` task allocation method to `finishingIf`
  - This is to reduce confusion presented by `finishingWhen` where it would be perceived as the task would only finish when this condition was met
  - Documentation has been updated to clarify that `finishingIf` will add an early-finish boolean expression to the task
  - The corresponding `inTimeFinishingWhen` method has also been renamed to `inTimeFinishingIf`
- Removed useless generic parameters for various tasks, including `AlignToAprilTagTask`, `MoveToAprilTagTask`, `AlignToContourTask`, `MoveToContourTask`, and `HolonomicDriveTask`
  - Construction of these tasks now do not have to mention generic parameters, which simply involves removing the `<>` from instantiating these tasks
  - Parameters now pass in `BunyipsSubsystem`s, which will be typecasted to the proper drive systems
- All methods in the `Limit` class now take instances of `RoadRunnerDrive` to get an instance of `DriveConstants`, after a bugfix related to static constraints
  - Instead of passing DriveConstants into Limit methods, you will now have to pass the entire drive
  - This does not apply to the RoadRunner interface `atVelocity` etc. methods
- `MovingAverageTimer` now passes back `Measure<Time>` instead of `double` for access methods
  - This also comes with a few timer methods being renamed, to include `loopTime` or `loop` for clarity
  - Since we are now using WPIUnits, the `averageString()` (not `toString()`) method was removed as it was redundant
  - Previously, a time unit was passed into the main access methods and conversion was done on the spot
  - However, this is not good practice and therefore all calls now return WPIUnits for the user to utilise, especially in the new `timer` field
- Angular acceleration limits from `Limit` was removed, as they don't actually serve any purpose in RoadRunner, but instead are part of the `turnConstraint`
  - This type of angular limit never existed to begin with, and only applies to turn requests in RoadRunner
- `RoadRunner` interface now no longer extends `RoadRunnerDriveInstance` as it has been removed
  - Any implementations that utilised `RoadRunnerDriveInstance` is a breaking change
- Fix `AlignToAprilTagTask` where it was hardcoded to only look for the tag with id `2`
  - The constructor now passes in a targetTag integer, where if set to -1 it will search for any tag
- `SmoothDamp` of `Mathf` has been reworked to use WPIUnits and Reference for velocity storing, and has been tested to work properly
  - This officially integrates SmoothDamp into BunyipsLib, including the creation of `DcMotorDamping`, however, the old array return was removed and the function was reworked
  - Any previous use of `Mathf.smoothDamp` will need to be rewritten
  - SmoothDamp allows variables to be gradually incremented, effectively giving a "velocity" to a value that may be updated over time
### Non-breaking changes
- `HoldableActuator` displays more telemetry regarding revolutions per second
- `HoldableActuator` now has software bounding limits, much like the old `Rotator` angle limits
- `HoldableActuator` now auto-cancels tasks if they move over the software bounding limits
- `HoldableActuator` now has minimum and maximum power clamps that will be applied to the motor
- `GetTriPositionContourTask` now have a fallback timer, where it will reset to `LEFT` after not seeing any of the appropriate contours for a set amount of time
- Various nullability annotations added throughout the codebase to indicate null handling (`@Nullable`, `@NonNull`)
- Added telemetry of the currently detected `GetTriPositionContourTask` position while the task is running
- Patched `AlignToAprilTagTask`'s bearing unboxing nullability problem
### Bug fixes
- RoadRunner constraints were made default instead of static, which was a bug that applied Mecanum constraints to non-Mecanum robots
  - The non-Mecanum RoadRunner code has not been tested by our team due to time constraints, tread with care
- Fixed a missing parameter in a logd call in `BunyipsOpMode`
- `RobotConfig` will no longer swallow `onSuccess` callback exceptions
  - Will be handled like a standard handled exception as DS messages were being suppressed from exceptions raised in the callback
- Switched `AlignToAprilTagTask` from using yaw to bearing, which was previously the incorrect unit
### Additions
- MovingAverageTimer now has a `deltaTime()` method, which will return the unfiltered raw duration since the last `update()`
  - Useful in precise applications where you require the usage of instant precise loop timings
- New CENTERSTAGE-specific `SpikeMarkBackdropId` utility under `vision/processors/centerstage`, which given a spike mark direction and starting position will return the tag ID of the backboard location for bonus points
- `DcMotorRamping` motor wrapper, which will apply a `Mathf.smoothDamp` on the `setPower()` method
  - This allows motors to be ramped up and down smoothly, and includes additional methods to configure and override this behaviour
  - `RobotConfig` will try to automatically instantiate this based on the `getHardware()` class parameter as well
  - As such, DcMotorRamping can be treated the exact same as a DcMotor in terms of creation, and goes in your config
  - DcMotorRamping implements DcMotor, and can be down and upcasted as the only method overridden is `setPower()`
- `EncoderTicks` utility, which allows conversion between encoder ticks, angle, and distance using WPIUnits
- AutonomousBunyipsOpMode `addTaskAtIndex`, which allows the insertion of tasks at any given index in the task queue
  - While methods have been implemented to try and stop race conditions, it is recommended that any asynchronous operations with this method are minimised
- RoadRunner turnConstraint methods `setAngVel` and `setAngAccel` (and reset methods) which are applied when rotating the robot in place

## v2.0.1 (2024-05-04)
Rotator exception patches and telemetry improvement.
### Bug fixes
- Fixed misuse of infinities with default lower and upper limits, replaced with `-Double.MAX_VALUE` and `Double.MAX_VALUE` respectively
### Non-breaking changes
- Hide infinite limits and replace with infinity symbols in telemetry for `Rotator`

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
