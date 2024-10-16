# BunyipsLib Changelog
###### BunyipsLib releases are made whenever a snapshot of the repository is taken following new features/patches that are confirmed to work.<br>All archived (removed) BunyipsLib code can be found [here](https://github.com/Murray-Bridge-Bunyips/BunyipsFTC/tree/devid-heath/TeamCode/Archived/common).

## v5.1.1 (2024-10-16)
Hotfixes for AprilTagPoseEstimator.
### Bug fixes
- Kalman filter heading outputs for AprilTagPoseEstimator are being stabilised with a low-pass filter as a hotfix
  - This should reduce the random rotation errors when moving while looking at a tag
  - In the upcoming v6.0.0 release, details below, ATPE will potentially be upgraded into its own localizer rather than its own component
    - This upgrade is subject to change for the final release
- Fixed broken rotation matrices for AprilTagPoseEstimator
  - The yaw of the tag was not being correctly used and only caused localization to be accurate if the robot was square with the tag
  - This has been resolved
- AprilTagPoseEstimator now uses an average if multiple tags are detected to improve accuracy
### Upcoming changes
- BunyipsLib v6.0.0
  - Major restructure of files, more files will be in appropriate folders and the namespace will be updated
  - RoadRunner will be upgraded from v0.5.6 to v1.0.0, with many of the drive classes being refactored and vastly simplified
  - Removal of over 30 files (4000 L.O.C)
  - Various other features and removals to improve and simplify BunyipsLib development

## v5.1.0 (2024-10-12)
Several quality-of-life adjustments, bug fixes, and features.
### Critical bug fixes
- PhotonFTC has been removed as a default required dependency to BunyipsLib
  - Due to the state of Photon in alpha, it has been decided for stability to instead define the Photon annotations at the user OpMode level
  - The BunyipsLib documentation still suggests installing Photon in the deps, but has been removed from `BunyipsOpMode` as it is the base to all OpModes
  - Since Photon is a one-line addition, this is more sustainable for the time being while stability of Photon improves
  - This reverts the integration of Photon to <5.0.0
-  The `Motor` class has been rewritten following a critical bug related to `RUN_TO_POSITION`
  - This bug would cause motors to sporadically run away from the setpoint when moving in a backward direction
    - The bug has been identified to exist since BunyipsLib v4.0.0
  - Previously, the implementation of `Motor` was that it extended `DcMotorImplEx`; however, an internal method which is responsible for setting power was calling `getMode()`, which related to the overridden version in `Motor`
  - Note that `Motor` forces the actual motor to always run in `RUN_WITHOUT_ENCODER` mode as the hub-level PID controllers should always be disabled
  - This caused all `setPower` calls to be positive when `RUN_TO_POSITION` was active, as internally the power is taken for an absolute value in the standard `RUN_TO_POSITION` mode
  - Unfortunately, this would cause all system controllers on `RUN_TO_POSITION` to respond as normal, but just before the command was sent to the motors it was taken for an absolute value, making the controller physically incapable of moving the motor backwards
  - This bug has been fixed by instead implementing the `DcMotorEx` interface and handling the motor controller manually
  - As a side effect, the `Motor` class now supports the usage of all `DcMotorEx` methods in some proportion
### Non-breaking changes
- `BunyipsComponent` no longer needs to be running in the context of a `BunyipsOpMode`
  - This makes components that extend `BunyipsComponent` simply a utility instead of a strict requirement
  - Opens up subsystems, tasks, and all other components to not just `BunyipsOpMode` contexts, oftentimes when these components didn't need a reference to the OpMode anyways
  - Previous implementations of BunyipsComponent are unaffected, however, the  `opMode` field has been made `@Nullable` which may throw some compiler warnings (as it is now possible for components to exist such that `opMode` is going to be null)
  - The new `BunyipsComponent` utilities now provide ways to assert behaviour and run code only on `BunyipsOpMode` contexts, such as `require(opMode)` and `opMode(o -> o)` to assert presence and no-op calls
  - Any custom components may need to be migrated with these new utilities to ensure the compiler does not throw errors
  - The integrated subsystems have already been migrated and no changes are needed in user space
- `BunyipsOpMode` active-loop runnables are now part of a `HashSet` rather than an `ArrayList` to prevent duplicates
- Several components including `DebugMode`, `AprilTagPoseEstimator`, and new runnable components now auto-attach to the `BunyipsOpMode` runnables list if they can on construction
  - All runnable components also have the convention of a static method called `enable()` for concise construction
- `Mathf` and `EncoderTicks` utilities now take in the `Number` supertype when taking in arguments
  - This removes the dual `double`/`float` methods of `Mathf`, and reduces the number of casts that need to be done when working with `EncoderTicks` that are sometimes in `int` or `double` forms
  - Internally, these `Number` supertypes are all converted to `double`s
- The `Switch` class has been disconnected from the `Cannon` class
  - This may cause some unknown method calls if using a `Cannon` method from a previous `Switch` class on migration
- `Task` composition has been made simpler
  - `AutonomousBunyipsOpMode` now returns the added task instance when calling `addTask()`
    - `RoadRunner` does the same for `addTask()`
  - Tasks can now compose with utility functions attached directly to `Task`
    - `.until` which composes the current task with a `RaceTaskGroup` and a `WaitUntilTask`
    - `.after` which composes a task with the current task in a `SequentialTaskGroup`
    - `.then` which composes the current task with a task in a `SequentialTaskGroup`
    - `.with` which composes a `ParallelTaskGroup` with all the tasks
    - `.race` which composes a `RaceTaskGroup` with all the tasks
    - `.during` which composes a `DeadlineTaskGroup` with the current task being the deadline
    - `.repeatedly` which wraps the current task in a `RepeatTask`
  - These integrations make task construction more concise without having to construct groups manually for small operations
- The `ColourBlob` vision data type from the `ColourLocator` processor can now be converted into `ContourData` instances
  - This allows tasks like `AlignToContourTask` and `MoveToContourTask` to be used with the new SDK 10.1 processors
  - To convert all list processor data into a `ContourData` list, a simple map operation can be performed as listed in the documentation for this method
  - The corresponding contour-based tasks now have alternate constructors for taking in instances of `Supplier<List<ContourData>>` rather than specifying a strict requirement on `Processor<ContourData>`
- `PIDFController` changes
  - Following conventions throughout BunyipsLib, `PIDFController` (and derivatives) now have builder-like patterns to allow for more streamlined construction
  - `PIDFController` also now has a `setOutputClamps` method to control the maximum and minimum results the controller may output
  - Variants `ArmController` and `PIDFFController` now support instances of the base `PIDF` to be used to allow profiled PID controllers instead of the hardcoded normal PID
- `Encoder` now pipes acceleration estimation data through a built-in low-pass filter that can be modified via `setAccelLowPassGain(double)`
- All previous tasks and components that relied on instance of `RoadRunnerDrive` have been abstracted via the new `Moveable` interface
  - This changes the abstraction level of various tasks and components to be as minimal as possible - RoadRunner is no longer required to use most of the drivebase features in BunyipsLib
  - Some classes that take in instances of `BunyipsSubsystem` for their drive base have been migrated to `Moveable`, note that auto-attaching the task to a subsystem is still performed internally
- `Reference` now implements `Supplier` and `Consumer`
- `DualServos` and `Cannon` no longer auto-calls `update()` on init
  - This is to consolidate the idea that no hardware writes will occur outside of the user's command, especially as penalties exist for servo movements during the match init-phase
  - If you wish to preserve the old behaviour, simply call `.update()` in your init method
### Bug fixes
- Acceleration information from `Encoder` can no longer be stale and is now updated on every loop with epsilon checks
- `RoadRunner` `withTimeout` builder parameter now sets the timeout to infinite if a negative timeout is supplied
- Various outdated docs updates
- `addTask` of `RoadRunner` now checks for an active instance of `AutonomousBunyipsOpMode` to throw an exception early if it is not initialised
- `disableHardwareStopOnFinish()` of `AutonomousBunyipsOpMode` is now a final method
- `Motor` now clamps the `RUN_TO_POSITION` system controller output to -1 and 1 to allow the `setPower` argument to have a direct proportion over the controller
- `Encoder` now flips the velocity readings depending on the direction, before this would only flip the current position
- `HoldableActuator` is no longer affected by the user-defined min and max encoder limits when homing
- The `BlinkinLights` internal `setPattern` call is now cached to prevent spamming Logcat or calling unnecessary code
  - Do be aware of this if you manually call `setPattern` on the `RevBlinkinLedDriver`
- `HoldableActuator` update method is now more conservative with hardware calls
  - Hardware calls are executed at the end of the update method, which prevents multiple calls to `setTargetPosition()` and `setMode()`
  - The motor power magnitude has also been updated to the proper sign depending on the signum of error to allow bounds checking to work properly
- Various optimisations for `AprilTagPoseEstimator` to reduce jank
  - The component now does not spam Logcat as much when seeing an AprilTag to adjust pose
  - Several modulus operations for heading have been added to reduce problems
### Additions
- Pure Pursuit implementation using a parametric look-ahead
  - Offered as a new component called `PurePursuit` which can generate paths similar to RoadRunner to follow
  - Uses a parametric look-ahead based on the supplied path instead of a circle-line intersection for simplicity and more precise cornering
  - This algorithm is a step up from PID-To-Point (DriveToPoseTask) and below RoadRunner
  - The PurePursuit class has been built around the RoadRunner coordinate systems and should work well with currently implemented tools
- New HardwareDevice drop-in replacement `ProfiledServo` to allow motion-profiled servos
  - Similar to the `Motor` class wrapping a `DcMotor`, the `ProfiledServo` class wraps a `Servo`
  - `ProfiledServo` exposes a new method `setConstraints` which can be used to define a `TrapezoidalProfile` that will be used to motion profile the servo
  - This allows velocity and acceleration control over the servo, while still conforming to the `Servo` interface for use in any previous code
  - Additional features from `Motor` including position cache tolerance and refresh rate have also been integrated in the methods `setPositionDeltaThreshold` and `setPositionRefreshRate`
  - Review the `ProfiledServo` class for more information
- The `Moveable` interface, which is a common interface for all drive bases
  - This interface allows drives such as the `CartesianMecanumDrive` to be used in place of RoadRunner drives, often when the features of RoadRunner don't need to be used
  - Exposes two methods, `@Nullable getLocalizer()` and `setPower(Pose2d)`, which respect the Robot Coordinate System and are implemented by `RoadRunnerDrive` and `CartesianMecanumDrive`
  - By moving to these interfaces, component flexibility is improved dramatically where localizers can be used without the need for RoadRunner
  - `CartesianMecanumDrive` can support holding a `Localizer` through `setLocalizer()`, which may be required if a drive instance is passed around as a `Moveable`
- Ported `MecanumLocalizer` and `TankLocalizer` from RoadRunner
  - These localizers have no dependencies on the RoadRunner drives, which allows pure functional suppliers to be used for localization
  - This makes the `Localizer` from RoadRunner a common interface on all drives
- `EncoderTicks` now has a utility function to convert ticks directly to inches, which is used in the new ported Localizers
  - This converts a tick reading, wheel radius in inches, gear ratio, and ticks per revolution into inches for use with the Localizers
- New `TurnTask` which is similar to `DriveToPoseTask` but only requires one PID controller for turning, and does not have a strict dependency on a `RoadRunnerDrive`
  - Supports either global robot-frame heading targets or heading offsets
- New `Rect` class which represents two `Vector2d` instances to create an upright rectangle
  - Based on RoadRunner geometry classes, which allows more advanced uses of vectors
- New `Field` utility class for FTC fields
  - Provides a `Rect` which defines the maximum boundary of the field
  - `Season` enum that has methods `getRestrictedAreas()` and `getAprilTagLibrary()`
    - The INTO THE DEEP season has been included in this enum and denotes the restrictions of the submersible zone and supporting poles, as well as fetching the relevant AprilTag library from the SDK
- New `BoundedLocalization` runnable component to provide clamping of pose estimates to within the field
  - By default, pose estimates can often be in illegal positions, and for odometry that cannot determine these states causes the robot to leave the field
  - This component runs in the background and ensures the pose is in legal areas, and can be customised with `Rect`s to define spaces the robot cannot go in, including utilities from `Field` for built-in restricted areas
- `BunyipsOpMode` now has a method to remove runnables from the active-loop runnables list `detachActiveLoopRunnables`
- `TankDrive` (and the base RoadRunner equivalent) now have a `getCoefficients()` method for `TankCoefficients` similar to `MecanumDrive` for `MecanumCoefficients`
- `Encoder` now can track a motor direction as set by a functional interface, which allows the state of the `Encoder` to be controlled by an encapsulating component
- `Switch` subsystem has been revamped
  - Disconnected from `Cannon`, `Switch` is now able to represent positions that are not limited to simply two positions
  - This makes `Switch` the single counterpart to `DualServos`, but with more control on bounds with utility methods and toggles
  - Review the new `Switch` class for more information
- New `Mathf` utilities
  - `approximatelyEquals` which uses an epsilon of `1e-6` to check if two decimals are equal
  - `lineCircleIntersection` and `lineSegmentCircleIntersection` for calculating these intersection points using the quadratic formula
- `HoldableActuator` new `ceil()` task, which runs a home task in the opposite direction
- `ThreeWheelLocalizer` instances can now opt into using a periodic IMU reset, which will intermittently use the IMU to relocalise the robot heading
  - This is useful for correcting drift while not impacting loop times significantly due to I2C reads
  - To use this feature, an IMU must be passed via `ThreeWheelLocalizer.withRelocalizingIMU(IMU)`, and the `ThreeWheelLocalizer.Coefficients` must define a non-null `setIMURelocalizationInterval(Measure<Time>)`
  - `TriDeadwheelMecanumDrive` internally calls `withRelocalizingIMU(IMU)` with the IMU parameter
- `HoldableActuator` now has an optional mode where user input modes will instead follow the setpoint instead of feeding raw power
  - This can be enabled with `enableUserSetpointControl(DoubleSupplier)`, where the supplier is a multiplicative scale to use in setpoint adjustment (for things such as deltaTime or other math)
  - This mode will instead adjust the setpoint when a user uses a `setPower()` call, allowing the PID controller to do all the work
  - It is recommended to use this mode only if the motor used in the `HoldableActuator` is a `Motor` instance, as the stock motor may react too slow
- New `IMULocalizer`, which implements `Localizer` and can be used in place of a Localizer to simply provide heading information
  - This is useful for new tasks that may only need heading information (e.g. `TurnTask`) and a main localizer is not implemented
- `DashboardUtil` has a new `useCanvas(Consumer<Canvas>)` method which will attempt to retrieve a reference to the FtcDashboard canvas via `BunyipsOpMode`, or by creating a new packet under the hood and sending it on completion
  - This is useful for components to draw on the dashboard from any static context without having to check OpMode state
- `AprilTagData` now exposes a `toRect()` method to convert the bounding box to a `Rect`
  - This Rect can be used to construct a `ContourData` or other information as desired
- `AprilTagPoseEstimator` now supports filters
  - Can be added via `addDataFilter(Predicate<AprilTagData>)` and will run the `Predicate` on every detected AprilTag's `AprilTagData`
  - This can be used in combination with `toRect()` to filter results, such as filtering by area (for example, `.addDataFilter(t -> t.toRect().area() > 1000)`)

## v5.0.0 (2024-09-24)
Integration of SDK v10.1 and season-related features.
### Breaking changes
- Trajectory mirroring of the `RoadRunner` utility class has been revamped by an enum
  - Instead of passing a simple boolean state to reflect across the alliance plane, options now exist to mirror symmetrically
  - The new `MirrorMap` types are now `NONE`, `ALLIANCE_REFLECTION` (default), and `SYMMETRIC_MIRROR`
  - The symmetric mirror profile was created as the INTO THE DEEP field is symmetric and using global reference frames is important for AprilTag repositioning
  - All old instances of pose mirroring contexts have been replaced with taking in a parameter of type `MirrorMap`
- PhotonFTC (v3.0.2-ALPHA) by Eeshwar has been integrated as an included library
  - Teams may choose not to use Photon by removing the annotation from `BunyipsOpMode` and/or not including the library
  - Reread and follow the wiki installation instructions to bring in the new integrations, as well as making sure the versions match (for other lib. updates)
- `Processor` instances no longer take a copy of the frame before sending it to the processing methods
  - For most cases, this should not change anything; this change was needed to be made for the new SDK v10.1 processors
  - There should now be less memory overhead for vision processing as a side effect
  - This is listed as a breaking change as it may affect certain processors
- Fixed misnamed processor thresholds for all CENTERSTAGE Pixel colour processors, the thresholds were called YCBCR instead of YCRCB
### Non-breaking changes
- Various docs updates and migration to the wiki
- `DualTelemetry` now displays the appropriate unit of `Hz` instead of `l/s` once the loop times are too fast to be represented as milliseconds
- The slow loop speed warnings of `DualTelemetry` now don't show up during the init-phase when used in a `LinearOpMode` derivative
  - The init-phase is the heavy processing of the OpMode and loop times don't matter during init
- `Processor.getCameraDimensions()` is now a public method
- `Ramping.DcMotor` now extends `DcMotorImplEx` instead of `DcMotorImpl`
### Bug fixes
- `RoadRunnerDrive` watchdog timeouts no longer log in warnings but instead debug logs
- Fixed the separator in `TelemetryMenu`, there is now a gray bar at the start of every menu entry for ease of use on FtcDashboard
- `RoadRunnerDrive` instances now cache their run mode during the constant `setMode()` calls in the update loop
  - This reduces redundant invocations of `setMode()`
  - It is important to only dynamically modify the drivetrain run mode manually through the `RoadRunnerDrive` instance instead of the motors themselves, due to the caching
### Additions
- Update to SDK v10.1; the new recommended SDK version is now v10.1
  - New `ColourLocator` processor that wraps a `ColorBlobLocatorProcessor`
  - New `ColourSensor` processor that wraps a `PredominantColorProcessor`
  - Appropriate data classes with filtering methods are `ColourBlob` and `ColourSample`, appropriately
  - Since these processors are instance-based instead of abstract, their IDs are represented by `coloursensor0`, `coloursensor1`, etc. and `colourlocator0`, `colourlocator1` etc. depending on the number of instances
    - These IDs are important to know for FtcDashboard switching or processor differentiation
  - The BunyipsLib integration of these processors makes constant FtcDashboard viewing available and reduces unnecessary canvas overhead
  - These new processors work the same as the pre-existing `ColourThreshold` and `AprilTag` processors
  - The legacy `ColourThreshold` and `MultiColourThreshold` will not be deprecated
### Notes
- RoadRunner v1.0.0 is not planned for integration
  - This is because of the ties RoadRunner v0.5.6 has with the path generator, visualiser, and utilities that don't bring any significant value to BunyipsLib upon migration
  - Command-based features of RoadRunner v1.0.0 are already implemented as part of the `Scheduler` and `Task` system
  - Time is better spent for BunyipsLib development working on the wiki and new features rather than refactoring old but still useful systems
- `StartingConfigurations` assumes a square field, where the blue alliance is opposite the red alliance
  - In the event a future season does have a diamond field (it has not been the case for a while), then an integration may be made to support this
  - It is not of priority now or for this season, so it is not scheduled for implementation and won't be reconsidered until next season

## v4.1.0 (2024-09-16)
New features and additions for INTO THE DEEP.
### Critical bug fixes
- The `FieldTiles` (`FieldTile`) custom WPIUnits unit has been updated from being defined as 23.6 inches to 24 inches
  - This change may potentially break previous implementations that relied on the nature of `FieldTile` being 23.6 inches
  - This unit was supposed to have a magnitude of 24, where the approximation was used by mistake
- `TaskGroup` timeout calculation has been fixed
  - Previously, task group timeout calculations were done *all* via the parallel calculation, which takes the highest value and sets it to the group timeout
  - However, this was short-sighted as other task groups like the sequential task group instead operate on a sum-of-all-timeouts
  - These incorrectly implemented timeouts have been fixed appropriately for all task groups, via a new super argument for the `TaskGroup` abstraction
- `Scheduler` `runOnce()` now behaves uniformly between subsystems and independent tasks
  - Before, `runOnce()` would queue once on a subsystem, and execute once independently
  - This made inconsistent behaviour between the two due to an internal flaw in the `Scheduler`
  - Now, `runOnce()` will guarantee one queue of a task from start to finish regardless of environment, putting emphasis on `finishingIf()` to end queued execution
- `Exceptions` no longer swallows the `ForceStopException` (oops)
### Non-breaking changes
- `AprilTagPoseEstimator` now internally uses Kalman filters to fuse odometry and AprilTag readings
  - The Kalman gains are set to sane defaults and can be customised via `setKalmanGains`
  - Note! `setHeadingEstimate` now defaults to **true**, as the Kalman filter should now filter out the erratic heading readings
- `BunyipsComponent` and their derivatives (e.g. `BunyipsSubsystem`, `Task`) can now be constructed in the OpMode member fields
  - A constructor hook now supplies a partially constructed static reference to a BunyipsOpMode to allow instance calls to be made in the member fields
  - This partially constructed static instance is reassigned as usual at runtime to preserve normal behaviour
- The `toShortString()` (`toString()`) method attached to WPIUnits has been updated
  - Previously `Unit` instances that called `toString()` would provide the string with 3 decimal point scientific notation
  - This has been changed to simply show the magnitude in full for brevity
  - Instead of returning `1.234e+04 V/m`, this method will now return `1234 V/m`
  - This combines `toLongString()` without the full unit name
  - Previous scientific notation behaviour has been preserved in a new `toScientificString()` method
- `BunyipsOpMode` now exposes the previously protected methods `onActiveLoop` and `setInitTask` as public to allow static access
  - This allows you to add `activeLoop` snippets from anywhere, including the `RobotConfig`
- `toString()` of `StartingPositions` instances now internally call `StartingPositions.getHTMLIfAvailable()` to return an HTML-receiver-friendly output without having to call `getHTMLIfAvailable()` manually
  - The built-in `name()` method preserves the old behaviour of `toString()`
- FtcDashboard now shows more overlay data for certain `AlignTo` tasks, including vector powers for drive input
- Various JavaDoc and debugging statement updates
- `OnceTask` internal timeout has been increased to 10ms to allow interpretation of summed timeouts to work properly
- `BunyipsOpMode` `onActiveLoop` now runs all Runnables through the `Exceptions` handler to ensure exceptions do not hamper the rest of the loop
- Various uses of `PIDFController` in constructors have been replaced with the more relaxed `PIDF` interface to allow for a wider range of controllers
  - Note that since the coefficients are no longer constant, dynamically adjustable coefficients that were on the task/class level only apply to the underlying PIDF coefficients as they did previously
  - Other dynamically adjusted coefficients will need to be managed by yourself
- `BlinkinLights` now has methods for setting the default pattern to other patterns if desired, it is no longer final
  - To follow subsystem convention, `BlinkinLights` methods now return their instance for builder-like patterns
- `Task.reset()` now no-ops if the task has already been reset, preventing the multi-fire of `onReset()`
- `BunyipsOpMode` will now try to reset the RC lights via `exit()` if it can, since it still has access to hardware writes
- `TelemetryMenu` now spaces options slightly more to allow FtcDashboard users to read the menu easier
### Bug fixes
- Hardcoded values of `org.murraybridgebunyips.bunyipslib` have been replaced with the `BuildConfig.LIBRARY_PACKAGE_NAME` constant for cleanliness
- Modern uses of out-of-range exceptions thrown in BunyipsLib now respect the domain of the bound
  - Future bounds-checking now uses Apache Math3 utilities, old checks will be left as-is
- Debugging statement for the `onReady()` call in `AutonomousBunyipsOpMode` now strips HTML from the selected OpMode
- `setDefaultTask` of `BunyipsSubsystem` now internally calls `onSubsystem(this)` to ensure default tasks are assigned to the current subsystem
- The `PIDF` interface now extends `SystemController` as they are always used together
- `BunyipsOpMode` no longer initially sleeps before evaluating the init-loop for the first time
- Exceptions thrown via `Exceptions` during init will now be recognised and alerted in the `BunyipsOpMode` init-cycle
- `DualTelemetry.addData` now accepts nullable data parameters to match other instances of adding telemetry data
- Fix a missing return from an early return in `UserSelection`
### Additions
- New `StartingConfiguration` system to assist in robot starting configuration selection
  - This class has been designed to expand the `StartingPositions` enum following the 2024 season not being limited to simply two tiles per alliance
  - Instances of a `StartingConfiguration.Position` indicate the exact orientation/rotation/alliance/origin that the programmer has desired, rather than simply being on an alliance and left/right side
  - Construction of a starting position is done through a builder pattern (e.g. `blueRight().tile(4)`) with options for translation offset and rotation
  - These new configurations are designed to be used seamlessly with an `AutonomousBunyipsOpMode` through the `setOpModes()` method
    - The programmer now has a lot more information regarding the starting configuration of the robot, including a `toFieldPose()` method to extract the starting configuration in terms of the FTC Field Coordinate system
  - Position `toString()` method will return a HTML human-friendly output (for `UserSelection`)
    - `Red Alliance, Right` from StartingPositions becomes `On Red, Tile #3 from RIGHT wall` in a StartingConfiguration
  - Documentation and conversion methods have been added to both StartingPositions and the new class
  - StartingPositions is not planned to be deprecated to preserve backward compatibility
  - See the corresponding JavaDoc for more information
- New `Filter` class, which provides data and sensor filters
  - The newly added filters available through this class are a `LowPass` filter, 1D `Kalman` filter, and `WeightedFusion` filter
  - The `Kalman` filter is used internally in the `AprilTagPoseEstimator`
- New `AlignToPointDriveTask`,  which uses a PID controller and feedforward to rotate to a field point as a TeleOp drive task
- New `DebugMode`, which is a Runnable built via a builder pattern to auto-halt or terminate OpModes based on various conditions
  - This includes watchdog timeouts, gamepad kill switches, and IMU roll detection
  - These features were designed for use in a debugging environment to quickly stop the robot if working with experimental code
- `Motor` now has configuration options for setting a power cache tolerance and power refresh rate
  - These utilities allow hardware writes to be optimised, but must be used with caution
  - See `setPowerDeltaThreshold` and `setPowerRefreshRate` of `Motor`
- `AprilTagData` is now populated with a new field `robotPose` which is new from SDK 10.0
  - This pose is returned directly from the processor and is calculated via the camera-robot pose provided in the `AprilTag` builder
- All `RoadRunnerDrive` instances now must include a `setRotationPriorityWeightedDrivePower()` override to run the weighted drive power calculation with rotational priority
  - Custom implementations of `RoadRunnerDrive` will need to extend this new method and implement it accordingly
  - `CartesianMecanumDrive` now exposes an internal static method to calculate rotation priority Mecanum poses
- `IMUEx` now has a `setYawMultiplier()` method, which can be used to set a multiplicative scale for IMU yaw readings
  - This is similar to the X and Y multipliers attached to tracking wheel localizers
- `BunyipsOpMode` now has a `stopMotors()` method which serves as a softer alternative to `safeHaltHardware()`
- `Exceptions` now stores a static list of all exceptions that have been thrown
  - The reset method for this instance has been appended to `Storage.resetAllStaticFieldsForOpMode()`
- `BunyipsOpMode` now has a static method `ifRunning()` which will execute the supplied `Consumer<BunyipsOpMode>` if `BunyipsOpMode.isRunning()` is true
- `BunyipsOpMode` now has a `getInitTask()` method to return an `Optional<Runnable>` of the currently respected init-task
- `Text.removeHtml()` created to regex out HTML tags and non-breaking spaces from strings
- `Text.upper()` and `.lower()` created for cross-compatibility reasons in Kotlin, which will uppercase and lowercase strings

## v4.0.0 (2024-09-08)
Major INTO THE DEEP season release.
### Breaking changes
<i>Note! BunyipsLib v4.0.0 has many breaking changes that consolidate the codebase; please review every item carefully if migrating.</i>
- SDK v10.0 related deprecations
  - The BunyipsLib SDK version has been bumped from v9.2 to v10.0 for usability in the INTO THE DEEP season
  - TFOD-related subsystems have been archived following their removal in the SDK
    - This includes deprecated in v3.5.0 classes `TFOD` and `TfodData`
    - The SDK no longer offers TFOD; future vision integrations are expected to use OpenCV, which is integrated into BunyipsLib through the `Processor` pipelines.
- BunyipsLib removal of v3.5.0 deprecations
  - Previously deprecated methods have been removed, which include `BunyipsOpMode` methods such as `addTelemetry()`, `log()`, and other telemetry-related methods.
    - To access telemetry, use the standard `telemetry` object which integrates as a `DualTelemetry` object. All methods that were removed from `BunyipsOpMode` is equivalently available through the `telemetry` object
  - `DualTelemetry.removeRetained()` (alias for `remove()`) has been removed
  - `Scheduler.ConditionalTask.runDebounced()` (alias for `runOnce()`) has been removed
- The subsystem convention for `Task` exposure has been updated
  - *All tasks* exposed on `BunyipsSubsystem`s, including the built-in defaults now expose their tasks as part of an inner class with a public final field called `tasks`
  - This was to separate tasks from standard operations instead of having tasks be intermixed between operations
  - An example of the new system is the `HoldableActuator.homeTask()` being refactored to `HoldableActuator.tasks.home()`
  - Future BunyipsLib subsystems will continue to follow this standard
- Examples directory removed
  - To be replaced by the Wiki at https://github.com/Murray-Bridge-Bunyips/BunyipsLib/wiki
- `CameraType` implementation and classes removed
  - Camera intrinsics should be defined via the SDK's built-in tools, such as the teamwebcamcalibrations.xml file or by directly injecting camera intrinsics using the new construction patterns
  - This extends to the `AprilTag` processor, which is the primary use case for these intrinsics
- `RampingValue`, `RampingFunction`, `RampingSupplier`, and `DcMotorRamping` have all been compressed into a single interface for brevity
  - New interface name is `Ramping`, and is the equivalent of `RampingFunction` with implementations for `Ramping.Value`, `Ramping.Supplier`, and `Ramping.DcMotor`
  - The `setMaxDelta` method has also been renamed to `setMaxRampingDelta`
  - `Ramping.DcMotor` now only extends `DcMotor`, instead of extending `DcMotorEx`
- Vision systems updated to support multi-camera configurations
  - This includes `Processor` now carrying a copy of the camera dimensions, as multiple cameras may use different resolutions
  - Static camera dimensions now work based on a default value, unless defined by the user on construction of `Vision`. These dimensions are carried with the Vision instance for all processors to carry a copy of these dimensions
  - To access camera dimensions from a processor, instead of accessing the static constant from `Vision`, access `getCameraDimensions()`. Note that this will be null if this processor is not attached to a Vision instance (and therefore is not streaming on a camera so we can't get dimensions)
- `addSubsystems()` of `CommandBunyipsOpMode` and `AutonomousBunyipsOpMode` has been removed
  - Subsystems are now implicitly registered through a static array, therefore if you construct a subsystem, it will be added automatically
  - If you wish to use a special set of subsystems, you can opt to using the `useSubsystems()` method
- Integrated methods for Field-Centric driving have been removed
  - These systems were inconsistent between each drive in BunyipsLib, and have been removed
  - Drive tasks such as `HolonomicDriveTask` allow for a supplier to indicate when field-centric drive should be enabled, which will continue to work by doing the calculations internally
  - Other implementations of field-centric drives are left as an exercise for the user
- `Task` system was reorganised for brevity
  - This includes the removal of abstract extensions of `Task` such as the `NoTimeoutTask`, where this can simply be done now by extending `Task` with no timeout parameter
  - The `RobotTask` interface was removed as this does not have usefulness with the evolved `Task` system
    - Frequently this lead to classes that did support `RobotTask` to perform instance checks and cast to `Task`, where this degraded code cleanliness for no reason
    - Tasks now extend the built-in `Runnable`, so previous implementations that used `RobotTask` only now are represented by the default type of Runnable
    - `BunyipsOpMode` now supports taking in a `Runnable` to use as the init-task which will run indefinitely during init unless it is a `Task` that has reported as finished
  - These reorganisations emphasise the idea that a `Task` is simply a `Runnable` with a timeout and finish condition
- Task subsystem dependencies have been reorganised to use a method `onSubsystem()`
  - This replaces the need for tasks to make both non-subsystem and subsystem dependant tasks, and now exists as a builder-like method
  - Parameter argument orders remain consistent with the (subsystem, override) signature
  - Tasks to run on subsystems should now be constructed like so: `new MyTask(...).onSubsystem(subsystem, false)` instead of passing args directly into the constructor
  - Consequently, all BunyipsLib tasks have been simplified not to have dual (dependant/non-dependant) constructors
- `AprilTagPoseEstimator` now implements `Runnable`, the `update()` method is now `run()` of Runnable
- Major refactor of RoadRunner tuning processes
  - RoadRunner tuning has been inconsistent since the initial addition of it into BunyipsLib
  - This system is mixed with the new `RoadRunnerTuningOpMode` class, see below in Additions
  - All RoadRunner tuning OpModes themselves are now TriConsumers that can be run on demand, but it is recommended to use the dynamic `RoadRunnerTuningOpMode`
  - Old code that would extend these OpModes should resort to extending the singular `RoadRunnerTuningOpMode`
- Removed `DEFAULT_TIMEOUT` field from `RoadRunner` interface (Use `Task.INFINITE_TIMEOUT`)
  - `RoadRunnerTask` will still continue to have a task timeout equal to the trajectory duration by default
- The `RoadRunner` interface now consolidates mirrored poses when using `makeTrajectory()`
  - Pose refs that are mirrored now use a parameter on the constructor if this pose should be mirrored in a ref
  - This was created as pose refs used in the constructor are often global pose estimates that should not be flipped
  - Note that builders will not use a mirrored pose ref for the constructor argument to `makeTrajectory()`
  - Review the documentation regarding these behaviours via the JavaDoc to ensure compatibility
- The `IMUOp` subsystem has been redesigned from the ground-up as a `HardwareDevice` replacement
  - The new replacement is called `IMUEx`
  - The old `IMUOp` did not serve much purpose as it would wrap values that could be obtained easily from the universal IMU
  - The new class now focuses on implementing the universal `IMU` interface to continually retrieve values and update public volatile fields
  - This can be integrated with subsystem-like integrations such as multithreading
  - `IMUEx` now comes with a Yaw Domain that can be set, to allow readings of heading to be in one of three modes:
    - Signed (default, -180 degrees to +180 degrees exclusive)
    - Unsigned (0 degrees to 360 degrees exclusive)
    - Unrestricted (-inf to +inf)
    - Note these domains only apply to the public fields, the internal `IMU` interface methods that expect signed values will use signed values
  - These field units are supplied as WPIUnits, to simplify unit conversion and to integrate nicely with other BunyipsLib components
  - `IMUEx` also implements the `IMU` Universal Interface, so it can be used in place of other IMUs and hardware devices at the config level
- `CartesianFieldCentricMecanumDrive` has been redesigned to not rely on another subsystem (old `IMUOp`) and now takes in an instance of the universal `IMU`
  - Construction now should be as simple as passing in the IMU from HardwareMap
- `GetTriPositionContourTask` has been renamed to `GetDualSplitContourTask` which fundamentally restructures the idea of the task
  - Instead of assuming the conditions of where a camera is facing, the new refactored common task will now simply report on which half of the camera the largest contour is (`Direction.LEFT`, `Direction.RIGHT`, or `Direction.ZERO`)
  - A convenience method `getMappedPosition` now exists to provide the legacy functionality by mapping camera position reports to other directions (such as the assumption of position this task originally was for)
- The RoadRunner localizers and coefficients have been renamed and refactored for brevity
  - TwoWheelTrackingLocalizer has been renamed to `TwoWheelLocalizer`
  - TwoWheelTrackingLocalzierCoefficients has been integrated into the inner class `TwoWheelLocalizer.Coefficients`
  - ThreeWheelTrackingLocalizer has been renamed to `ThreeWheelLocalizer`
  - ThreeWheelTrackingLocalzierCoefficients has been integrated into the inner class `ThreeWheelLocalizer.Coefficients`
- `HoldableActuator.tasks.runFor` has been switched to (timeout, power) from (power, timeout) for consistency between other time-based methods throughout BunyipsLib
- Gains for the WPILib-ported feedforward controllers have switched to getters instead of exposing public final fields
- `setPowers()` of `MecanumDrive` has been standardised back to `setMotorPowers()` (new override) as found in the wrapped instance and across all RoadRunner drives for consistency
- Renamed `EncoderTicks.EncoderTickGenerator` to `EncoderTicks.Generator` for brevity
- BunyipsLib RoadRunner drive constructors no longer take in instances of `voltageSensor`, as these can be inferred internally like how `DualTelemetry` is internally injected
  - These constructors are now more concise than they used to be
- Tank RoadRunner drives now no longer conform to the 4-wheel only specification and now support the intended list of left and right motors
  - This was an oversight that was present in the `SampleTankDrive` of the RoadRunner quickstart, but was incorrectly implemented
  - Any 2+ tank wheel configuration can now be used with these tank drives
- Removed `GetSignalTask` for being too specific and useless in the scope of BunyipsLib
- `NullSafety.assertComponentArgs()` constructor has changed instead of accepting classes it will accept a component name, a class name to exclude, and the objects to check, which can now distinguish between differently named subsystems (experimental, actual uses of this util are provided by the integrated subsystem assertions on construction)
- Switched to using WPIUnits for the `While` class timeout
- Standardise names for abstract OpMode extensions, renaming the following classes
  - ColourTuner to `ColourTunerOpMode`
  - PathRecorder to `PathRecorderOpMode`
  - New `RoadRunnerTuningOpMode`
  - These names match abstract OpMode conventions, such as BunyipsOpMode, AutonomousBunyipsOpMode, CommandBasedBunyipsOpMode, etc.
- `RoadRunner` base drives now replace the (optional) instances of `BunyipsOpMode` for `DualTelemetry`
  - This is still a nullable field, and new constructors exist that omit this term if desired
  - The telemetry fields are simply for dashboard reporting of robot position, so the original implementation using `BunyipsOpMode` was not required
- `AprilTagData` has been updated, using `Optional` instances for ftcPose, metadata, and rawPose
  - These fields collapse the previous x, y, and other nullable coordinates for an AprilTag detection, as null checks were tedious
  - `AprilTagData` now integrates the data classes from the FTC SDK itself as part of this data class, instead of unpacking them as previously done
  - Utility method `isInLibrary()` will check for null against these optional instances now, as they will only be present if they are in the AprilTag library attached to the AprilTag processor
- `UserSelection` now is a `BunyipsComponent`, and therefore no longer needs BunyipsOpMode injection (a legacy BunyipsLib paradigm)
  - This was a late migration after the static OpMode singleton, and is now reflected in `UserSelection` for consistency
  - The constructor parameters have therefore been reduced from 3 to 2, the `this` parameter is not required
- `unitPose`, `unitVec` and `mirror` are now static methods of the `RoadRunner` interface
- `RoadRunnerDrive` now defines vararg-length getMotorPowers() and setMotorPowers() methods, where previously these overrides would be implemented at a per-class basis
- This makes RoadRunner drives more consistent with their interfaces
- `Storage.memory().unusableComponents` is now a `HashSet` rather than a `List`
### Non-breaking changes
- `DualTelemetry` now smartly parses telemetry captions based on the presence of an automatic separator string
  - Instead of telemetry objects getting stuck as DS001 etc, if the messages are formatted correctly they will be added and updated as their own items
  - The default automatic dashboard caption value separator is `: `
  - BunyipsLib telemetry already follows the `Caption: Value` format, meaning FtcDashboard telemetry should be more organised and should be easier to graph and log
  - The pre-existing caption value separator methods have been revamped to change and get this separator, as well as the public property
  - This separator does not impact the real telemetry caption value separator, which should be an empty string with `DualTelemetry`
- Undeprecated `DualTelemetry.addData` methods to instead work similar to `add()` but with using the automatic separator string as mentioned above to ensure separation
- Improved `HoldableActuator` homing task to check for the bottom limit switch before starting the task
- `Processor` for Vision can now be delegated, useful for composite processors that recursively use processors internally
- All subsystems now have a `name` property, as many subsystems would internally reference a name property
  - The name property is protected to match the legacy implementations of the name system
  - Public access to get the name property is achieved via the `toString()` call
    - The new `toVerboseString()` method (similar to `Task`) will give the legacy extended behaviour when calling `toString()` on a subsystem in previous versions
  - Public access to set the name property is achieved via `withName()` which will return `this` (matches common legacy implementations)
  - New method `isDefaultName()` for checks between the default name (`getClass().getSimpleName()`) and a custom name
- `AprilTagPoseEstimator` now has a heading estimate mode, where heading information from the pose estimator will also be used as part of the pose estimate
- `RoadRunnerTask` now only cancels the current trajectory on task finish if it has been interrupted
- `AutonomousBunyipsOpMode` `addTaskAtIndex` will no longer throw an exception if the index is greater than the task list size
  - The task will be appended to the end of the task list instead
  - Negative indices will still throw an exception
- `AutonomousBunyipsOpMode` and `Scheduler` now updates subsystems before executing tasks, allowing subsystems to have run at least once before accepting tasks
- Debugging improvements through telemetry on subsystem disables and Logcat warnings for peculiar states
- The Inertial Measurement Unit is now nullable in all Mecanum RoadRunner drives
  - This is because Three Wheel odometry does not rely on the IMU, therefore the proper null safety exists now and you can simply pass null into the parameter
- The Colour Tuner OpMode scalars can now be tuned via FtcDashboard (previously impossible as it would override scalar values)
  - This is accomplished by changing the scalars of the ColourTunerOpMode object in the dashboard
- `CartesianMecanumDrive` now auto-sets the motors to `ZeroPowerBehavior.BRAKE` on construction
- `RunForTask` can now take in an extra argument for a callback to run after the task is completed
- Normalised all constructors that take in an instance of `DcMotorEx` to take in `DcMotor` instead and upcast internally
- `ThreeWheelLocalizer` constructor has been given more overloads to be more brief in specifying last known positions
- Multi-threading methods exposed in the `Threads` utility have been improved to provide better logging and more method overloads
- The formulae used in `EncoderTicks` have been optimised to reduce approximation inaccuracy
- Exception handling now extends to threads run via the `Threads` utility in the same way main thread exceptions would be logged to the DS and Logcat
- `RobotConfig` init() method will now throw an exception if called more than once, as `HardwareMap` calls are expensive and not required more than once
- Changed init-phase telemetry fault error messages to be more explanative
### Bug fixes
- Added improved robot safety features, primarily with RoadRunner drives
  - A watchdog now observes all BunyipsLib RoadRunner drive instances and auto-locks them if they have not had a call to `update()` recently
  - This watchdog timeout is a constant (`RoadRunner.DRIVE_UPDATE_SAFETY_TIMEOUT`), set by default to 500ms
  - `DualTelemetry` now displays a red timer that overrides the yellow slow loop timer when the safety watchdog is in range of triggering
  - The drive subsystems are special in the regards that hardware can propagate outside of `update()`, most `RoadRunnerDrive` methods propagating instantly on the hardware
  - This timeout is to ensure the subsystem is actually enabled before allowing calls to these methods, which has not been the case previously (a disabled subsystem should never respond to commands)
  - The drive is also the most volatile subsystem, where loop times of >500ms could make the robot dangerously unresponsive
- Fixed a buffer parse error in `Controller` related to the SDK gamepad; the controller should now natively copy over the gamepad ID and other state without unexpected behaviour
- The `RoadRunnerDrive` `stop()` method implementations now cancel trajectories as well
- `AlignToContourTask`, `MoveToContourTask` and `AlignToAprilTagTask` now do not end their Autonomous tasks unless they have seen a contour/AprilTag first
- Fixed a rotation matrix flaw in `AprilTagPoseEstimator` where tags that were not 90 degrees offset to the field did not rotate the robot with the tag
- Fixed a bug where RoadRunner pose data was not being transferred between OpModes due to switching localizers clearing pose data
- Improve race condition telemetry of `UserSelection`
- Fixed a telemetry bug where the target and current positions of `HoldableActuator` in AUTO mode were flipped
- FtcDashboard now uses full size tags for logging improvement
- `MecanumDrive` now lets you access MecanumCoefficients attached to the drive via `getCoefficients()`
- `BunyipsOpMode` now early returns if a stop was requested on initialisation
- Improve `Cartesian` utility class docs from being confusing
- RoadRunner drives now properly switch the drive mode when using Velocity PID when it needs it and when it doesn't
- Fixed a bug in `DualTelemetry` where `setValue` calls to a `HtmlItem` would not build the reference but the value
- Fixed a potential runaway motor when `HoldableActuator` subsystems were disabled
- Fixed a double reference `get()` error in the `DualTelemetry` update method for log messages
- `RobotConfig` now error handles the `onRuntime()` method for `BunyipsOpMode`
- Fixed a bug where `NullSafety.assertComponentArgs()` would never return false
- Fixed a bug where an exception could be thrown during the init-phase although guarded by `assertParamsNotNull()` internally
### Additions
- The `Motor` class, an abstraction around the DC motor that is a drop-in replacement to the SDK approach for `RUN_USING_ENCODER` and `RUN_TO_POSITION` modes
  - The `Motor` class extends the `DcMotorEx` implementation, wrapping a motor entirely and can be obtained directly from `RobotConfig`
  - This wrapper allows you to set your own `SystemController`s to use for `RUN_USING_ENCODER` and `RUN_TO_POSITION`, which will be updated and applied on every `setPower` call to your motor
  - This **bypasses the SDK** and allows you to run your own custom PID which is orders of magnitude faster to run via this class
  - In the SDK, the built-in modes only run at 20Hz, which is a major limitation that makes the controllers sluggish
  - `Motor` is designed to be downcasted to `DcMotor` in any implementation while still providing abstraction over the system controllers
  - This class functions as close as possible to the original DcMotor, where the operation of this class is the same as how it would be done with a stock `DcMotor`. Migration should be near instant.
  - These system controllers have built-in gain scheduling support, using an `InterpolatedLookupTable` to schedule gains based on internal encoder positions while interpolating between known gain setpoints
  - The actual commanded mode of motors running in this class is set to `RUN_WITHOUT_ENCODER`, but the internal and exposed mode will be managed internally by `Motor`, allowing maximum speed and accuracy from your system controllers
  - Additional features allow configurations at the motor level, including max power and faster encoder resets
  - Review the new documentation about this wrapper for the full capabilities of what this class can do
- Alongside `Motor`, the `SystemController` and `PIDF` interfaces has been added to provide common ground for a system control algorithm such as PID or feedforward
  - This interface is implemented across all system controllers and can be used to manage coefficients without necessarily knowing about the controller underneath
- New `ArmController` system controller, which internally composes a PID controller and the WPILib `ArmFeedforward` to provide robust controls for arms suspended on beams at angles
- New `PIDFFController` system controller, which internally composes a PID controller and the WPILib `SimpleMotorFeedforward`
- New `HolonomicVectorDriveTask`, a variant of the `HolonomicDriveTask` that will use pose locking to improve TeleOp capability
  - This task will take a snapshot of the current position after a short duration when the user input translation or rotation components are released
  - The task will automatically use PID controllers defined by you to hold that current position snapshot and resist forces
  - Note that this will not solve issues such as translational drift, there are other solutions to this issue, although it will attempt to tackle rotational drift
  - This system can be comparable to one of a drone, where releasing the sticks and allowing it to hover will hold position and resist external forces
  - You may also choose to set your own locking positions that will cause the robot to move to that position in real-time
    - This is most useful with heading, for example, allowing you to bind a button to rotate towards the front of the field or to lock rotation forwards
- `TelemetryMenu`, which is a component that allows you to construct gamepad-interactable menus through any telemetry receiver (extracted from SDK v9.2, creator OpenFTC)
- `RoadRunnerTuningOpMode` abstract class, which uses a `TelemetryMenu` to provide dynamic selection of tuning mode at runtime
  - `gamepad1` will be used to select which tuning routine from the v0.5 RoadRunner tuning process to use
  -  This makes having to rebuild the code not required in order to perform multiple RoadRunner tuning tests, which dramatically decreases tuning time from waiting
  - To use this, extend `RoadRunnerTuningOpMode` and supply a base `RoadRunnerDrive`
  - Once annotated with `@TeleOp` and run, the telemetry will provide a menu to pick a tuning OpMode
  - FtcDashboard tuning adjustments have been neatly compacted under the `RoadRunnerTuningOpMode` tab and in their own sub-branches
  - All RoadRunner tuning OpModes have improved their declaration of units to FtcDashboard
- `IntrinsicMecanumLocalizer` RoadRunner localizer, which is an emergency time-drive equivalent localizer that guesses the position of a Mecanum-drive robot based on the power supplied to the wheels
  - This is useful for an emergency situation where no other localizers will work (including the default)
  - The coefficients for this localizer must be tuned empirically
- `SwitchableLocalizer` RoadRunner localizer, which is a composite localizer that allows for dynamic switching between two localizers via self-test tasks or by the static field attached to the class
  - This allows for testing of deadwheels to be performed before an OpMode, allowing you to switch to a backup if desired through automatic or manual testing via telemetry
  - RoadRunner drive classes now have `useFallbackLocalizer` methods for mentioning which fallback localizer should be used with this drive, returning the SwitchableLocalizer instance for you to run a test task
  - SwitchableLocalizer is not a subsystem, but exposes tasks via the BunyipsSubsystem convention (`tasks` public field)
- `BlinkinLights` subsystem, which is a subsystem for the REV Blinkin Lights LED driver
  - Integrates with the `Task` system to queue lighting effects for durations
- `Processor` has new virtual methods `onAttach()` and `onRunning()` for processors to hook these Vision events
- `SwitchableVisionSender` now supports streaming across cameras, adjustable with the static index to mention which vision instance to use. Note that this index will only work if the instance itself has called `startPreview()`. Methods such as `setPreview()` will auto-set the index to the current vision instance, therefore not breaking old use cases of `SwitchableVisionSender`
- The Dokka documentation tool is now used with BunyipsLib. All clones of BunyipsLib can access the HTML docs in the docs/ directory, or they may be accessed at our link https://murray-bridge-bunyips.github.io/BunyipsLib/
  - Alongside Dokka, BunyipsLib has had a documentation update and all JavaDoc/KDoc should be more robust compared to previous versions
  - This includes `@since` declarations that will exist on all BunyipsLib classes, `v1.0.0-pre` indicating that this class was originally implemented before BunyipsLib had a semver
  - More debug logging has been added to Logcat for various components throughout BunyipsLib
  - A GitHub wiki will contribute to where example code and information will exist for BunyipsLib hereon, alongside JavaDoc
- New `DynamicTask`, which is a task wrapper that will construct an inner task when the wrapper begins running (runtime construction)
  - This is the equivalent of the WPILib `DeferredCommand`
  - Useful for tasks that can only be evaluated when the OpMode starts, such as pre-game vision tracking
- Subsystems are now constructed with implicit registration, you can access all currently constructed subsystems via static `BunyipsSubsystem.getInstances()`, or you can conveniently update all constructed subsystem through `BunyipsSubsystem.updateAll()`
- `Storage` has a new memory field `lastKnownAlliance`, which is set either by the user or automatically via a `UserSelection` instance that uses values in `StartingPositions`
- As there are many fields within BunyipsLib that are static, `Storage` provides a static method `resetAllStaticFieldsForOpMode()` for non-BunyipsOpModes to reset fields. This is done automatically in BunyipsOpMode.
- `Tasks` utility which is a utility to run `Task` instances through a iterative lifecycle
  - This util was created as many of BunyipsLib core subsystem features are now tasks, and this utility bridges the gap to allow iterative (non-command-based) OpModes to run tasks
  - Read the docs for more information on how to use this class
- `AprilTag` has alternate constructors which allow you to add custom builder instructions from user code, rather than relying on the BunyipsLib defaults only
- `AprilTagPoseEstimator` has a `setCameraOffset()` method, where you can set a robot-to-camera offset that will be used to further the accuracy of the AprilTag pose estimator
- New `Task` virtual method `onInterrupt()` which will be called if the task is finished but the finish condition has not been met (e.g. ended via `finish()` or `finishNow()`)
- New Reset Last Known Positions built-in OpMode that will reset the last known Storage Memory values for last known pose and alliance
- `RoadRunner` trajectory task builder `setScale()` method, which will multiplicatively scale distances in further builder instructions
- `RoadRunner` `unitPose` and `unitVec` methods now have scalar multiplier overload parameters to mention a multiplier for the original inches
- New `noTimeouts` Reference type is exposed on the `RoadRunner` interface to not set the future built `RoadRunnerTask` timeouts to the end of the trajectory time
- `Exceptions` now has a utility method `runUserMethod` which is used internally to execute volatile user methods with exception catching
- `RoadRunner` has a new builder parameter `setRefMirroring()` which can toggle the state of pose ref mirroring (which is similar to the new implementation of how a builder parameter will set mirroring for the constructor parameter)
- Two and three-wheel localizers now have an X encoder multiplier and Y encoder multiplier that can be empirically tuned to increase accuracy
  - These multipliers are in the RoadRunner quickstart but didn't make it into BunyipsLib until now
- `DriveConstants` has a new RoadRunner constant for admissible error and timeout via `setAdmissibleError`
  - This directly connects to the PIDVA controller used when working with trajectory following, and can be adjusted to your liking
  - The default parameters are usually sufficient and do not need any method calls to use
  - This can be paired with the `noTimeouts` `RoadRunner` field for tasks
- Subsystems can now be run asynchronously via `startThread` and `stopThread`
  - These are experimental options that allow you to run this subsystem on another thread, but take due care as hardware calls on other threads are dangerous and looked down upon in FTC
  - In any case, if you do choose to multithread, the created threads will be managed by the `Threads` utility
- `Encoder` class, which can be used to represent an encoder with built-in support for velocity overflow correction and acceleration calculations
  - This class is used internally by `Motor`
- `TriConsumer` interface, which is the `BiConsumer` but with three arguments (used internally for RoadRunner tuning modes)
- `EncoderTicks` now has methods for retrieving motor angular/translational velocity and acceleration (accel in `Motor` class only) via the `EncoderTicks.Generator`
- `Cartesian` utility class now has a `rotate()` method to allow simpler application of rotation matrices
  - RoadRunner has this built-in to `Vector2d` via `rotated()`
- `Controls` now allows the creation of robot/Cartesian vectors as well as poses
- Tracking wheel localizer coefficients now have a `setOverflowCompensation()` method to set whether their encoders should be using velocity overflow correction
  - This makes overflow compensation options interpret at a config-level instead of needing to define them at the OpMode-level
- `CartesianMecanumDrive` now has a `setMode()` method to set the mode of all motors on the drive
- `InterpolatedLookupTable` now has error-safety methods such as `testOutOfRange` and clamped outputs with `getMin()` and `getMax()`
  - This lookup table will now throw OutOfRangeExceptions if `get()` is called with a value outside of the function domain
  - The lookup table also now has early catches for null references on tables that are not built yet, which will not auto-build but alert the user more elegantly
- `BunyipsSubsystem` now has an `isDisabled()` boolean check method to allow users to see if a subsystem is currently disabled

## v3.5.0 (2024-07-12)
Migration to SDK v9.2, integration of new external control systems, some bug fixes and stability improvements.
### Breaking changes
- New deprecations which will be removed alongside SDK v10.0, including Tensorflow Object Detection
  - `TFOD` and `GetWhitePixelTask` will be archived in the major release where SDK v10.0 is adopted
  - As for other CENTERSTAGE-specific tasks, they will remain in BunyipsLib under the `centerstage` packages where appropriate
  - In the BunyipsLib major release where SDK v10.0 is adopted, all other deprecated features throughout BunyipsLib will also be removed
  - For now, these features will continue to work as expected and expected removals have been marked as deprecated
  - This may include other changes to the Vision system to adopt full multi-camera support (https://github.com/Murray-Bridge-Bunyips/BunyipsLib/issues/29), as it is only partially implemented at the moment
  - `VisionTest` has been removed of TFOD
### Non-breaking changes
- Added a Recommended SDK Version to the build process, which will be checked at runtime to ensure the currently used SDK version is the recommended version
  - This is to ensure maximum compatibility with the BunyipsLib codebase, as this version is the one that has been tested and confirmed to work
  - This will not prevent the user from running OpModes, but will log a robot warning message to the Driver Station telemetry and to Logcat at the start of any `BunyipsOpMode`
  - The recommended version has been set to v9.2, which as of this release is the latest SDK version
- `VisionTest` has changed which processors that can be tested, and now has options of `RAW`, `APRILTAG`, and the four CENTERSTAGE pixels
### Bug fixes
- Fixed broken implementations throughout `RoadRunner` where the method overloads were not being reflected properly
   - This was due to not implementing correct methods from the `TrajectorySequenceBuilder` interface
   - waitFor() has been given an additional overload to match the scheme of (magnitude, unit) for RoadRunner task building
- Fix a misparse of the `Controller` gamepad to match what does SDK does
  - This may have caused issues where unmanaged gamepad inputs did not work as expected as the buffer was not being wrapped properly
### Additions
- New control system utilities from WPILib have been ported to BunyipsLib
  - This includes the `TrapezoidProfile`, `ArmFeedforward`, `ElevatorFeedforward`, `SimpleMotorFeedforward`, and `ProfiledPIDController` classes
  - Documentation on using these utilities can be sourced from WPILib/FTCLib, as these classes are in line with these counterparts
  - `ArmFeedforward` has been updated to use WPIUnits for the calculation method, to interop with the `EncoderTicks` class

## v3.4.2 (2024-07-03)
Improved exception handling and edge case fixes.
### Breaking changes
- The `toString()` method of `Task` has been made final and documentation updated appropriately
  - In order to change the name of a task, the `withName()` method should be used
  - As this is a non-user-facing change, it is not enough to warrant a major version bump
  - Users who have overridden the `toString()` method in their tasks will need to change this to use the `withName()` method
### Non-breaking changes
- `dynamic_init` is now in a while loop as opposed to a do-while loop, ensuring an early exit is respected before running more initialisation code
### Bug fixes
- Improved error catching with the `initTask`, where exceptions during the onFinish() method would cause a full crash
- `BunyipsOpMode` now takes in a `RobotTask` instead of a `Task` in `setInitTask(...)`
  - This was the intended behaviour, as it supported using a minimal interface to begin with

## v3.4.1 (2024-07-02)
Minor visual changes.
### Non-breaking changes
- Robot Controller blink pattern updated
  - Replaced the READY state (solid green) with a *blinking green-cyan* (similar to `dynamic_init`) pattern, this is because the READY state was too similar to the default state of the Robot Controller
- `DualTelemetry`'s `opModeStatus` can now be set to empty values
  - The overhead status will now update more uniformly when the status is empty 
### Bug fixes
- Fixed mismatched HTML tags in `BunyipsOpMode`

## v3.4.0 (2024-06-28)
Vision related updates and functionality optimisations.
### Non-breaking changes
- Changing `loopSpeed` of `BunyipsOpMode` will now log a warning message to Logcat to inform the user of this change
  - The display of the loop speed in the Driver Station telemetry will also additionally turn yellow if this target speed cannot be achieved due to too slow of a loop speed
- Removed/fixed/mitigated all TODOs in the BunyipsLib codebase
- `AutonomousBunyipsOpMode` now has the `disableHardwareStopOnFinish()` method, which will turn off the automatic hardware halt protection when all tasks are completed
  - This is useful to disable if you wish to keep a motor running after the BunyipsOpMode activeLoop is finished 
  - By default, this safety feature is and has been enabled throughout all `AutonomousBunyipsOpMode`s
### Bug fixes
- Fixed faulty constructors of `AlignToContourTask` and `MoveToContourTask` where they were using type `MultiColourThreshold` instead of `Processor<ContourData`
  - This made it so these tasks could only be used with a MultiColourThreshold, which was not the intended behaviour
  - The constructors now take a `Processor<ContourData>` instead, which allows for any contour processor to be used
  - Old implementations will still work as the `MultiColourThreshold` is a `Processor<ContourData>`
### Additions
- `ColourTuner`, a utility OpMode that can be extended to tune a variety of colour-based vision processors
  - This allows the lower and upper bounds of a `ColourThreshold` to be tuned in real-time via controller input, and the processed image to be displayed on FtcDashboard 
  - Consequently, the `colourSpace` of all `ColourThreshold` classes has been exposed as a public final field
    - A utility attached to the `ColourSpace` is now able to determine the channel name (e.g Red, Green, Blue) of a given scalar
    - These functions are used in the `ColourTuner` to display the channel name of the scalar being tuned
- A new universal OpMode, "Reset Robot Controller Lights" has been added and inserted next to the FtcDashboard enable/disable OpMode
  - As BunyipsOpMode will change the RC lights and not be able to change them back when the OpMode is stopped (as OpModes cannot access LynxModules when stopped), this OpMode will reset the lights to their default state
  - This OpMode is useful for resetting the lights after a BunyipsOpMode has run, as the lights will remain in their last state, therefore resetting will inform others the robot is disabled
  - Integrated/test OpModes, including this one, have been moved from the `test` package to the `integrated` package
- `DualTelemetry` has an alias method for adding a newline to telemetry, `addNewLine()`
- `HtmlItem` of `DualTelemetry` now has a builder parameter that can take in a `BooleanSupplier` to determine if the item should have styles applied to it
  - Note that these styles only apply to the ones attached to the `HtmlItem` itself, and not the text inside the item
  - This is useful for selectors that may want to apply styles to the item based on a condition
- `Text.formatString` has an alias method `Text.format` for convenience

## v3.3.2 (2024-06-25)
General bug fixes and utility improvements.
### Non-breaking changes
- The `OpModeAnnotationProcessor` has been removed, as it does not impact any BunyipsLib code
  - This processor is used when making OpModes in the BunyipsLib project, however all OpModes that should have this checking enabled shouldn't be in the BunyipsLib project
  - The implementation of it in the past for BunyipsLib was also bugged and did not work anyways, and as such has been removed
- `Reference` is now a `volatile` field
  - Methods that rely on the snapshot of this field will be taken as a read on function call 
- `HoldableActuator` now has a maximum timeout (default 5s enabled) for the `homeTask()`
  - Ensure to call `disableHomingTimeout()` or increase the timeout if the actuator takes longer than 5s to home 
  - As such, the `homeTask()` now returns an instance of Task rather than NoTimeoutTask
- `AprilTagPoseEstimator` now has a `setActive(boolean)` method to enable or disable the estimator
  - By setting this to false, no vision data will be processed and the pose will not be updated
- `AutonomousBunyipsOpMode` now exposes a `addTask(Runnable, String)` method to add a task with a custom name
  - This is useful for adding RunTasks that you want to give a name for, without needing to create a new RunTask explicitly
- `StartingPositions` exposes new methods for determining information about a certain starting position
  - `isRed()` and `isBlue()` to check if the starting position is on the red or blue alliance
  - `isLeft()` and `isRight()` to check if the starting position is on the left or right side of the field
  - These results are lazy-loaded and cached for future use, as with the `getVector()` method
- `DriveToPoseTask` now has `withMaxForwardSpeed(...)`, `withMaxStrafeSpeed(...)`, and `withMaxTurnSpeed(...)` methods to set the maximum speed ranges for the task
  - These speeds will be used to clamp the output of the PID controllers to ensure the robot does not exceed these speeds
  - This is similar to how `MoveToAprilTagTask` work, allowing smoother and more controlled motion
  - By default, these are set to `1.0`.
### Bug fixes
- Fixed a critical `RoadRunner` bug where the implicit drive pose was not discarded properly
  - This was due to a cached result being used after the pose info was discarded
  - Implicit pose construction should now work as expected
- Motor power is explicitly set to zero in the `HoldableActuator` `deltaTask(...)` and `gotoTask(...)` when the task is initialised
  - This seems to fix a bug where the motor controller ignores further power instructions and has the actuator moving at undefined speeds
  - The update cycle will set the power to the correct value, as intended to conform to power clamping
- `RoadRunnerTask` now actually sets the task timeout to the trajectory duration (if not explicitly set) and assigns the task to the subsystem
  - Previously, this would only work for `Trajectory` objects, and not `TrajectorySequence` objects due to a missing implementation 
- `DualTelemetry` methods for removing objects from telemetry now work to support HtmlItems
  - Previously, the DS would not be able to find these objects as they were not the appropriates instances
  - HtmlItems now expose a `getItem()` method to get the actual telemetry item that is wrapped
  - The remove methods will try to unwrap these items automatically from an HtmlItem, so the user does not need to manually unwrap them
- `AprilTagPoseEstimator` no longer checks for the processor to be active when instantiated
  - This was an oversight as the processor should simply not set poses if it is not active, and has been updated accordingly
  - This is to allow for the processor to be enabled or disabled at runtime which will impact if the pose estimator is run
  - The `update()` method of the pose estimator now checks if the processor is active(attached+running) before updating the pose, and no-ops if it is not active (rather than emergency stopping on init and only checking once)
- `DriveToPoseTask` now calculates twists properly
  - Before the error was fed twice into the PID controllers, which worked but was not the intended behaviour and could pose some unexpected results
  - The new calculation works as expected and should not change any previous implementations

## v3.3.1 (2024-06-14)
RoadRunner pose interpretation restructuring for improved global pose estimation and trajectory handling.
### Breaking changes
- While this is a minor version release, it changes how trajectories from the `RoadRunner` interface are parsed
  - This was due to a bug where poses would be discarded and global reference frames were impossible, and minor tweaks have been made to the RoadRunner interface
### Non-breaking changes
- All RoadRunner trajectories built with `RoadRunner` no longer set the pose estimate to the start pose of the trajectory
  - This is to allow for global reference frames to be used, and to allow for the pose estimate to be set to the start pose of the trajectory
  - This is a breaking change for any code that relied on this pose estimate, and code will likely need to be re-tested
  - This change was made to allow for more flexibility in the RoadRunner interface, and to allow for more accurate pose estimates
- `RoadRunner` no longer discards the last known pose when implicitly making a trajectory in the presence of an existing pose estimate
  - This allows users to call `.setPose()` without having their estimates reset by the implicit discarding rule
- `runSequence` of `RoadRunner`'s `makeTrajectory` now has an optional second parameter if the current pose should be set to the start pose of the trajectory
  - This is useful for when you want to set the pose estimate to the start pose of the trajectory, but not always
  - The reason this is now optional as these pose calls will be called at initialisation, rather than at runtime, and providing the option gives more flexibility
  - This will not change any previous `runSequence` implementations
- `RoadRunnerTask` now sets the default task name to include the start and end trajectory poses in inch/rad format
### Additions
- `RoadRunner` now has new methods
  - `setPose(...)` for setting the current pose of the robot in the RoadRunner drive quickly and with custom units
  - `unitVec(...)` and `unitPose(...)` for creating custom unit vectors and poses (like `Pose2d` and `Vector2d`) with WPIUnits
- `StartingPositions` now exposes a `pose` instead of a `vector` for the starting positions
  - These poses face inwards to the field, and can be rotated by adding with `.plus()` calls
  - The old `vector` field is still exposed for continuity, which simply calls `pose.vec()` internally, and will not break any previous code

## v3.3.0 (2024-06-13)
Drive To Pose task and some minor bug fixes.
### Bug fixes
- Added more try-catch blocks in `AutonomousBunyipsOpMode` and `CommandBasedBunyipsOpMode` when calling back user functions
  - Before, if an exception was thrown, additional code that needed to be run as part of the natural lifecycle of these OpMode variants would not be executed
  - OpModes should now be more exception resilient
### Non-breaking changes
- `BunyipsOpMode` now manually clears bulk cache data from the hardware
  - Cache is automatically cleared once per activeLoop or initLoop, and the bulk read is called at the start of the loop
  - This does not change any functionality, but is a performance improvement where hardware will not call multiple bulk reads in a single loop
- Robot Controller lights in a `BunyipsOpMode` flash in accordance with the OpMode state
  - **SOLID CYAN**: static_init 
  - **FLASHING CYAN**: dynamic_init
  - **SOLID GREEN**: ready with no exceptions thrown
  - **SOLID YELLOW**: an exception was thrown during static_init or dynamic_init, and was caught
  - **FLASHING GREEN**: running
  - **SOLID WHITE/LIGHT GRAY**: finished
  - **SOLID RED**: an *unhandled* exception was thrown during the execution of BunyipsOpMode
  - *Note*: Due to limitations on the FTC SDK, if the OpMode is stopped these lights will not update and remain as they were when the OpMode was stopped
- `Exceptions` class is now more Java-friendly
  - The lambda function to log messages on an exception is now a `Consumer<String>`, and is JVM static
  - This may break any custom implementations of `Exceptions` that relied on the old `(msg: String) -> Unit` KFunction
  - This method also no longer handles the case of `throws InterruptedException` as realistically users won't need to handle this
    - The FTC SDK will still handle this exception, but it is not necessary to add it as a throws clause in Java code 
  - This does not change any previous functionality or API surface
- Modified all instances of `PIDController` in BunyipsLib task construction to be `PIDFControllers`
  - Allows more flexibility in the construction of PID controllers, and allows for the use of feedforward gains
  - PIDController extends PIDFController, and as such all previous PIDController implementations will still work
### Additions
- Drive To Pose Task
  - A RoadRunner-powered pose task that will drive the robot in a straight line to a target pose from the current estimated pose
  - This task is different to a RoadRunner trajectory, as it will use PID and feedforward control to reach the pose
  - Useful for dynamic "at runtime" pose alignment, where the pose estimate may have been updated externally
    - An example of this is using the `AprilTagPoseEstimator` to update the pose of the robot, and then using the `DriveToPoseTask` to ensure the robot is aligned properly with a target pose

## v3.2.0 (2024-06-11)
New methods, improvements, and features.
### Breaking changes
- New deprecations, these functions will continue to work but be marked as deprecated and are subject to removal in future versions
  - `runDebounced` of `Scheduler` has been renamed to `runOnce`
  - Telemetry methods of `BunyipsOpMode` have been deprecated in favour of the new `DualTelemetry` methods (exposed through the `telemetry` field)
    - As a result, `BunyipsOpMode` now exposes a field called `t` which aliases `telemetry` for convenience and to solve a bug with Kotlin interop
  - `HtmlItem`'s now report errors when `addData` functions are called, as they are not supported
- `Storage` was revamped, moving all volatile memory items into the `.memory()` subclass
### Non-breaking changes
- `Cannon`, `Switch` and `DualServos` now check if the open and fired states are configured the same, throwing an exception if they are
- Task names throughout BunyipsLib have been updated to be more readable (e.g. "OpenServoTask" -> "Open:SIDE")
  - All tasks as a default name try to represent what they are, including the `TaskGroups` by seeking the name of the tasks in the group
- Debugging telemetry has been improved
  - Task group creation has been improved to show when created in `AutonmoousBunyipsOpMode` through the `logCreation()` method
  - `CommandBasedBunyipsOpMode` will try to describe what commands have been assigned to what subsystems and the binds to activate these commands
- The LynxModules in `BunyipsOpMode`'s are now controlled to blink and change colours in accordance with the OpMode state
### Bug fixes
- Fixed a bug where setting a default task was impossible when the subsystem was disabled, where it should be possible
- Removed an internal call to `update()` in `RoadRunnerTask`, users will have to ensure subsystems are being updated properly
- `RoadRunner` now constructs tasks properly to provide timeout information to the upper task
- Improved `AutonomousBunyipsOpMode` to busy-wait in the start phase of the OpMode for the UserSelection thread
- Patch an FtcDashboard bug where telemetry was not working causing memory leaks
- Fixed a crash where `AutonomousBunyipsOpMode` would crash if the user did not select an OpMode
### Additions
- `BunyipsOpMode` has a `onActiveLoop(...)` method, which adds functions (Runnables) to be called before the `activeLoop()`
  - This is useful for any pre-loop operations that need to be performed before the main loop, or utilities that require a functional update
- `AprilTagPoseEstimator` for RoadRunner drives, which will extract the pose from any Vision cameras that are attached to the robot
  - This will set the current pose of the RoadRunner drive pose to what the webcam sees, which allows for dynamic calibration and field awareness
  - It is structured like a subsystem with an update method, but is not a subsystem and should be treated as a utility, able to be updated in the `onActiveLoop()` method
- `Storage` has been revamped to include static methods for accessing volatile memory and the filesystem
  - Gson is used to store and retrieve objects from the filesystem, storing any object that can be serialised in a simple HashMap that can be accessed by the user
- `Cannon` and `Switch` now have `isFired()` and `isOpen()` (as well as `isReset()` methods) for polling the state of the servo
- `Dbg` has a `stamp()` method, which will log the current timestamp of a running BunyipsOpMode and user context
- `HoldableActuator` now supports a top limit switch, which will stop the actuator from moving if it reaches this upper bound
  - Configuration is performed similarly to the bottom switch, but by instead calling `withTopSwitch(...)`
- `Scheduler` now has `andIf()` and `orIf()` methods to chain boolean expressions together
- `BunyipsSubsystem`'s have virtual methods `onEnable()` and `onDisable()` which are called when the subsystem is enabled or disabled
  - `onEnable()` is called when `enable()` is called, or on the first active loop of the subsystem
  - `onDisable()` is called when `disable()` is called, or when `assertParamsNotNull()` auto-disables the subsystem
- `Scheduler` has `getAllocatedTasks()` and `getManagedSubsystems()` to retrieve the tasks and subsystems that are currently being managed by the scheduler
- `Text` has a `StringBuilder` implementation which internally uses `formatString()` to build a string
- New ramping function providers `RampingSupplier` and `RampingValue`
  - These are extensions of what `DcMotorRamping` does, but can be used in other contexts for any values that need smooth damping
- `BunyipsOpMode` has a new `loopSpeed` field, which can be set to a custom loop speed for the OpMode
  - This is useful for setting a custom target loop speed for the OpMode, which can be used to save CPU cycles
- `DualTelemetry` now displays the current time of when messages were logged in the telemetry log, if using a MovingAverageTimer
  - This behaviour is automatically enabled on `BunyipsOpMode`, which uses the `timer` field to log the time of the message
  - Bracket colours of the telemetry log timestamp represent init and active phases
  - `DualTelemetry` now exposes a logBracketColor field as a result
- `DualTelemetry` has a loopSpeedSlowAlert field, which will change the colour of the loop time (if available) to yellow if the loop speed is slower than this value
- `BunyipsOpMode` exposes `robotControllers` as a method to get the LynxModule instances of the robot

## v3.1.2 (2024-05-23)
Update BunyipsLib dependencies.
## Non-breaking changes
- Updated WPIUnits to the latest branch, which reworks the internal logic of unit construction
- `Task` now extends `BunyipsComponent` to retrieve the `opMode` field, instead of defining it again internally
## Additions
- Units can now be divided by other units performing dimensional analysis, similar to the multiplication of units
- A few new units now exist, including a new base unit for moment of inertia and more integrated second-derivative units (`FeetPerSecondPerSecond`, `RadiansPerSecondPerSecond`, etc.)

## v3.1.1 (2024-05-19)
Task timing and debugging telemetry improvements.
### Non-breaking changes
- Added an automatic `Dbg` call to run when the `onReady` callback is processed, with a list of tasks added at this period
  - This is to provide a more detailed log of the tasks that are added up to the point of the `onReady` callback
- Init-task now displays full verbose string into Logcat of when it starts running
- Renamed ACTIVE to READY in `Task` when calling a verbose string for clarity
### Bug fixes
- OnceTask has been updated to actually have a timeout of 1 millisecond as it was previously treated as infinite 
  - This allows telemetry to properly display the task as an instant task
- Fixed a bug in `RoadRunner` where the robot would not accept the implicit timeout construction of a trajectory
  - This resolves all RoadRunner trajectory tasks built with default parameters to be infinite

## v3.1.0 (2024-05-19)
HTML-enhanced improved Driver Station and FtcDashboard telemetry.
### Breaking changes
- `Scheduler.addTaskReport()` now takes an extra argument whether this task is a default task
  - This API was not designed to be used by the user, but rather by the `Scheduler` to collect task and subsystem info
  - As such, this is a breaking change for any custom implementations of tasks/subsystems that may have used this method
  - This is not enough to warrant a major version bump
- `DualTelemetry` now calls all setup methods from within its constructor
  - This removes the need to call `setup()` after creating a `DualTelemetry` object
  - This is a breaking change for any custom implementations of `DualTelemetry`
  - However, since this is an internal developer change, it is not enough to warrant a major version bump
- `UserSelection` no longer sets auto-clear to true for the init-phase
  - While this isn't an API change, it is a change in the default behaviour of `UserSelection` and may crash OpModes that relied on the auto-clear being active
  - You will need to ensure auto-clear is set yourself, as nowhere in BunyipsLib it is enabled automatically during the init-phase
  - Auto-clear is disabled in the init phase, and re-enabled in the active phase
  - Init tasks that report continuous telemetry should use a single added retained telemetry item and call `setValue()` on it
### Non-breaking changes
- Added colours and styles to telemetry messages in the Driver Station
  - All standard telemetry items associated with the stock usage of BunyipsLib have been updated to be more vibrant and easier to read
  - Information is now colour-coded based on importance, and size varying to indicate between debug and normal telemetry
  - `StartingPositions` now has a HTML generator when using `StartingPositions.use()` to display the starting position in a more readable format, reducing confusion
- `UserSelection`, when running will now flash at the user with vibrant colours to indicate an action is needed
  - This is to ensure the user is aware that they need to select an OpMode
  - The message for the selected OpMode has also been moved to the log instead of telemetry to de-clutter the user telemetry space
  - FtcDashboard will now display the selected OpMode status properly in a concise `USR` key, with time data of when it was selected relative to the start of init
- `AutonomousBunyipsOpMode` now sends task finish messages to Logcat with time execution details
- Exceptions are now shown more clearly in the Driver Station telemetry log
- `StartingPositions.use()` now returns an Array instead of a List
  - This array is backed by the JVM as a `StartingPositions[]`
- Telemetry messages have been improved throughout BunyipsLib
  - `Task` and `TaskGroup` now display their timeout information when added to the `AutonomousBunyipsOpMode` task queue
    - `TaskGroup` is calculated by the maximum timeout of all tasks in the group, but may end early depending on the group's finish condition
    - `TaskGroup` logs to the Driver Station as being added as a single task, but generates additional logs for each task in the group on construction
  - Scheduler messages have been reduced in size and made more concise to be one line
  - `AutonomousBunyipsOpMode` task queue messages have been refined to reduce wordiness
  - Integrated subsystems now use colour to indicate their status in the Driver Station
- `RoadRunnerTask` now sets the task timeout of itself to the trajectory duration if the task timeout is not set
  - This is to ensure provide metrics on how long an Autonomous will take to complete as the task time should be the same as the trajectory time
- `BunyipsOpMode` now slows down the loop during periods of idle execution to save CPU cycles
  - During the `ready` phase, the loop is reduced to 5 iters/sec
  - During a `halted` state, the loop is reduced to 10 iters/sec
  - During the `finished` state, the loop is reduced to 2 iters/sec
  - All other phases continue to run at maximum available speed
- `BunyipsOpMode` will no longer run `activeLoop()` at least once if the OpMode is stopped before the first loop
  - This is to prevent actuating motors or doing any heavy work when we want to stop the robot
  - This effectively has moved the activeLoop from a do-while to a while loop
  - Other loop-based methods such as `onInitLoop()` still use a do-while
- `DualTelemetry` now uses the `Func<T>` constructor to pass data to the Driver Station telemetry
  - This does not change any functionality but allows the internal state to work properly, but may present potential issues with clearing telemetry if not aware
- `DualTelemetry`'s `removeItem` is no longer deprecated as it will call `removeRetained` internally
  - The purpose of the deprecation was to discourage removing telemetry items that weren't retained, however to remove complication the method has been restored 
### Bug fixes
- Fixed a critical bug in `AutonomousBunyipsOpMode` where an exception would be thrown for an OpMode that doesn't call `setOpModes()`
  - Additionally, `onReady()` will no longer be called if the OpMode is ended before the user selects an OpMode
- Fixed a faulty debug statement in `BunyipsSubsystem` when a subsystem is enabled via `enable()`.
- Exceptions thrown for `BunyipsSubsystem`'s default task ending now includes the class name
- Improved no-subsystem operation of `CommandBasedBunyipsOpMode`, where an exception will still be thrown however it will not stop the calling of `assignCommands()`
  - We choose to throw an exception here as passing no subsystems shouldn't be a common use case of this command-based system
  - This is to prevent the user from accidentally running a command-based system without any subsystems and therefore no motor output
- Various fixes for `DualTelemetry`
  - FtcDashboard user packets should no longer be dropped
  - Fixed a `ConcurrentModificationException` caused by a missing `synchronized` call on the dashboard items in `update()`
  - In order to let HTML modifications make their way to the dashboard, the internal structure of `DualTelemetry` has been reworked
    - Dashboard items are now a string by reference, which will be updated when the Driver Station evaluates the telemetry function
  - FtcDashboard telemetry indexing has been vastly improved
    - The telemetry is now indexed by type and accommodates for the alphabetical sorting of telemetry items
    - This ensures the telemetry on the dashboard is as close to the DS as possible without any manual intervention
  - Fixed a clearing bug for telemetry where items would be removed based on a clone instead of the original item
### Additions
- Telemetry now uses HTML formatting to enhance the appearance of telemetry messages
  - Allows customisation of telemetry messages, including bold, italic, underline, and colour
  - New HTML text builder utility `Text.html()` and built-in builder-like value wrappers for the `telemetry.add()` method, (`.bold()`, `.italic()`, etc)
    - See `HtmlItem` of `DualTelemetry` for more information
  - These changes also apply to the FtcDashboard telemetry
  - Manual tags can also be used, such as `<b>`, `<i>`, `<u>`, `<font color="red">`, etc
- `AutonomousBunyipsOpMode` now provides more concise data on the overhead display message
  - This includes the current task, current task index, and a new Estimated Time Remaining field
  - The Estimated Time Remaining field is calculated by the sum of the timeouts of all tasks after the current task
  - If the task is a `RoadRunnerTask`, the time remaining will be calculated by the time remaining in the trajectory
  - However, if it is an infinite task, the time remaining will be calculated by adding an arbitrary amount of time as defined by the static field in `AutonomousBunyipsOpMode`
  - This is to provide more information to the driver on the current state of the autonomous program
- `DualTelemetry` now includes a `overheadSubtitle`, which is a customisable text field that can be used to display additional information just under the main status
  - This is useful for displaying statistics similar to the standard `BunyipsOpMode` timing and controller information
- `DualTelemetry` now has an `overrideStatus` field, which allows the user to set the status message to a custom string
  - This will override the status message set by the `opModeStatus` field, and will be displayed in the same location
  - This field is used internally by `BunyipsOpMode` to display 'error' if an exception is thrown
- `Dbg` now includes new `logTmp()` methods, which will log a message just like `log()`, but is marked as deprecated
  - This is to ensure all temporary debug messages are removed before committing, as sometimes these log messages may be forgotten and hamper performance
- `Reference` now has new methods `setIfNotPresent()` and `setIfPresent()`
  - These methods are for setting the reference based on the nullability of the current reference 

## v3.0.1 (2024-05-16)
Minor bug fix for `MoveToAprilTagTask`.
### Bug fixes
- Fixed a missing constructor assignment in `MoveToAprilTagTask`, where the targetTag was not being assigned to the class field
  - This caused the task to always search and lock onto any tag, essentially ignoring the targetTag parameter

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
