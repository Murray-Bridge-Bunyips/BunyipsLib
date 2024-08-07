# BunyipsLib Changelog
###### BunyipsLib releases are made whenever a snapshot of the repository is taken following new features/patches that are confirmed to work.<br>All archived (removed) BunyipsLib code can be found [here](https://github.com/Murray-Bridge-Bunyips/BunyipsFTC/tree/devid-heath/TeamCode/Archived/common).

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
