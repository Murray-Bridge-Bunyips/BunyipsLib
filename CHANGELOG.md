# BunyipsLib Changelog
###### BunyipsLib releases are made whenever a snapshot of the repository is taken following new features/patches that are confirmed to work.<br>All archived (removed) BunyipsLib code can be found [here](https://github.com/Murray-Bridge-Bunyips/BunyipsFTC/tree/devid-heath/TeamCode/Archived/common).

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
