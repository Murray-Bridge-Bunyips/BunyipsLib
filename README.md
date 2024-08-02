![BunyipsLib)](https://github.com/Murray-Bridge-Bunyips/.github/blob/main/bunyipslib_banner.png)
[![GitHub Release](https://img.shields.io/github/v/release/Murray-Bridge-Bunyips/BunyipsLib?color=darkgreen)](https://github.com/Murray-Bridge-Bunyips/BunyipsLib/releases/latest)
[![GitHub commits since latest release](https://img.shields.io/github/commits-since/Murray-Bridge-Bunyips/BunyipsLib/latest)](https://github.com/Murray-Bridge-Bunyips/BunyipsLib/commits/master/)
[![Recommended FTC SDK version](https://img.shields.io/static/v1?label=sdk&message=v9.2&color=orange)](https://github.com/FIRST-Tech-Challenge/FtcRobotController/tree/v9.2)
[![View Changelog](https://img.shields.io/static/v1?label=changelog&message=View&color=informational)](/CHANGELOG.md)<br>
[![GitHub commit activity](https://img.shields.io/github/commit-activity/m/Murray-Bridge-Bunyips/BunyipsLib)](https://github.com/Murray-Bridge-Bunyips/BunyipsLib/pulse/monthly)
[![CodeFactor](https://www.codefactor.io/repository/github/murray-bridge-bunyips/bunyipslib/badge)](https://www.codefactor.io/repository/github/murray-bridge-bunyips/bunyipslib)<br>

##### A powerful all-in-one library for FTC used by the Murray Bridge High School Student Robotics Club.<br>[Check out our new Wiki!](https://github.com/Murray-Bridge-Bunyips/BunyipsLib/wiki)

<br>
<div align="center">
    <img src="https://github.com/user-attachments/assets/75408bde-ccba-408d-a4f9-3969af3b9ec5">
    <br>
    Two <b>BunyipsLib-powered</b> robots scoring in a <i>CENTERSTAGE (2023-2024 season)</i> autonomous.
    <br>
    See: <a href="https://github.com/Murray-Bridge-Bunyips/BunyipsFTC/blob/703ab81345e168eca18ca137d2a8fcae4c716899/TeamCode/GLaDOS/src/main/java/org/murraybridgebunyips/glados/autonomous/l5/GLaDOSUltimatePreloadLeftPark.java">Left (15215)</a> OpMode, <a href="https://github.com/Murray-Bridge-Bunyips/BunyipsFTC/blob/703ab81345e168eca18ca137d2a8fcae4c716899/TeamCode/Wheatley/src/main/java/org/murraybridgebunyips/wheatley/autonomous/arm/WheatleyLeftClawAuto.java">Right (22407)</a> (<a href="https://github.com/Murray-Bridge-Bunyips/BunyipsFTC/blob/703ab81345e168eca18ca137d2a8fcae4c716899/TeamCode/Wheatley/src/main/java/org/murraybridgebunyips/wheatley/autonomous/arm/WheatleyRightClawAuto.java">variant shown</a>) OpMode
</div>
<br>
    
BunyipsLib was originally a collection of common functions that our FTC teams used in competition.
It has since evolved into a full-fledged library that is used by all of our teams. It is designed to
provide useful development utilities in making robot code, offering efficient common modules and
OpMode abstractions with
error handling, dynamic device initialisation, native RoadRunner, FtcDashboard, and Vision
processing, and an
extremely expansible ecosystem.

BunyipsLib takes inspiration from FTCLib, WPILib, RoadRunner, and other libraries, and is designed
to be
the one-stop shop for all of our teams' needs. It is designed to be easy to use, and to integrate
the latest and greatest features from the FTC SDK and other libraries.

BunyipsLib is in active and continually experimental development, expect bugs and to keep up to
date!

## Features

BunyipsLib plans to make OpMode development simpler and easier. This includes integrated exception
catching with a similar ecosystem
to the standard OpMode.

All BunyipsLib OpModes are simple, declarative, and expansible.

```java
@TeleOp(name = "My OpMode")
public class MyOpMode extends BunyipsOpMode {
    private final MyBotConfig config = new MyBotConfig();

    @Override
    protected void onInit() {
        config.init(); // Instant initialisation defined by a common `config`
    }

    @Override
    protected void activeLoop() {
        // Runs repeatedly in the active loop
    }

    // Other virtual methods include: onInitLoop(), onInitDone(), onStart(), onStop()
}
```

Subsystems are declared expressively, allowing a transitionary command-based system with procedural
code.

```java
@TeleOp(name = "My TeleOp")
public class MyTeleOp extends BunyipsOpMode {
    private final MyBotConfig config = new MyBotConfig();

    private MecanumDrive drive;
    private MyArm arm;
    private Cannon cannon;

    @Override
    protected void onInit() {
        config.init();
        drive = new DualDeadwheelMecanumDrive(
                config.driveConstants, config.mecanumCoefficients,
                hardwareMap.voltageSensor, config.imu, config.frontLeft, config.frontRight,
                config.backLeft, config.backRight, config.localizerCoefficients,
                config.parallelEncoder, config.perpendicularEncoder
        );

        // Custom gamepad objects which support feeding lambda functions to evaluate each
        // input or a group of inputs (for custom functions for inputs without having to change component code)
        gamepad1.set(Controls.AnalogGroup.STICKS, (v) -> v * 0.8f);

        cannon = new Cannon(/* ... */);
        arm = new MyArm(/* ... */);
    }

    @Override
    protected void activeLoop() {
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x;

        drive.setSpeedUsingController(x, y, r);
        if (gamepad2.y) {
            arm.faceClawToBoard();
        } else if (gamepad2.a) {
            arm.faceClawToGround();
        }

        // Custom gamepad objects allow for shorthand field names (rt == right_trigger)
        if (gamepad1.rt == 1.0) {
            cannon.fire();
        }
        if (gamepad1.back) {
            cannon.reset();
        }

        if (gamepad1.dpad_up) {
            arm.extendHook();
        } else if (gamepad1.dpad_down) {
            arm.retractHook();
        }

        drive.update();
        arm.update();
        cannon.update();
    }
}
```

Are you more of a fan of full-command-based systems? We've brewed our own command scheduler with
full integrations with our ecosystem.

```java
@TeleOp(name = "Align To Pixel (Command Based)")
public class MyAlignToPixelTeleOp extends CommandBasedBunyipsOpMode {
    private final MyBotConfig config = new MyBotConfig();

    private MecanumDrive drive;
    private Vision vision;
    private MultiColourThreshold pixels;

    @Override
    protected void onInitialise() {
        config.init();

        drive = new DualDeadwheelMecanumDrive(/* ... */);
        vision = new Vision(/* ... */);

        pixels = new MultiColourThreshold(Pixels.createProcessors());
        // Choosable processors with full control methods
        vision.init(pixels, vision.raw);
        vision.start(pixels, vision.raw);

        // FtcDashboard integrations for individual processor stream previews
        // Vision previews work across multiple instances of Vision, allowing debugging across
        // multiple processors across multiple cameras within FtcDashboard.
        vision.startPreview();
        
        // Programmatic selection of previews exist as well, which will auto-switch the stream to
        // focus on this processor and camera.
        vision.setPreview(pixels);

        // The Scheduler will automatically manage all subsystems that are constructed, however
        // users can choose to update their own choice of subsystems for rapid debugging purposes.
        // More integrated solutions include calling .disable() on the subsystems in question
        // that should not be updated.
        useSubsystems(drive, vision); // Only update drive and vision
    }

    @Override
    protected void assignCommands() {
        // Running with default binds on gamepad1, without activating Field Centric drive (POV only)
        drive.setDefaultTask(new HolonomicDriveTask(gamepad1, drive, () -> false));

        // Calls reset yaw method every cycle while gamepad1.y is held
        driver().whenHeld(Controls.Y)
                .run(() -> drive.resetYaw());

        // Calls drive.stop() for 2 seconds after gamepad2.x is released for 2 seconds
        operator().whenReleased(Controls.X)
                .run(new RunForTask(Seconds.of(2), () -> drive.stop()))
                .in(Seconds.of(2));

        // Starts the AlignToContourTask on the drive subsystem when gamepad1.rb is pressed, cancelling on release
        driver().whenPressed(Controls.RIGHT_BUMPER)
                .run(new AlignToContourTask<>(gamepad1, drive, pixels, new PIDController(1, 0.25, 0.0)))
                .finishingIf(() -> !gamepad1.right_bumper);
    }
}
```

BunyipsLib has a multi-purpose Task system, not only for general commands but for actionable tasks to do in
both Autonomous and TeleOp.

```java
public class RunArmFor extends Task {
    private final MyArm arm;
    private final MyScooper scooper;
    private final double power;

    public RunArmFor(Measure<Time> time, MyArm arm, MyScooper scooper, double power) {
        super(time); // Built-in timeout management with the full WPILib units conversion system
        // You may also wish to run this task on a subsystem with super(time, <subsystem>, <should override>),
        // this only applies to TeleOp OpModes, as Autonomous OpModes only run one task at a time.
        this.arm = arm;
        this.power = power;
        this.scooper = scooper;
    }

    @Override
    public void init() {
        arm.setPower(power);
        scooper.startScooping();
    }

    @Override
    public void periodic() {
        // Full telemetry utilities for FtcDashboard and Driver Station
        opMode.telemetry.add("Arm is at % ticks.", arm.getTicks());
        // Smaller set of an active loop to complete a task
        if (arm.isOverCurrent()) {
            scooper.pause();
        } else {
            scooper.resume();
        }
    }

    @Override
    public void onFinish() {
        arm.stopNow();
    }

    @Override
    public boolean isTaskFinished() {
        return arm.calculatePosition() > 400 && scooper.finishedScooping();
    }
}
```

All with full RoadRunner utilities, runtime OpMode selectors, and much more.

```java

@Autonomous(name = "Place a Pixel")
public class MyPlacePixelAuto extends AutonomousBunyipsOpMode implements RoadRunner {
    private final MyBotConfig config = new MyBotConfig();
    private DualDeadwheelMecanumDrive drive;

    private MyArm arm;
    private Vision vision;
    private PurplePixel purplePixelProcessor;
    private WhitePixel whitePixelProcessor;

    private YourInitTask initTask;

    @Override
    protected void onInitialise() {
        config.init();

        drive = new DualDeadwheelMecanumDrive(/* ... */);
        vision = new Vision(/* ... */);
        arm = new MyArm(/* ... */);

        vision.init(purplePixelProcessor, whitePixelProcessor);
        vision.start(whitePixelProcessor);

        // Optionally send all feeds to FtcDashboard, can switch between processor feeds for debugging
        // vision.startPreview();

        telemetry.add("Hello world!"); // Custom telemetry object to provide joint FtcDashboard/DS telemetry
        telemetry.addRetained("Greetings, world.");

        // Full support for any object to use as a selection, including enums
        // This will be automatically shown on the init-cycle.
        setOpModes(
                "PARK",
                "GO_TWICE",
                "SABOTAGE_ALLIANCE",
                "STOP"
        );

        // A task reference to dispatch after initialisation until the init-phase is done, or the task finishes
        setInitTask(initTask);
    }

    @NotNull
    @Override
    public MecanumDrive getDrive() {
        // Passes the drive to the RoadRunner interface to provide
        // utility methods for fast, reusable, and efficient trajectory generation (makeTrajectory() util)
        return drive;
    }

    @Override
    protected void onInitDone() {
        switch (initTask.getSpikeResult()) {
            case LEFT:
                // RoadRunner trajectory to get to the Spike Mark
                makeTrajectory(/* ... */);
                break;
            // ...
        }
        vision.stop(whitePixelProcessor);
    }

    @Override
    protected void onReady(@Nullable Reference<?> selectedOpMode, Controls selectedButton) {
        addTask(() -> vision.start(purplePixelProcessor));

        // Full RoadRunner support with utility methods such as makeTrajectory()
        makeTrajectory(new Pose2d(11.40, -62.00, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(16.40, -48.10, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(11.71, -34.52, Math.toRadians(90.00)))
                .addTask();

        // Interchangeable tasks between TeleOp commands and Autonomous tasks using overloads
        // This makes tasks reusable and easier to write, where you can define your own tasks just as quick
        addTask(new MoveToContourTask<>(Seconds.of(5), drive, purplePixel, new PIDController(1, 0.25, 0)));

        addTask(arm::drop);

        switch (selectedOpMode.toString()) {
            case "PARK":
                addTask(/* ... */);
                break;
            case "GO_TWICE":
                // Convenience task groupings for allocating multiple subsystem interactions
                // while only using a singular task. Useful in command-based OpModes.
                addTask(new SequentialTaskGroup(/* ..., ..., ... */));
                break;
            case "SABOTAGE_ALLIANCE":
                // BunyipsLib offers full automatic exception catching and debug logging
                // with support for exceptions that can emergency stop an OpMode as well
                throw new EmergencyStop("You have been banished for being a traitor.");
            default:
            case "STOP":
                break;
        }
    }
} 
```

This is only a small fraction of what BunyipsLib can do. BunyipsLib is continually expanding
with newer and faster features, including full FtcDashboard-compatible Vision processing,
robust logging, and a focus on developer utilities.

![bunyipslib](https://github.com/Murray-Bridge-Bunyips/BunyipsLib/assets/81782264/8cc2ac0f-c377-4383-9c3b-b8c4815834ac)

BunyipsLib will provide the tools to get the job done quickly, no matter the skill level,
experience, or understanding. Reusability between robots is an essential design element
of BunyipsLib, to get ideas implemented faster and to write effective code.

See the [examples](./src/main/java/org/murraybridgebunyips/bunyipslib/example) for getting
familiar with BunyipsLib.  
We use BunyipsLib ourselves in both Kotlin and Java for our own
robots, see [BunyipsFTC](https://github.com/Murray-Bridge-Bunyips/BunyipsFTC/) for more examples.

Consult our [wiki](https://github.com/Murray-Bridge-Bunyips/BunyipsLib/wiki/) for more information regarding installing and getting more familiar with BunyipsLib.
___

###### BunyipsLib. Copyright (c) 2024 Lucas Bubner, MBHS Student Robotics Club, under the MIT License.<br><br>External libraries implemented/dependencies to BunyipsLib include [RoadRunner](https://github.com/acmerobotics/road-runner/tree/423c9554c97ecac249b8d07eff43235496b22f39) v0.5, WPILib's [WPIUnits](https://github.com/wpilibsuite/allwpilib/tree/0c822b45ab49b201455417d135474fc2f7b00f86/wpiunits/src/main/java/edu/wpi/first/units), [FtcDashboard](https://github.com/acmerobotics/ftc-dashboard), and [FTCLib](https://github.com/FTCLib/FTCLib)'s utilities such as PID/FF controllers and lookup tables. These library features are copyright of their respective owners.<br><br>I am not responsible for any failed autonomous, broken servos or nuclear warfare that result after the usage of BunyipsLib.<br>Test and understand your code thoroughly.
