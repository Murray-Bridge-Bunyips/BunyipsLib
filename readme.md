![BunyipsLib)](https://github.com/Murray-Bridge-Bunyips/.github/blob/main/bunyipslib_banner.png)
##### A powerful all-in-one library for FTC used by the Murray Bridge High School Student Robotics Club.

BunyipsLib was originally a collection of common functions that our FTC teams used in competition.
It has since evolved into a full-fledged library that is used by all of our teams. It is designed to
provide useful development utilities in making robot code, offering efficient common modules and OpMode abstractions with
error handling, dynamic device initialisation, native RoadRunner, FtcDashboard, and Vision processing, and an
extremely expansible ecosystem.

BunyipsLib takes inspiration from FTCLib, WPILib, RoadRunner, and other libraries, and is designed to be
the one-stop shop for all of our teams' needs. It is designed to be easy to use, and to integrate
the latest and greatest features from the FTC SDK and other libraries.

BunyipsLib is in active and continually experimental development, expect bugs and to keep up to date!

## Features
BunyipsLib plans to make OpMode development simpler and easier. This includes integrated exception catching with a similar ecosystem
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

Subsystems are declared expressively, allowing a transitionary command-based system with procedural code.
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
        cannon = new Cannon(...);
        arm = new MyArm(...);
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

        if (gamepad1.right_trigger == 1.0) {
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

Are you more of a fan of full-command-based systems? We've brewed our own command scheduler with full integrations with our ecosystem.
```java
@TeleOp(name = "Align To Pixel (Command Based)")
public class GLaDOSCommandBasedAlignToPixelTest extends CommandBasedBunyipsOpMode {
    private final MyBotConfig config = new MyBotConfig();

    private MecanumDrive drive;
    private Vision vision;
    private MultiYCbCrThreshold pixels;

    @Override
    protected void onInitialisation() {
        config.init();

        drive = new DualDeadwheelMecanumDrive(...);
        vision = new Vision(...);

        pixels = new MultiYCbCrThreshold(Pixels.createProcessors());
        vision.init(pixels, vision.raw);
        vision.start(pixels, vision.raw);

        // Full FtcDashboard integrations for individual processor stream previews
        vision.startPreview();
    }

    @Override
    protected BunyipsSubsystem[] setSubsystems() {
        return new BunyipsSubsystem[] {
                drive,
        };
    }

    @Override
    protected void assignCommands() {
        drive.setDefaultTask(new HolonomicDriveTask<>(gamepad1, drive, () -> false));

        scheduler().whenHeld(Controller.User.ONE, Controller.Y)
                .run(new InstantTask(() -> drive.resetYaw()))
                .immediately();

        scheduler().whenPressed(Controller.User.ONE, Controller.RIGHT_BUMPER)
                .run(new AlignToPixelTask<>(gamepad1, drive, pixels, new PIDController(1, 0.25, 0.0)))
                .finishingWhen(() -> !gamepad1.right_bumper);
    }
}
```
BunyipsLib has a multi-purpose Task system, not only for commands but for actionable tasks to do in Autonomous.
```java
public class RunArmFor extends Task {
    private final MyArm arm;
    private final double power;

    public RunArmFor(double time, MyArm arm, double power) {
        super(time); // Built-in timeout management
        this.arm = arm;
        this.power = power;
    }

    @Override
    public void init() {
        arm.setPower(power);
    }

    @Override
    public void periodic() {
        // Full telemetry utilities for FtcDashboard and Driver Station
        opMode.addTelemetry("Arm is at % ticks.", arm.getTicks());
        arm.update();
    }

    @Override
    public void onFinish() {
        arm.stopNow();
    }

    @Override
    public boolean isTaskFinished() {
        return arm.calculatePosition() > 400;
    }
}
```
All with full RoadRunner utilities, runtime OpMode selectors, and much more.
```java
@Autonomous(name = "Place a Pixel")
public class PlacePixel extends RoadRunnerAutonomousBunyipsOpMode<MecanumDrive> {
    private final MyBotConfig config = new MyBotConfig();

    private MyArm arm;
    private Vision vision;
    private PurplePixel purplePixelProcessor;
    private WhitePixel whitePixelProcessor;

    private YourInitTask initTask;

    @Override
    protected void onInitialise() {
        config.init();

        vision = new Vision(...);
        arm = new MyArm(...);

        vision.init(purplePixelProcessor, whitePixelProcessor);
        vision.start(whitePixelProcessor);

        // Optionally send all feeds to FtcDashboard, can switch between processor feeds for debugging
        // vision.startPreview();

        addTelemetry("Hello world!"); // Full telemetry utilities with FtcDashboard
        addRetainedTelemetry("Greetings, world.");
    }

    @Override
    protected MecanumDrive setDrive() {
        return new DualDeadwheelMecanumDrive(...);
    }

    @Override
    protected List<OpModeSelection> setOpModes() {
        // Full support for any object to use as a selection, including enums
        // This will be automatically shown on the init-cycle.
        return new Arrays.asList(
                new OpModeSelection("PARK"),
                new OpModeSelection("GO_AGAIN"),
                new OpModeSelection("SABOTAGE_ALLIANCE")
        );
    }

    @Override
    protected RobotTask setInitTask() {
        return initTask;
    }

    @Override
    protected void onInitDone() {
        switch (initTask.getSpikeResult()) {
            case LEFT:
                // RoadRunner trajectory to get to the Spike Mark
                addNewTrajectory(...);
                break;
            // ...
        }
        vision.stop(whitePixelProcessor);
    }

    @Override
    protected void onQueueReady(@Nullable OpModeSelection selectedOpMode) {
        addTask(new InstantTask(() -> vision.start(purplePixelProcessor));

        // Full RoadRunner support with utility methods such as addNewTrajectory()
        addNewTrajectory(new Pose2d(11.40, -62.00, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(16.40, -48.10, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(11.71, -34.52, Math.toRadians(90.00)))
                .build();

        // Interchangeable tasks between TeleOp commands and Autonomous tasks using overloads
        // This makes tasks reusable and easier to write, where you can define your own tasks just as quick
        addTask(new MoveToPixelTask<>(5, drive, purplePixel, new PIDController(1, 0.25, 0)));

        addTask(new InstantTask(() -> arm.drop());

        switch (selectedOpMode.getName()) {
            case "PARK":
                addTask(...);
                break;
            case "GO_AGAIN":
                addTask(new SequentialTaskGroup(..., ..., ...));
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

BunyipsLib will provide the tools to get the job done quickly, no matter the skill level,
experience, or understanding. Reusability between robots is an essential design element
of BunyipsLib, to get ideas implemented faster and to write effective code.

See the [examples](./src/main/java/org/murraybridgebunyips/bunyipslib/example) for getting
familiar with BunyipsLib.  
We use BunyipsLib ourselves in both Kotlin and Java for our own
robots, see [BunyipsFTC](https://github.com/Murray-Bridge-Bunyips/BunyipsFTC/) for more examples.

## Using BunyipsLib for yourself
BunyipsLib is distributed via a `git submodule`. This allows for direct source code modification by you, as BunyipsLib should work harmoniously with your codebase.

To get BunyipsLib in your codebase, you'll need experience with `git`, the Gradle build system, and the command line (used in this tutorial).

### Integrating BunyipsLib into your codebase
1. **Getting the code**  
From the top-level directory in your own FtcRobotController fork, run `git submodule add https://github.com/Murray-Bridge-Bunyips/BunyipsLib`, which should clone and create a new BunyipsLib folder with the source code.<br><br>
2. **Adding dependencies**  
BunyipsLib depends on RoadRunner, FtcDashboard, and Kotlin. These tools make development faster and are legal FTC tools.  
You will first need to install *Kotlin v1.9.20* using the guide provided by FIRST: https://ftc-docs.firstinspires.org/en/latest/programming_resources/shared/installing_kotlin/Installing-Kotlin.html<br><br>

After you've done this, go to your `build.dependencies.gradle` file located in your top-level directory and paste the following dependencies in the `dependencies {}` block:
```gradle
implementation 'com.acmerobotics.dashboard:dashboard:0.4.12'
implementation 'org.apache.commons:commons-math3:3.6.1'
implementation 'com.fasterxml.jackson.core:jackson-databind:2.12.7'
implementation 'com.acmerobotics.roadrunner:core:0.5.6'
```

Finally, add this to the top `repositories {}` block in the same file:
```gradle
maven { url = 'https://maven.brott.dev/' }
```
Your dependencies file should look something like [this.](https://github.com/Murray-Bridge-Bunyips/BunyipsFTC/blob/stable/build.dependencies.gradle)<br><br>
5. **Telling Gradle BunyipsLib exists**  
First, we need tell Gradle to build BunyipsLib as well. To do this, open `settings.gradle` in your top-level directory and append the following lines.
```
include ':BunyipsLib'
```
Finally, we need to tell Gradle that we want to integrate BunyipsLib with the TeamCode module. Open the `build.gradle` file associated with the TeamCode module, and append to the `dependencies {}` block:
```
implementation project(':BunyipsLib')
```
5. **Gradle sync**  
In Android Studio, run a Gradle Sync (Ctrl+Shift+O) to ensure your classpaths are updated with BunyipsLib and your dependencies are downloaded.<br><br>
6. **You're done!**  
You now have access to BunyipsLib. Ensure to read how to update BunyipsLib and how to ensure your future cloners can get BunyipsLib in their local repositories.

### Using the BunyipsLib submodule in a clone of your fork
For new cloners of your fork with BunyipsLib, you can simply run `git clone --recurse-submodules <YOUR_REPO>`.  <br><br>
For repositories that already exist, run `git submodule update --init --recursive`, to fetch the submodule that was placed in your `.gitmodules` top-level directory file.

### Updating BunyipsLib
BunyipsLib is continually getting updates to ensure the latest features have been rigorously debugged and tested. To ensure that you *aren't* using code that I've accidentally implemented a 30-gigabyte memory leak into, you'll need to know how to update it.

1. `git pull`  
Navigate to the directory of your BunyipsLib submodule, and run `git pull` on the `master` branch. This should bring any changes into your repository.

###### BunyipsLib. Copyright (c) 2024 Lucas Bubner, MBHS Student Robotics Club, under the MIT License. <br><br>I am not responsible for any failed autonomous, broken servos or nuclear warfare that result after the usage of BunyipsLib.<br>Test and understand your code thoroughly.
