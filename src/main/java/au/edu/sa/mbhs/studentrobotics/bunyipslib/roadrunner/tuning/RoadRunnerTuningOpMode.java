package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.tuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.ftc.AngularRampLogger;
import com.acmerobotics.roadrunner.ftc.DeadWheelDirectionDebugger;
import com.acmerobotics.roadrunner.ftc.DriveType;
import com.acmerobotics.roadrunner.ftc.DriveView;
import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.ForwardPushTest;
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger;
import com.acmerobotics.roadrunner.ftc.LateralPushTest;
import com.acmerobotics.roadrunner.ftc.LateralRampLogger;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner;
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsOpMode;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.DualTelemetry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.TelemetryMenu;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.Localizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.MecanumLocalizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.TankLocalizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.ThreeWheelLocalizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.TwoWheelLocalizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.accumulators.Accumulator;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.RoadRunnerDrive;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.Constants;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.MecanumDrive;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.TankDrive;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Threads;

/**
 * Utility OpMode you will extend to enable an OpMode to tune your RoadRunner coefficients and parameters. This uses
 * Telemetry to show a menu of tuning options, with static fields being adjusted here, essentially collecting all
 * RoadRunner tuning into one OpMode. You may choose to run the OpModes yourself as they consume the instances you
 * provide, but using this class makes it more straightforward and has built-in dashboard tuning. It is not recommended
 * you run the tuning OpModes yourself, and instead run them using this class.
 * <p>
 * To use this class, extend it, supply an appropriately configured instance of your drive, and ensure the localizer
 * you wish to use has been set for it. It is not recommended to use an alternate Accumulator while tuning.
 * Then, treat is like a normal OpMode ({@link TeleOp} annotation).
 * You can adjust all the constants used in testing via FtcDashboard, nested under this class definition.
 * <p>
 * Telemetry is mirrored between FtcDashboard and the Driver Station using a {@link DualTelemetry} instance, the same
 * used in {@link BunyipsOpMode}. It is not required that a Driver Station be active for tuning.
 * <p>
 * To look at details of a tuning process, follow the <a href="https://rr.brott.dev/docs/v1-0/tuning/">tuning guide for RoadRunner v1.0</a>.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
@Config
public abstract class RoadRunnerTuningOpMode extends LinearOpMode {
    /**
     * Instantiate hardware and return the fully configured RoadRunner drive instance to use for tuning.
     * <p>
     * It is recommended to use the default {@link Accumulator}, however, setting the used {@link Localizer}
     * is required for tuning a {@link TwoWheelLocalizer} or {@link ThreeWheelLocalizer}.
     *
     * @return the configured drive instance to use for tuning
     */
    @NonNull
    protected abstract RoadRunnerDrive getDrive();

    @Override
    @SuppressWarnings("unchecked")
    public final void runOpMode() throws InterruptedException {
        DualTelemetry out = new DualTelemetry(this, null, "RR Tuning");
        RoadRunnerDrive drive = Objects.requireNonNull(getDrive(), "getDrive() returned null");

        DriveViewFactory dvf;
        if (drive instanceof MecanumDrive) {
            dvf = (h) -> {
                MecanumDrive md = (MecanumDrive) drive;
                md.setPose(Geometry.zeroPose());
                List<Encoder> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
                List<Encoder> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
                Localizer localizer = md.getLocalizer();
                if (localizer instanceof MecanumLocalizer) {
                    MecanumLocalizer dl = (MecanumLocalizer) localizer;
                    leftEncs.add(dl.leftFront);
                    leftEncs.add(dl.leftBack);
                    rightEncs.add(dl.rightFront);
                    rightEncs.add(dl.rightBack);
                } else if (localizer instanceof ThreeWheelLocalizer) {
                    ThreeWheelLocalizer dl = (ThreeWheelLocalizer) localizer;
                    parEncs.add(dl.par0);
                    parEncs.add(dl.par1);
                    perpEncs.add(dl.perp);
                } else if (localizer instanceof TwoWheelLocalizer) {
                    TwoWheelLocalizer dl = (TwoWheelLocalizer) localizer;
                    parEncs.add(dl.par);
                    perpEncs.add(dl.perp);
                } else {
                    throw new RuntimeException("Unknown localizer: " + localizer.getClass().getName());
                }

                // We don't want to expose the motors on the subsystem directly, we'll just use reflection here.
                // The DriveView constructor is fairly limited in what can be adjusted, including the requirement for
                // a strict LazyIMU when it is locked to using hardwareMap. This explains why a separate IMU config
                // is used.
                Class<?> mdClass = md.getClass();
                Field leftFrontField, leftBackField, rightFrontField, rightBackField, lazyImuField;
                try {
                    leftFrontField = mdClass.getDeclaredField("leftFront");
                    leftBackField = mdClass.getDeclaredField("leftBack");
                    rightFrontField = mdClass.getDeclaredField("rightFront");
                    rightBackField = mdClass.getDeclaredField("rightBack");
                    lazyImuField = mdClass.getDeclaredField("lazyImu");
                } catch (NoSuchFieldException e) {
                    throw new RuntimeException("MecanumDrive is missing fields for motors or IMU! This shouldn't happen!");
                }
                leftFrontField.setAccessible(true);
                leftBackField.setAccessible(true);
                rightFrontField.setAccessible(true);
                rightBackField.setAccessible(true);
                lazyImuField.setAccessible(true);
                DcMotorEx leftFront, leftBack, rightFront, rightBack;
                LazyImu lazyImu;
                try {
                    leftFront = (DcMotorEx) leftFrontField.get(md);
                    leftBack = (DcMotorEx) leftBackField.get(md);
                    rightFront = (DcMotorEx) rightFrontField.get(md);
                    rightBack = (DcMotorEx) rightBackField.get(md);
                    lazyImu = (LazyImu) lazyImuField.get(md);
                } catch (IllegalAccessException e) {
                    throw new RuntimeException("Unable to access objects on MecanumDrive!");
                }

                Constants c = md.getConstants();
                assert lazyImu != null;
                return new DriveView(
                        DriveType.MECANUM,
                        c.getDriveModel().inPerTick,
                        c.getMotionProfile().maxWheelVel,
                        c.getMotionProfile().minProfileAccel,
                        c.getMotionProfile().maxProfileAccel,
                        hardwareMap.getAll(LynxModule.class),
                        Arrays.asList(
                                leftFront,
                                leftBack
                        ),
                        Arrays.asList(
                                rightFront,
                                rightBack
                        ),
                        leftEncs,
                        rightEncs,
                        parEncs,
                        perpEncs,
                        lazyImu,
                        hardwareMap.voltageSensor.iterator().next(),
                        () -> new MotorFeedforward(c.getMotionProfile().kS,
                                c.getMotionProfile().kV / c.getDriveModel().inPerTick,
                                c.getMotionProfile().kA / c.getDriveModel().inPerTick)
                );
            };
        } else if (drive instanceof TankDrive) {
            dvf = (h) -> {
                TankDrive td = (TankDrive) drive;
                td.setPose(Geometry.zeroPose());
                List<Encoder> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
                List<Encoder> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
                Localizer localizer = td.getLocalizer();
                if (localizer instanceof TankLocalizer) {
                    TankLocalizer dl = (TankLocalizer) localizer;
                    leftEncs.addAll(dl.leftEncs);
                    rightEncs.addAll(dl.rightEncs);
                } else if (localizer instanceof ThreeWheelLocalizer) {
                    ThreeWheelLocalizer dl = (ThreeWheelLocalizer) localizer;
                    parEncs.add(dl.par0);
                    parEncs.add(dl.par1);
                    perpEncs.add(dl.perp);
                } else if (localizer instanceof TwoWheelLocalizer) {
                    TwoWheelLocalizer dl = (TwoWheelLocalizer) localizer;
                    parEncs.add(dl.par);
                    perpEncs.add(dl.perp);
                } else {
                    throw new RuntimeException("Unknown localizer: " + localizer.getClass().getName());
                }

                Class<?> tdClass = td.getClass();
                Field leftMotorsField, rightMotorsField, lazyImuField;
                try {
                    leftMotorsField = tdClass.getDeclaredField("leftMotors");
                    rightMotorsField = tdClass.getDeclaredField("rightMotors");
                    lazyImuField = tdClass.getDeclaredField("lazyImu");
                } catch (NoSuchFieldException e) {
                    throw new RuntimeException("TankDrive is missing fields for motors or IMU! This shouldn't happen!");
                }
                leftMotorsField.setAccessible(true);
                rightMotorsField.setAccessible(true);
                lazyImuField.setAccessible(true);
                List<DcMotorEx> leftMotors, rightMotors;
                LazyImu lazyImu;
                try {
                    leftMotors = (List<DcMotorEx>) leftMotorsField.get(td);
                    rightMotors = (List<DcMotorEx>) rightMotorsField.get(td);
                    lazyImu = (LazyImu) lazyImuField.get(td);
                } catch (IllegalAccessException e) {
                    throw new RuntimeException("Unable to access the objects on TankDrive!");
                }

                Constants c = drive.getConstants();
                assert leftMotors != null && rightMotors != null && lazyImu != null;
                return new DriveView(
                        DriveType.TANK,
                        c.getDriveModel().inPerTick,
                        c.getMotionProfile().maxWheelVel,
                        c.getMotionProfile().minProfileAccel,
                        c.getMotionProfile().maxProfileAccel,
                        hardwareMap.getAll(LynxModule.class),
                        leftMotors,
                        rightMotors,
                        leftEncs,
                        rightEncs,
                        parEncs,
                        perpEncs,
                        lazyImu,
                        hardwareMap.voltageSensor.iterator().next(),
                        () -> new MotorFeedforward(c.getMotionProfile().kS,
                                c.getMotionProfile().kV / c.getDriveModel().inPerTick,
                                c.getMotionProfile().kA / c.getDriveModel().inPerTick)
                );
            };
        } else {
            throw new RuntimeException("Unknown RoadRunnerDrive type!");
        }

        TelemetryMenu.MenuElement root = new TelemetryMenu.MenuElement("Tuning OpMode Selection", true);
        LinearOpMode[] modes = {
                new AngularRampLogger(dvf),
                new ForwardPushTest(dvf),
                new ForwardRampLogger(dvf),
                new LateralPushTest(dvf),
                new LateralRampLogger(dvf),
                new ManualFeedforwardTuner(dvf),
                new MecanumMotorDirectionDebugger(dvf),
                new DeadWheelDirectionDebugger(dvf),
                new ManualFeedbackTuner(drive),
                new SplineTest(drive),
                new LocalizationTest(drive)
        };

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                    AngularRampLogger.class,
                    ForwardRampLogger.class,
                    LateralRampLogger.class,
                    ManualFeedforwardTuner.class,
                    MecanumMotorDirectionDebugger.class,
                    ManualFeedbackTuner.class
            )) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });

        TelemetryMenu.StaticClickableOption[] opModes = new TelemetryMenu.StaticClickableOption[modes.length];
        // Have to use array access due to inner class variable mutation
        Object[] selection = {null};
        for (int i = 0; i < modes.length; i++) {
            // Must be considered final as it is used in the inner class
            int finalI = i;
            opModes[i] = new TelemetryMenu.StaticClickableOption(modes[finalI].getClass().getSimpleName()) {
                @Override
                protected void onClick() {
                    selection[0] = modes[finalI];
                }
            };
        }
        root.addChildren(opModes);
        TelemetryMenu menu = new TelemetryMenu(out, root);

        while (selection[0] == null && opModeInInit()) {
            menu.loop(gamepad1);
            out.add("Select an option above to run tuning for using <font face='monospace'>gamepad1</font>. Restart the OpMode to pick a different mode.");
            out.update();
        }

        out.clearAll();
        if (selection[0] == null) return;

        // Defer the OpMode to the tuning OpMode now, it is confirmed to be safe to cast
        out.setOpModeStatus(Text.html().bold(selection[0].getClass().getSimpleName()).toString());
        out.update();
        try {
            // We're delegating the OpMode, however, runBlocking calls do not know that motor powers must be updated,
            // so we call periodic in the background to mitigate this.
            Threads.startLoop(drive::periodic, "RR Tuning Drive Update Executor");
            ((LinearOpMode) selection[0]).runOpMode();
        } finally {
            Threads.stopAll();
        }
    }
}
