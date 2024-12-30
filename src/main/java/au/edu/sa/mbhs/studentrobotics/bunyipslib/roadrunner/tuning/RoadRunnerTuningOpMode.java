package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.tuning;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliseconds;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.DriveModel;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.MotionProfile;
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
 * To look at details of a tuning process, follow the <a href="https://github.com/Murray-Bridge-Bunyips/BunyipsLib/wiki/RoadRunner">BunyipsLib RoadRunner tuning guide</a>.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
@SuppressWarnings("MissingJavadoc")
public abstract class RoadRunnerTuningOpMode extends LinearOpMode {
    // Intermediary fields for the tuning process. These are used to store the values of the tuning process, and are
    // initially populated with the values from the drive instance. They are used to allow for dynamic adjustment
    // of the values used in the tuning process.
    public static double tuningMotionProfile_kS = -1;
    public static double tuningMotionProfile_kV = -1;
    public static double tuningMotionProfile_kA = -1;
    public static double tuningMotionProfile_maxWheelVel = -1;
    public static double tuningMotionProfile_minProfileAccel = -1;
    public static double tuningMotionProfile_maxProfileAccel = -1;

    // Even though these three are not used in the tuning process, they are still stored here for completeness.
    public static double tuningMotionProfile_maxAngVel = -1;
    public static double tuningMotionProfile_maxAngAccel = -1;
    public static double tuningDriveModel_lateralInPerTick = -1;

    public static double tuningDriveModel_inPerTick = -1;
    public static double tuningDriveModel_trackWidthTicks = -1;

    @Nullable
    public static Double tuningMecanumGains_axialGain;
    @Nullable
    public static Double tuningMecanumGains_lateralGain;
    @Nullable
    public static Double tuningMecanumGains_headingGain;
    @Nullable
    public static Double tuningMecanumGains_axialVelGain;
    @Nullable
    public static Double tuningMecanumGains_lateralVelGain;
    @Nullable
    public static Double tuningMecanumGains_headingVelGain;

    @Nullable
    public static Double tuningTankGains_ramseteZeta;
    @Nullable
    public static Double tuningTankGains_ramseteBBar;
    @Nullable
    public static Double tuningTankGains_turnGain;
    @Nullable
    public static Double tuningTankGains_turnVelGain;

    private static int lastSelectedIndex;

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
        // Still need to send some telemetry to the dashboard, instead of using DualTelemetry which is overkill
        // for this purpose, we'll just use MultipleTelemetry.
        Telemetry fusedTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RoadRunnerDrive drive = Objects.requireNonNull(getDrive(), "getDrive() returned null");

        Field selectedIdxField;
        try {
            selectedIdxField = TelemetryMenu.class.getDeclaredField("selectedIdx");
            selectedIdxField.setAccessible(true);
        } catch (NoSuchFieldException e) {
            throw new RuntimeException("Unable to find an internal field! This shouldn't happen!");
        }

        DriveViewFactory dvf;
        if (drive instanceof MecanumDrive) {
            dvf = (h) -> {
                MecanumDrive md = (MecanumDrive) drive;
                md.setPose(Geometry.zeroPose());
                List<Encoder> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
                List<Encoder> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
                Localizer localizer = md.getLocalizer();
                if (localizer instanceof MecanumLocalizer dl) {
                    leftEncs.add(dl.leftFront);
                    leftEncs.add(dl.leftBack);
                    rightEncs.add(dl.rightFront);
                    rightEncs.add(dl.rightBack);
                } else if (localizer instanceof ThreeWheelLocalizer dl) {
                    parEncs.add(dl.par0);
                    parEncs.add(dl.par1);
                    perpEncs.add(dl.perp);
                } else if (localizer instanceof TwoWheelLocalizer dl) {
                    parEncs.add(dl.par);
                    perpEncs.add(dl.perp);
                } else {
                    throw new RuntimeException("Unknown localizer: " + localizer.getClass().getName());
                }

                // We don't want to expose the motors on the subsystem directly, we'll just use reflection here.
                // The DriveView constructor is fairly limited in what can be adjusted, including the requirement for
                // a strict LazyIMU, which is why on the drive classes only is where LazyImu instances are used.
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
                    throw new RuntimeException("Unable to access objects on MecanumDrive! This shouldn't happen!");
                }

                Constants c = md.getConstants();
                // Update only once for init to prevent re-init from rewriting
                if (tuningMotionProfile_kS == -1) {
                    tuningMotionProfile_kS = c.getMotionProfile().kS;
                    tuningMotionProfile_kV = c.getMotionProfile().kV;
                    tuningMotionProfile_kA = c.getMotionProfile().kA;
                    tuningMotionProfile_maxWheelVel = c.getMotionProfile().maxWheelVel;
                    tuningMotionProfile_minProfileAccel = c.getMotionProfile().minProfileAccel;
                    tuningMotionProfile_maxProfileAccel = c.getMotionProfile().maxProfileAccel;
                    tuningMotionProfile_maxAngVel = c.getMotionProfile().maxAngVel;
                    tuningMotionProfile_maxAngAccel = c.getMotionProfile().maxAngAccel;
                    tuningDriveModel_inPerTick = c.getDriveModel().inPerTick;
                    tuningDriveModel_lateralInPerTick = c.getDriveModel().lateralInPerTick;
                    tuningDriveModel_trackWidthTicks = c.getDriveModel().trackWidthTicks;
                }
                if (tuningMecanumGains_axialGain == null) {
                    tuningMecanumGains_axialGain = md.gains.axialGain;
                    tuningMecanumGains_lateralGain = md.gains.lateralGain;
                    tuningMecanumGains_headingGain = md.gains.headingGain;
                    tuningMecanumGains_axialVelGain = md.gains.axialVelGain;
                    tuningMecanumGains_lateralVelGain = md.gains.lateralVelGain;
                    tuningMecanumGains_headingVelGain = md.gains.headingVelGain;
                }
                Threads.startLoop("Update Tuning Parameters (Mecanum)", Milliseconds.of(500), () -> {
                    assert tuningMecanumGains_axialGain != null && tuningMecanumGains_lateralGain != null &&
                            tuningMecanumGains_headingGain != null && tuningMecanumGains_axialVelGain != null &&
                            tuningMecanumGains_lateralVelGain != null && tuningMecanumGains_headingVelGain != null;
                    md.gains.axialGain = tuningMecanumGains_axialGain;
                    md.gains.lateralGain = tuningMecanumGains_lateralGain;
                    md.gains.headingGain = tuningMecanumGains_headingGain;
                    md.gains.axialVelGain = tuningMecanumGains_axialVelGain;
                    md.gains.lateralVelGain = tuningMecanumGains_lateralVelGain;
                    md.gains.headingVelGain = tuningMecanumGains_headingVelGain;
                    DriveModel dm = md.getConstants().getDriveModel();
                    dm.inPerTick = tuningDriveModel_inPerTick;
                    dm.lateralInPerTick = tuningDriveModel_lateralInPerTick;
                    dm.trackWidthTicks = tuningDriveModel_trackWidthTicks;
                    MotionProfile mp = md.getConstants().getMotionProfile();
                    mp.kS = tuningMotionProfile_kS;
                    mp.kV = tuningMotionProfile_kV;
                    mp.kA = tuningMotionProfile_kA;
                    mp.maxWheelVel = tuningMotionProfile_maxWheelVel;
                    mp.minProfileAccel = tuningMotionProfile_minProfileAccel;
                    mp.maxProfileAccel = tuningMotionProfile_maxProfileAccel;
                    mp.maxAngVel = tuningMotionProfile_maxAngVel;
                    mp.maxAngAccel = tuningMotionProfile_maxAngAccel;
                });
                assert lazyImu != null;
                return new DriveView(
                        DriveType.MECANUM,
                        tuningDriveModel_inPerTick,
                        tuningMotionProfile_maxWheelVel,
                        tuningMotionProfile_minProfileAccel,
                        tuningMotionProfile_maxProfileAccel,
                        h.getAll(LynxModule.class),
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
                        h.voltageSensor.iterator().next(),
                        () -> new MotorFeedforward(tuningMotionProfile_kS,
                                tuningMotionProfile_kV / tuningDriveModel_inPerTick,
                                tuningMotionProfile_kA / tuningDriveModel_inPerTick)
                );
            };
        } else if (drive instanceof TankDrive) {
            dvf = (h) -> {
                TankDrive td = (TankDrive) drive;
                td.setPose(Geometry.zeroPose());
                List<Encoder> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
                List<Encoder> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
                Localizer localizer = td.getLocalizer();
                if (localizer instanceof TankLocalizer dl) {
                    leftEncs.addAll(dl.leftEncs);
                    rightEncs.addAll(dl.rightEncs);
                } else if (localizer instanceof ThreeWheelLocalizer dl) {
                    parEncs.add(dl.par0);
                    parEncs.add(dl.par1);
                    perpEncs.add(dl.perp);
                } else if (localizer instanceof TwoWheelLocalizer dl) {
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
                    throw new RuntimeException("Unable to access the objects on TankDrive! This shouldn't happen!");
                }

                Constants c = drive.getConstants();
                if (tuningMotionProfile_kS == -1) {
                    tuningMotionProfile_kS = c.getMotionProfile().kS;
                    tuningMotionProfile_kV = c.getMotionProfile().kV;
                    tuningMotionProfile_kA = c.getMotionProfile().kA;
                    tuningMotionProfile_maxWheelVel = c.getMotionProfile().maxWheelVel;
                    tuningMotionProfile_minProfileAccel = c.getMotionProfile().minProfileAccel;
                    tuningMotionProfile_maxProfileAccel = c.getMotionProfile().maxProfileAccel;
                    tuningMotionProfile_maxAngVel = c.getMotionProfile().maxAngVel;
                    tuningMotionProfile_maxAngAccel = c.getMotionProfile().maxAngAccel;
                    tuningDriveModel_inPerTick = c.getDriveModel().inPerTick;
                    tuningDriveModel_trackWidthTicks = c.getDriveModel().trackWidthTicks;
                }
                if (tuningTankGains_ramseteZeta == null) {
                    tuningTankGains_ramseteZeta = td.gains.ramseteZeta;
                    tuningTankGains_ramseteBBar = td.gains.ramseteBBar;
                    tuningTankGains_turnGain = td.gains.turnGain;
                    tuningTankGains_turnVelGain = td.gains.turnVelGain;
                }
                Threads.startLoop("Update Tuning Parameters (Tank)", Milliseconds.of(500), () -> {
                    assert tuningTankGains_ramseteZeta != null && tuningTankGains_ramseteBBar != null &&
                            tuningTankGains_turnGain != null && tuningTankGains_turnVelGain != null;
                    td.gains.ramseteZeta = tuningTankGains_ramseteZeta;
                    td.gains.ramseteBBar = tuningTankGains_ramseteBBar;
                    td.gains.turnGain = tuningTankGains_turnGain;
                    td.gains.turnVelGain = tuningTankGains_turnVelGain;
                    DriveModel dm = td.getConstants().getDriveModel();
                    dm.inPerTick = tuningDriveModel_inPerTick;
                    dm.trackWidthTicks = tuningDriveModel_trackWidthTicks;
                    MotionProfile mp = td.getConstants().getMotionProfile();
                    mp.kS = tuningMotionProfile_kS;
                    mp.kV = tuningMotionProfile_kV;
                    mp.kA = tuningMotionProfile_kA;
                    mp.maxWheelVel = tuningMotionProfile_maxWheelVel;
                    mp.minProfileAccel = tuningMotionProfile_minProfileAccel;
                    mp.maxProfileAccel = tuningMotionProfile_maxProfileAccel;
                    mp.maxAngVel = tuningMotionProfile_maxAngVel;
                    mp.maxAngAccel = tuningMotionProfile_maxAngAccel;
                });
                assert leftMotors != null && rightMotors != null && lazyImu != null;
                return new DriveView(
                        DriveType.TANK,
                        tuningDriveModel_inPerTick,
                        tuningMotionProfile_maxWheelVel,
                        tuningMotionProfile_minProfileAccel,
                        tuningMotionProfile_maxProfileAccel,
                        h.getAll(LynxModule.class),
                        leftMotors,
                        rightMotors,
                        leftEncs,
                        rightEncs,
                        parEncs,
                        perpEncs,
                        lazyImu,
                        h.voltageSensor.iterator().next(),
                        () -> new MotorFeedforward(tuningMotionProfile_kS,
                                tuningMotionProfile_kV / tuningDriveModel_inPerTick,
                                tuningMotionProfile_kA / tuningDriveModel_inPerTick)
                );
            };
        } else {
            throw new RuntimeException("Unknown RoadRunnerDrive type!");
        }

        ForwardRampLogger.POWER_PER_SEC = 0.2;
        LateralRampLogger.POWER_PER_SEC = 0.2;
        ManualFeedforwardTuner.DISTANCE = 96;

        TelemetryMenu.MenuElement root = new TelemetryMenu.MenuElement("RoadRunner Tuning Selection", true);
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
                configRoot.putVariable("[RR] " + c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });

        TelemetryMenu.StaticClickableOption[] opModes = new TelemetryMenu.StaticClickableOption[modes.length];
        // Have to use array access due to inner class variable mutation
        Object[] selection = {null};
        for (int i = 0; i < modes.length; i++) {
            // Must be considered final as it is used in the inner class
            int finalI = i;
            opModes[i] = new TelemetryMenu.StaticClickableOption(modes[finalI].getClass().getSimpleName(),
                    () -> selection[0] = modes[finalI]);
        }
        root.addChildren(opModes);
        TelemetryMenu menu = new TelemetryMenu(fusedTelemetry, root);
        try {
            selectedIdxField.setInt(menu, lastSelectedIndex);
        } catch (IllegalAccessException e) {
            throw new RuntimeException("Access exception! This shouldn't happen!");
        }

        while (selection[0] == null && opModeInInit()) {
            menu.loop(gamepad1);
            fusedTelemetry.update();
            try {
                lastSelectedIndex = selectedIdxField.getInt(menu);
            } catch (IllegalAccessException e) {
                throw new RuntimeException("Access exception! This shouldn't happen!");
            }
        }

        fusedTelemetry.clearAll();
        if (selection[0] == null) {
            // Force select current option if none was selected
            selection[0] = modes[lastSelectedIndex];
        }

        try {
            //noinspection ExtractMethodRecommender
            LinearOpMode opMode = (LinearOpMode) selection[0];
            try {
                //noinspection DataFlowIssue
                Class<?> opModeInternal = getClass().getSuperclass().getSuperclass().getSuperclass().getSuperclass();
                //noinspection DataFlowIssue
                Field started = opModeInternal.getDeclaredField("isStarted");
                // We need to force start the OpMode that will execute, although we miss the init step we can wait
                // here instead.
                started.setAccessible(true);
                started.set(opMode, true);
            } catch (NoSuchFieldException | IllegalAccessException | NullPointerException e) {
                throw new RuntimeException("Internal error while starting OpMode. This shouldn't happen!");
            }
            opMode.gamepad1 = gamepad1;
            opMode.gamepad2 = gamepad2;
            opMode.hardwareMap = hardwareMap;
            if (opMode instanceof ForwardPushTest || opMode instanceof LateralPushTest) {
                // These OpModes report telemetry but not to the dashboard for whatever reason, so we'll just use
                // our own fused telemetry instance.
                opMode.telemetry = fusedTelemetry;
            } else {
                opMode.telemetry = telemetry;
            }

            fusedTelemetry.setCaptionValueSeparator(": ");
            fusedTelemetry.addData(Text.html().bold(selection[0].getClass().getSimpleName()).toString(), "Ready.");
            fusedTelemetry.update();

            FtcDashboard.getInstance().withConfigRoot(configRoot ->
                    configRoot.putVariable("[RR] Tuning Parameters", ReflectionConfig.createVariableFromClass(getClass())));

            waitForStart();
            if (isStopRequested())
                return;

            // Quickly construct the DriveView even for OpModes that don't need it, though still required for FtcDashboard tuning.
            // This mostly applies to the ManualFeedbackTuner and allows the constants on the drive to be adjusted.
            if (opMode instanceof ManualFeedbackTuner || opMode instanceof SplineTest || opMode instanceof LocalizationTest) {
                // Ensures the update loop is started for the gains
                dvf.make(hardwareMap);
            }

            opMode.runOpMode();
        } finally {
            Threads.stopAll();
        }
    }
}
