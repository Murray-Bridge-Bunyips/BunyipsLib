package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.tuning;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliseconds;

import android.graphics.Color;

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
import com.acmerobotics.roadrunner.ftc.EncoderGroup;
import com.acmerobotics.roadrunner.ftc.EncoderRef;
import com.acmerobotics.roadrunner.ftc.ForwardPushTest;
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger;
import com.acmerobotics.roadrunner.ftc.LateralPushTest;
import com.acmerobotics.roadrunner.ftc.LateralRampLogger;
import com.acmerobotics.roadrunner.ftc.LynxQuadratureEncoderGroup;
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner;
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger;
import com.acmerobotics.roadrunner.ftc.OTOSAngularScalarTuner;
import com.acmerobotics.roadrunner.ftc.OTOSEncoderGroup;
import com.acmerobotics.roadrunner.ftc.OTOSHeadingOffsetTuner;
import com.acmerobotics.roadrunner.ftc.OTOSIMU;
import com.acmerobotics.roadrunner.ftc.OTOSLinearScalarTuner;
import com.acmerobotics.roadrunner.ftc.OTOSPositionOffsetTuner;
import com.acmerobotics.roadrunner.ftc.PinpointEncoderGroup;
import com.acmerobotics.roadrunner.ftc.PinpointIMU;
import com.acmerobotics.roadrunner.ftc.PinpointView;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsOpMode;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.DualTelemetry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.TelemetryMenu;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.Localizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.MecanumLocalizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.OTOSLocalizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.PinpointLocalizer;
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
import dev.frozenmilk.util.cell.LateInitCell;

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
 * As of BunyipsLib v7.0.0, tuning is now supported for the goBILDA® Pinpoint Computer and SparkFun OTOS.
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
     * is required for tuning a {@link TwoWheelLocalizer}, {@link ThreeWheelLocalizer}, {@link OTOSLocalizer},
     * or {@link PinpointLocalizer}.
     *
     * @return the configured drive instance to use for tuning
     */
    @NonNull
    protected abstract RoadRunnerDrive getDrive();

    @Override
    @SuppressWarnings("unchecked")
    public final void runOpMode() throws InterruptedException {
        // We are not a system OpMode so lights will be automatically cleaned up on completion
        hardwareMap.getAll(LynxModule.class).forEach(h -> h.setPattern(Arrays.asList(
                new Blinker.Step(Color.GREEN, 400, TimeUnit.MILLISECONDS),
                new Blinker.Step(Color.LTGRAY, 400, TimeUnit.MILLISECONDS)
        )));

        // Still need to send some telemetry to the dashboard, instead of using DualTelemetry which is overkill
        // for this purpose, we'll just use MultipleTelemetry.
        Telemetry fusedTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RoadRunnerDrive drive = Objects.requireNonNull(getDrive(), "getDrive() returned null");

        Field selectedIdxField;
        try {
            selectedIdxField = TelemetryMenu.class.getDeclaredField("selectedIdx");
            selectedIdxField.setAccessible(true);
        } catch (NoSuchFieldException e) {
            throw new RuntimeException("Unable to find an internal field! This shouldn't happen!", e);
        }

        List<EncoderGroup> encoderGroups = new ArrayList<>();
        List<EncoderRef> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
        List<EncoderRef> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
        Localizer localizer = drive.getLocalizer();
        LateInitCell<IMU> imu = new LateInitCell<>();
        if (localizer instanceof MecanumLocalizer dl) {
            encoderGroups.add(new LynxQuadratureEncoderGroup(
                    hardwareMap.getAll(LynxModule.class),
                    Arrays.asList(dl.leftFront, dl.leftBack, dl.rightFront, dl.rightBack)
            ));
            leftEncs.add(new EncoderRef(0, 0));
            leftEncs.add(new EncoderRef(0, 1));
            rightEncs.add(new EncoderRef(0, 2));
            rightEncs.add(new EncoderRef(0, 3));
        } else if (localizer instanceof TankLocalizer tl) {
            List<Encoder> allEncoders = new ArrayList<>();
            allEncoders.addAll(tl.leftEncs);
            allEncoders.addAll(tl.rightEncs);
            encoderGroups.add(new LynxQuadratureEncoderGroup(
                    hardwareMap.getAll(LynxModule.class),
                    allEncoders
            ));
            for (int i = 0; i < tl.leftEncs.size(); i++) {
                leftEncs.add(new EncoderRef(0, i));
            }
            for (int i = 0; i < tl.rightEncs.size(); i++) {
                rightEncs.add(new EncoderRef(0, tl.leftEncs.size() + i));
            }
        } else if (localizer instanceof ThreeWheelLocalizer dl) {
            encoderGroups.add(new LynxQuadratureEncoderGroup(
                    hardwareMap.getAll(LynxModule.class),
                    Arrays.asList(dl.par0, dl.par1, dl.perp)
            ));
            parEncs.add(new EncoderRef(0, 0));
            parEncs.add(new EncoderRef(0, 1));
            perpEncs.add(new EncoderRef(0, 2));
        } else if (localizer instanceof TwoWheelLocalizer dl) {
            encoderGroups.add(new LynxQuadratureEncoderGroup(
                    hardwareMap.getAll(LynxModule.class),
                    Arrays.asList(dl.par, dl.perp)
            ));
            parEncs.add(new EncoderRef(0, 0));
            perpEncs.add(new EncoderRef(0, 1));
        } else if (localizer instanceof OTOSLocalizer ol) {
            assert ol.otos != null;
            encoderGroups.add(new OTOSEncoderGroup(ol.otos));
            parEncs.add(new EncoderRef(0, 0));
            perpEncs.add(new EncoderRef(0, 1));
            imu.accept(new OTOSIMU(ol.otos));
        } else if (localizer instanceof PinpointLocalizer pl) {
            PinpointView pv = new PinpointView() {
                GoBildaPinpointDriver.EncoderDirection parDirection = pl.params.initialParDirection;
                GoBildaPinpointDriver.EncoderDirection perpDirection = pl.params.initialPerpDirection;

                @Override
                public void update() {
                    assert pl.pinpoint != null;
                    pl.pinpoint.update();
                }

                @Override
                public int getParEncoderPosition() {
                    assert pl.pinpoint != null;
                    return pl.pinpoint.getEncoderX();
                }

                @Override
                public int getPerpEncoderPosition() {
                    assert pl.pinpoint != null;
                    return pl.pinpoint.getEncoderY();
                }

                @Override
                public float getHeadingVelocity(@NonNull UnnormalizedAngleUnit unit) {
                    assert pl.pinpoint != null;
                    return (float) pl.pinpoint.getHeadingVelocity(unit);
                }

                @NonNull
                @Override
                public DcMotorSimple.Direction getParDirection() {
                    return parDirection == GoBildaPinpointDriver.EncoderDirection.FORWARD ?
                            DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
                }

                @Override
                public void setParDirection(@NonNull DcMotorSimple.Direction direction) {
                    parDirection = direction == DcMotorSimple.Direction.FORWARD ?
                            GoBildaPinpointDriver.EncoderDirection.FORWARD :
                            GoBildaPinpointDriver.EncoderDirection.REVERSED;
                    assert pl.pinpoint != null;
                    pl.pinpoint.setEncoderDirections(parDirection, perpDirection);
                }

                @NonNull
                @Override
                public DcMotorSimple.Direction getPerpDirection() {
                    return perpDirection == GoBildaPinpointDriver.EncoderDirection.FORWARD ?
                            DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
                }

                @Override
                public void setPerpDirection(@NonNull DcMotorSimple.Direction direction) {
                    perpDirection = direction == DcMotorSimple.Direction.FORWARD ?
                            GoBildaPinpointDriver.EncoderDirection.FORWARD :
                            GoBildaPinpointDriver.EncoderDirection.REVERSED;
                    assert pl.pinpoint != null;
                    pl.pinpoint.setEncoderDirections(parDirection, perpDirection);
                }
            };
            encoderGroups.add(new PinpointEncoderGroup(pv));
            parEncs.add(new EncoderRef(0, 0));
            perpEncs.add(new EncoderRef(0, 1));
            imu.accept(new PinpointIMU(pv));
        } else {
            throw new RuntimeException("Unknown localizer: " + localizer.getClass().getName());
        }

        Constants c = drive.getConstants();
        // Update only once for init to prevent re-init from rewriting, since these are static
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

        Threads.startLoop("update common tuning parameters", Milliseconds.of(500), () -> {
            DriveModel dm = drive.getConstants().getDriveModel();
            dm.inPerTick = tuningDriveModel_inPerTick;
            dm.lateralInPerTick = tuningDriveModel_lateralInPerTick;
            dm.trackWidthTicks = tuningDriveModel_trackWidthTicks;
            MotionProfile mp = drive.getConstants().getMotionProfile();
            mp.kS = tuningMotionProfile_kS;
            mp.kV = tuningMotionProfile_kV;
            mp.kA = tuningMotionProfile_kA;
            mp.maxWheelVel = tuningMotionProfile_maxWheelVel;
            mp.minProfileAccel = tuningMotionProfile_minProfileAccel;
            mp.maxProfileAccel = tuningMotionProfile_maxProfileAccel;
            mp.maxAngVel = tuningMotionProfile_maxAngVel;
            mp.maxAngAccel = tuningMotionProfile_maxAngAccel;
        });

        DriveViewFactory dvf;
        if (drive instanceof MecanumDrive md) {
            dvf = (h) -> {
                md.setPose(Geometry.zeroPose());

                // We don't want to expose the motors on the subsystem directly, we'll just use reflection here.
                Class<?> mdClass = md.getClass();
                Field leftFrontField, leftBackField, rightFrontField, rightBackField, imuField;
                try {
                    leftFrontField = mdClass.getDeclaredField("leftFront");
                    leftBackField = mdClass.getDeclaredField("leftBack");
                    rightFrontField = mdClass.getDeclaredField("rightFront");
                    rightBackField = mdClass.getDeclaredField("rightBack");
                    imuField = mdClass.getDeclaredField("imu");
                } catch (NoSuchFieldException e) {
                    throw new RuntimeException("MecanumDrive is missing fields for motors or IMU! This shouldn't happen!", e);
                }
                leftFrontField.setAccessible(true);
                leftBackField.setAccessible(true);
                rightFrontField.setAccessible(true);
                rightBackField.setAccessible(true);
                imuField.setAccessible(true);
                DcMotorEx leftFront, leftBack, rightFront, rightBack;
                try {
                    leftFront = (DcMotorEx) leftFrontField.get(md);
                    leftBack = (DcMotorEx) leftBackField.get(md);
                    rightFront = (DcMotorEx) rightFrontField.get(md);
                    rightBack = (DcMotorEx) rightBackField.get(md);
                    if (!imu.getInitialised())
                        imu.accept((IMU) imuField.get(md));
                } catch (IllegalAccessException e) {
                    throw new RuntimeException("Unable to access objects on MecanumDrive! This shouldn't happen!", e);
                }

                // Init once to allow editing
                if (tuningMecanumGains_axialGain == null) {
                    tuningMecanumGains_axialGain = md.gains.axialGain;
                    tuningMecanumGains_lateralGain = md.gains.lateralGain;
                    tuningMecanumGains_headingGain = md.gains.headingGain;
                    tuningMecanumGains_axialVelGain = md.gains.axialVelGain;
                    tuningMecanumGains_lateralVelGain = md.gains.lateralVelGain;
                    tuningMecanumGains_headingVelGain = md.gains.headingVelGain;
                }
                Threads.startLoop("update mecanum tuning parameters", Milliseconds.of(500), () -> {
                    assert tuningMecanumGains_axialGain != null && tuningMecanumGains_lateralGain != null &&
                            tuningMecanumGains_headingGain != null && tuningMecanumGains_axialVelGain != null &&
                            tuningMecanumGains_lateralVelGain != null && tuningMecanumGains_headingVelGain != null;
                    md.gains.axialGain = tuningMecanumGains_axialGain;
                    md.gains.lateralGain = tuningMecanumGains_lateralGain;
                    md.gains.headingGain = tuningMecanumGains_headingGain;
                    md.gains.axialVelGain = tuningMecanumGains_axialVelGain;
                    md.gains.lateralVelGain = tuningMecanumGains_lateralVelGain;
                    md.gains.headingVelGain = tuningMecanumGains_headingVelGain;
                });

                return new DriveView(
                        DriveType.MECANUM,
                        tuningDriveModel_inPerTick,
                        tuningMotionProfile_maxWheelVel,
                        tuningMotionProfile_minProfileAccel,
                        tuningMotionProfile_maxProfileAccel,
                        encoderGroups,
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
                        new LazyImuShim(imu.get()),
                        h.voltageSensor.iterator().next(),
                        () -> new MotorFeedforward(tuningMotionProfile_kS,
                                tuningMotionProfile_kV / tuningDriveModel_inPerTick,
                                tuningMotionProfile_kA / tuningDriveModel_inPerTick),
                        0
                );
            };
        } else if (drive instanceof TankDrive td) {
            dvf = (h) -> {
                td.setPose(Geometry.zeroPose());

                Class<?> tdClass = td.getClass();
                Field leftMotorsField, rightMotorsField, imuField;
                try {
                    leftMotorsField = tdClass.getDeclaredField("leftMotors");
                    rightMotorsField = tdClass.getDeclaredField("rightMotors");
                    imuField = tdClass.getDeclaredField("imu");
                } catch (NoSuchFieldException e) {
                    throw new RuntimeException("TankDrive is missing fields for motors or IMU! This shouldn't happen!", e);
                }
                leftMotorsField.setAccessible(true);
                rightMotorsField.setAccessible(true);
                imuField.setAccessible(true);
                List<DcMotorEx> leftMotors, rightMotors;
                try {
                    leftMotors = (List<DcMotorEx>) leftMotorsField.get(td);
                    rightMotors = (List<DcMotorEx>) rightMotorsField.get(td);
                    if (!imu.getInitialised())
                        imu.accept((IMU) imuField.get(td));
                } catch (IllegalAccessException e) {
                    throw new RuntimeException("Unable to access the objects on TankDrive! This shouldn't happen!", e);
                }

                if (tuningTankGains_ramseteZeta == null) {
                    tuningTankGains_ramseteZeta = td.gains.ramseteZeta;
                    tuningTankGains_ramseteBBar = td.gains.ramseteBBar;
                    tuningTankGains_turnGain = td.gains.turnGain;
                    tuningTankGains_turnVelGain = td.gains.turnVelGain;
                }
                Threads.startLoop("update tank tuning parameters", Milliseconds.of(500), () -> {
                    assert tuningTankGains_ramseteZeta != null && tuningTankGains_ramseteBBar != null &&
                            tuningTankGains_turnGain != null && tuningTankGains_turnVelGain != null;
                    td.gains.ramseteZeta = tuningTankGains_ramseteZeta;
                    td.gains.ramseteBBar = tuningTankGains_ramseteBBar;
                    td.gains.turnGain = tuningTankGains_turnGain;
                    td.gains.turnVelGain = tuningTankGains_turnVelGain;
                });

                assert leftMotors != null && rightMotors != null;
                return new DriveView(
                        DriveType.TANK,
                        tuningDriveModel_inPerTick,
                        tuningMotionProfile_maxWheelVel,
                        tuningMotionProfile_minProfileAccel,
                        tuningMotionProfile_maxProfileAccel,
                        encoderGroups,
                        leftMotors,
                        rightMotors,
                        leftEncs,
                        rightEncs,
                        parEncs,
                        perpEncs,
                        new LazyImuShim(imu.get()),
                        h.voltageSensor.iterator().next(),
                        () -> new MotorFeedforward(tuningMotionProfile_kS,
                                tuningMotionProfile_kV / tuningDriveModel_inPerTick,
                                tuningMotionProfile_kA / tuningDriveModel_inPerTick),
                        0
                );
            };
        } else {
            throw new RuntimeException("Unknown RoadRunnerDrive type!");
        }

        ForwardRampLogger.POWER_PER_SEC = 0.2;
        LateralRampLogger.POWER_PER_SEC = 0.2;
        ManualFeedforwardTuner.DISTANCE = 72;

        TelemetryMenu.MenuElement root = new TelemetryMenu.MenuElement("RoadRunner Tuning Selection", true);
        LinearOpMode[] opModes = {
                new AngularRampLogger(dvf),
                new ForwardPushTest(dvf),
                new ForwardRampLogger(dvf),
                new LateralPushTest(dvf),
                new LateralRampLogger(dvf),
                new ManualFeedforwardTuner(dvf),
                new MecanumMotorDirectionDebugger(dvf),
                new DeadWheelDirectionDebugger(dvf),
                new OTOSAngularScalarTuner(dvf),
                new OTOSLinearScalarTuner(dvf),
                new OTOSHeadingOffsetTuner(dvf),
                new OTOSPositionOffsetTuner(dvf),
                new ManualFeedbackTuner(drive),
                new SplineTest(drive),
                new LocalizationTest(drive)
        };

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> cls : Arrays.asList(
                    AngularRampLogger.class,
                    ForwardRampLogger.class,
                    LateralRampLogger.class,
                    ManualFeedforwardTuner.class,
                    MecanumMotorDirectionDebugger.class,
                    ManualFeedbackTuner.class
            )) {
                configRoot.putVariable("[RR] " + cls.getSimpleName(), ReflectionConfig.createVariableFromClass(cls));
            }
        });

        TelemetryMenu.StaticClickableOption[] opModeSelections = new TelemetryMenu.StaticClickableOption[opModes.length];
        LateInitCell<LinearOpMode> selection = new LateInitCell<>();
        for (int i = 0; i < opModes.length; i++) {
            LinearOpMode opMode = opModes[i];
            opModeSelections[i] = new TelemetryMenu.StaticClickableOption(opMode.getClass().getSimpleName(),
                    () -> selection.accept(opMode));
        }
        root.addChildren(opModeSelections);
        TelemetryMenu menu = new TelemetryMenu(fusedTelemetry, root);
        try {
            selectedIdxField.setInt(menu, lastSelectedIndex);
        } catch (IllegalAccessException e) {
            throw new RuntimeException("Access exception! This shouldn't happen!", e);
        }

        while (!selection.getInitialised() && opModeInInit()) {
            menu.loop(gamepad1);
            fusedTelemetry.update();
            try {
                lastSelectedIndex = selectedIdxField.getInt(menu);
            } catch (IllegalAccessException e) {
                throw new RuntimeException("Access exception! This shouldn't happen!", e);
            }
        }

        if (isStopRequested())
            return;

        fusedTelemetry.clearAll();
        if (!selection.getInitialised()) {
            // Force select current option if none was selected
            selection.accept(opModes[lastSelectedIndex]);
        }

        try {
            LinearOpMode opMode = selection.get();
            opMode.gamepad1 = gamepad1;
            opMode.gamepad2 = gamepad2;
            opMode.telemetry = telemetry;
            opMode.hardwareMap = hardwareMap;

            // Generate some feedback when pressing an OpMode so the user doesn't think the procedure is broken
            fusedTelemetry.setCaptionValueSeparator(": ");
            fusedTelemetry.addData(Text.html().bold(opMode.getClass().getSimpleName()).toString(), "Ready.");
            double r = getRuntime();
            while (getRuntime() < r + 1)
                fusedTelemetry.update();

            FtcDashboard.getInstance().withConfigRoot(configRoot ->
                    configRoot.putVariable("[RR] Tuning Parameters", ReflectionConfig.createVariableFromClass(getClass())));

            Threads.start("wait for start", () -> {
                // We wait for the OpMode to become a ready state then force the internal OpMode to begin
                while (opModeInInit()) {
                    //noinspection BusyWait
                    Thread.sleep(500);
                }
                if (isStopRequested())
                    return null;
                try {
                    //noinspection ExtractMethodRecommender
                    Class<?> opModeInternal = Objects.requireNonNull(OpMode.class.getSuperclass());
                    Field isStarted = opModeInternal.getDeclaredField("isStarted");
                    // Force start as this OpMode is not technically the one running, but we have the instance
                    // We could alternatively use the OpModeManager to force re-init a new OpMode, but we may as well
                    // keep it all localised into a single class.
                    isStarted.setAccessible(true);
                    isStarted.set(opMode, true);
                    Field runningNotifier = LinearOpMode.class.getDeclaredField("runningNotifier");
                    runningNotifier.setAccessible(true);
                    Object notifier = Objects.requireNonNull(runningNotifier.get(opMode));
                    synchronized (notifier) {
                        notifier.notifyAll();
                    }
                } catch (NoSuchFieldException | IllegalAccessException | NullPointerException e) {
                    throw new RuntimeException("Internal error while starting OpMode. This shouldn't happen!", e);
                }
                return null;
            });

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
