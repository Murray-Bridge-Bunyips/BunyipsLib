package au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Degrees;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.DegreesPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliseconds;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.ImuInitMessage;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Dbg;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.EmergencyStop;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Periodic;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.RobotConfig;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Velocity;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Threads;

/**
 * Drop-in replacement for an Inertial Measurement Unit that provides WPIUnit data through fields and yaw domain,
 * while implementing lazy loading compatible with RoadRunner.
 * <p>
 * This class supports different readings of IMU measurement, such as an unrestricted domain
 * on heading reads, while also being able to provide IMU units in terms of WPIUnits.
 * <p>
 * This class is also down-castable to the Universal IMU Interface. Note that all data in this class is retrieved
 * via the {@link #run()} method, which is auto-called when the conventional {@link IMU} methods are called.
 * Threading capabilities do exist for IMUEx, however, they are dangerous operations.
 * <p>
 * Since this class is down-castable, it also can be used as a {@link HardwareDevice} and serves
 * as drop-in replacements for the {@link IMU} and {@link LazyImu} interfaces.
 * <p>
 * Field-exposed angles from this class are intrinsic. Read more in the {@link YawPitchRollAngles} and {@link Orientation} classes.
 * <p>
 * This class also fuses the old DynIMU to expose {@link LazyImu} interface by taking a reference to an
 * uninitialised {@link IMU} object and initialising it on the first call to a method.
 * <p>
 * As of v7.0.0, IMUEx is the standard IMU recommended for use in BunyipsLib, as it implements both {@link LazyImu} and {@link IMU}.
 * A read refresh rate can also be specified via {@link #setRefreshRate}.
 *
 * @author Lucas Bubner, 2025
 * @since 7.0.0
 */
public class IMUEx implements IMU, LazyImu, Runnable {
    private final IMU imu;
    /**
     * Last read yaw (heading) of the IMU with the yaw offset.
     * The domain of this value is controlled by the currently set by the {@link YawDomain} (see {@link #setYawDomain}).
     * Defaults to a zero magnitude unit.
     */
    @NonNull
    public volatile Measure<Angle> yaw = Degrees.zero();
    /**
     * Last read pitch of the IMU.
     * Defaults to a zero magnitude unit.
     */
    @NonNull
    public volatile Measure<Angle> pitch = Degrees.zero();
    /**
     * Last read roll of the IMU.
     * Defaults to a zero magnitude unit.
     */
    @NonNull
    public volatile Measure<Angle> roll = Degrees.zero();
    /**
     * Last read yaw velocity of the IMU.
     * Defaults to a zero magnitude unit.
     */
    @NonNull
    public volatile Measure<Velocity<Angle>> yawVel = DegreesPerSecond.zero();
    /**
     * Last read pitch velocity of the IMU.
     * Defaults to a zero magnitude unit.
     */
    @NonNull
    public volatile Measure<Velocity<Angle>> pitchVel = DegreesPerSecond.zero();
    /**
     * Last read roll velocity of the IMU.
     * Defaults to a zero magnitude unit.
     */
    @NonNull
    public volatile Measure<Velocity<Angle>> rollVel = DegreesPerSecond.zero();
    /**
     * Last known robot-centric quaternion reported from the IMU and yaw offset. Intended for advanced users.
     * Defaults to the identity quaternion if not updated.
     */
    @NonNull
    public volatile Quaternion quaternion = Quaternion.identityQuaternion();
    /**
     * Last acquisition of data from the IMU by IMUOp. Nullable if this subsystem hasn't updated. Unit is in
     * nanoseconds as returned by {@link System#nanoTime()}.
     */
    @Nullable
    public volatile Long lastAcquisitionTimeNanos = null;
    @Nullable
    private Parameters parameters;
    private boolean triedInit;
    private double timeoutMs = 700;
    private String threadName = null;
    private YawDomain domain = YawDomain.SIGNED;
    private Measure<Angle> yawOffset = Degrees.zero();
    private double yawDeltaMultiplier = 1;
    private double angleSumDeg;
    private double lastYawDeg;
    private final Periodic executor = new Periodic(Milliseconds.zero(), this::internalUpdate);

    /**
     * Wrap a new IMUEx that will initialise the given {@link IMU} object when a read is required.
     * <p>
     * The IMU will be lazy initialized with the given IMU parameters.
     *
     * @param uninitializedIMU the {@link IMU} instance that is <b>not yet initialised</b>.
     *                         Do not manually call {@link #initialize(Parameters)} on this IMU before passing it here.
     *                         Your IMU will be doubly initialised if you initialise it manually outside this class.
     * @param lazyParameters   IMU parameters to use for lazy initialization if not manually initialized
     */
    public IMUEx(@NonNull IMU uninitializedIMU, @NonNull Parameters lazyParameters) {
        parameters = lazyParameters;
        imu = uninitializedIMU;
    }

    /**
     * Wrap a new IMUEx that will initialise the given {@link IMU} object when a read is required.
     * <p>
     * Note: When using this constructor you need to call {@link #lazyInitialize(Parameters)} or the conventional
     * {@link #initialize(Parameters)} to allow initialization capabilities. Exceptions will be thrown if IMU accessors
     * are called on an uninitialised IMU.
     *
     * @param uninitializedIMU the {@link IMU} instance that is <b>not yet initialised</b>.
     *                         Do not manually call {@link #initialize(Parameters)} on this IMU before passing it here.
     *                         Your IMU will be doubly initialised if you initialise it manually outside this class.
     */
    public IMUEx(@NonNull IMU uninitializedIMU) {
        imu = uninitializedIMU;
    }

    /**
     * Creates a IMUEx with no IMU attached. This is useful if you know for a fact that you will never need to use the
     * IMU, but still need to pass an IMU object to a method.
     *
     * @return a IMUEx object that will always return invalid zero data, do be cautious that accessing this object in
     * a method that would require the presence of an IMU will cause warnings and potentially cause issues.
     */
    @NonNull
    public static IMUEx none() {
        return new IMUEx(new IMU() {
            private final boolean[] warned = new boolean[12];

            @Override
            public boolean initialize(Parameters parameters) {
                if (!warned[0]) Dbg.warn("IMUEx.none() was accessed for initialisation.");
                warned[0] = true;
                return false;
            }

            @Override
            public void resetYaw() {
                if (!warned[1]) Dbg.warn("IMUEx.none() was accessed for yaw reset.");
                warned[1] = true;
            }

            @Override
            public YawPitchRollAngles getRobotYawPitchRollAngles() {
                if (!warned[2]) Dbg.warn("IMUEx.none() was accessed for yaw pitch roll angles.");
                warned[2] = true;
                return new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);
            }

            @Override
            public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
                if (!warned[3]) Dbg.warn("IMUEx.none() was accessed for orientation.");
                warned[3] = true;
                return new Orientation();
            }

            @Override
            public Quaternion getRobotOrientationAsQuaternion() {
                if (!warned[4]) Dbg.warn("IMUEx.none() was accessed for orientation as quaternion.");
                warned[4] = true;
                return new Quaternion();
            }

            @Override
            public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
                if (!warned[5]) Dbg.warn("IMUEx.none() was accessed for angular velocity.");
                warned[5] = true;
                return new AngularVelocity();
            }

            @Override
            public Manufacturer getManufacturer() {
                if (!warned[6]) Dbg.warn("IMUEx.none() was accessed for manufacturer.");
                warned[6] = true;
                return Manufacturer.Unknown;
            }

            @Override
            public String getDeviceName() {
                if (!warned[7]) Dbg.warn("IMUEx.none() was accessed for device name.");
                warned[7] = true;
                return "IMUEx.none()";
            }

            @Override
            public String getConnectionInfo() {
                if (!warned[8]) Dbg.warn("IMUEx.none() was accessed for connection info.");
                warned[8] = true;
                return "IMUEx.none()";
            }

            @Override
            public int getVersion() {
                if (!warned[9]) Dbg.warn("IMUEx.none() was accessed for version.");
                warned[9] = true;
                return 0;
            }

            @Override
            public void resetDeviceConfigurationForOpMode() {
                if (!warned[10]) Dbg.warn("IMUEx.none() was accessed for device configuration reset.");
                warned[10] = true;
            }

            @Override
            public void close() {
                if (!warned[11]) Dbg.warn("IMUEx.none() was accessed for close.");
                warned[11] = true;
            }
        }, new Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
    }

    /**
     * Set a custom timeout for the initialisation of the IMU that will fail the initialisation if it takes too long.
     *
     * @param timeout the timeout for the initialisation, default 500ms
     */
    public void setInitTimeout(@NonNull Measure<Time> timeout) {
        timeoutMs = timeout.in(Milliseconds);
    }

    private boolean tryInit(String reason) {
        if (parameters == null)
            throw new EmergencyStop("IMUEx does not have any information available to initialise the required IMU! Since this data is unavailable when it is needed, use `lazyInitialize` or `initialize` on this object to resolve this error.");

        if (triedInit) return true;
        triedInit = true;

        Dbg.logd(getClass(), "IMU initialisation in progress (reason=%) ...", reason);
        boolean res = imu.initialize(parameters);
        if (!res)
            RobotLog.addGlobalWarningMessage("IMUEx failed to initialise the IMU!");

        long start = System.nanoTime();
        ElapsedTime timer = new ElapsedTime();
        do {
            Quaternion q = imu.getRobotOrientationAsQuaternion();
            if (q.acquisitionTime != 0) {
                FlightRecorder.write("IMU_INIT", new ImuInitMessage(imu.getDeviceName(), start, System.nanoTime(), false));
                Dbg.logv(getClass(), "IMU initialised.");
                return res;
            }
        } while (timer.milliseconds() < timeoutMs);

        FlightRecorder.write("IMU_INIT", new ImuInitMessage(imu.getDeviceName(), start, System.nanoTime(), true));
        RobotLog.addGlobalWarningMessage("IMU through a IMUEx instance continues to return invalid data after " + timeoutMs + " ms!");
        return res;
    }

    /**
     * Ready the IMU for lazy initialisation using these parameters. This method is <b>REQUIRED</b> to be called
     * if you have not passed in a {@link Parameters} object to the constructor of IMUEx (such as acquiring this
     * instance through {@link RobotConfig}) and IMU accessor methods are desired to be called.
     * <p>
     * This method differs from {@link #initialize(Parameters)}, which will initialise your IMU now.
     * <p>
     * A fatal exception will be thrown if {@code lazyInitialize}/{@code initialize} is not called and IMU accessors are called.
     *
     * @param parametersToUseLater the IMU parameters as defined by the Universal IMU Interface
     */
    public void lazyInitialize(@NonNull Parameters parametersToUseLater) {
        parameters = parametersToUseLater;
    }

    @Override
    public boolean initialize(@NonNull Parameters parametersToUseNow) {
        this.parameters = parametersToUseNow;
        // We reinitialise always if we call the manual initialize method
        if (triedInit)
            Dbg.logd(getClass(), "IMU is reinitialising...");
        triedInit = false;
        return tryInit("MANUAL_INIT");
    }

    @Override
    public void resetYaw() {
        angleSumDeg = 0;
        lastYawDeg = 0;
        imu.resetYaw();
    }

    /**
     * Note: This method will not respect the currently set {@link YawDomain}, as it may not respect the angle requirement
     * as listed by the {@link YawPitchRollAngles} that is provided by this method. This ensures consistent use of
     * the {@link YawPitchRollAngles} class while still delegating the collection of IMU data.
     * <p>
     * Data will be automatically updated when this method is called at the refresh rate, unless it is being updated asynchronously.
     *
     * @inheritDoc
     */
    @NonNull
    @Override
    @SuppressWarnings("DataFlowIssue")
    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        tryInit("GET_ROBOT_YAW_PITCH_ROLL_ANGLES");
        run();
        return new YawPitchRollAngles(
                AngleUnit.DEGREES,
                // Note: Exposed yaw is controlled by the yaw domain, and may not conform to the requirement as listed by the YawPitchRollAngles class.
                // Therefore, this is why the yaw property of this specific method will not respect the YawDomain, and instead access
                // the raw quaternion to generate the instance of YawPitchRollAngles. (Yaw offset and multiplier still apply)
                quaternion.toOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle,
                pitch.in(Degrees),
                roll.in(Degrees),
                lastAcquisitionTimeNanos
        );
    }

    /**
     * Note: This Orientation will not respect the currently set {@link YawDomain} to ensure consistency across usages
     * of the Universal IMU Interface.
     * <p>
     * Data will be automatically updated when this method is called at the refresh rate, unless it is being updated asynchronously.
     *
     * @inheritDoc
     */
    @NonNull
    @Override
    public Orientation getRobotOrientation(@NonNull AxesReference reference, @NonNull AxesOrder order, @NonNull AngleUnit angleUnit) {
        tryInit("GET_ROBOT_ORIENTATION");
        return getRobotOrientationAsQuaternion().toOrientation(reference, order, angleUnit);
    }

    /**
     * Note: This Orientation will not respect the currently set {@link YawDomain} to ensure consistency across usages
     * of the Universal IMU Interface.
     * <p>
     * Data will be automatically updated when this method is called at the refresh rate, unless it is being updated asynchronously.
     *
     * @inheritDoc
     */
    @NonNull
    @Override
    public Quaternion getRobotOrientationAsQuaternion() {
        tryInit("GET_ROBOT_ORIENTATION_AS_QUATERNION");
        run();
        return quaternion;
    }

    @NonNull
    @Override
    @SuppressWarnings("DataFlowIssue")
    public AngularVelocity getRobotAngularVelocity(@NonNull AngleUnit angleUnit) {
        tryInit("GET_ROBOT_ANGULAR_VELOCITY");
        run();
        return new AngularVelocity(AngleUnit.DEGREES, (float) pitchVel.magnitude(), (float) rollVel.magnitude(), (float) yawVel.magnitude(), lastAcquisitionTimeNanos);
    }

    @NonNull
    @Override
    public Manufacturer getManufacturer() {
        return imu.getManufacturer();
    }

    @NonNull
    @Override
    public String getDeviceName() {
        return imu.getDeviceName();
    }

    @NonNull
    @Override
    public String getConnectionInfo() {
        return imu.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return imu.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        imu.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        imu.close();
    }

    @NonNull
    @Override
    public IMU get() {
        tryInit("LAZY_INIT");
        return this;
    }

    /**
     * Gets the current yaw domain, which is the domain this IMU is reporting {@link #yaw} at.
     *
     * @return the yaw domain that can be used to view the current domain and convert between
     */
    @NonNull
    public YawDomain getYawDomain() {
        return domain;
    }

    /**
     * Sets the domain range of what the {@link #yaw} field will return. By default, this is set to the expected
     * behaviour of {@code [-180, 180)} degrees, but can be adjusted here to one of the {@link YawDomain} options.
     * Note that this yaw domain will only apply to the {@link #yaw} field, not in the IMU interface overrides.
     *
     * @param newDomain the new domain of the {@link #yaw} property
     */
    public void setYawDomain(@NonNull YawDomain newDomain) {
        domain = newDomain;
    }

    /**
     * @return the current yaw offset as respected by this class and its methods
     */
    @NonNull
    public Measure<Angle> getYawOffset() {
        return yawOffset;
    }

    /**
     * Set an offset of the yaw value that will apply for <i>all</i> future yaw reads in this class.
     *
     * @param yawOffset the yaw offset that will be added, applies to all IMU-related reads
     */
    public void setYawOffset(@NonNull Measure<Angle> yawOffset) {
        this.yawOffset = yawOffset;
    }

    /**
     * Set a multiplicative scale that will be applied to every delta update of the IMU heading. This is similar
     * to calibrating a dead wheel, where you will supply a ratio between measured/actual values to improve IMU performance.
     *
     * @param mul multiplicative scale for yaw readings, applies to all IMU-related reads (default 1)
     */
    public void setYawMultiplier(double mul) {
        yawDeltaMultiplier = mul;
    }

    /**
     * Sets the refresh rate at which the IMU should be read.
     * <p>
     * By setting a higher interval, data will be more stale, but hardware reads will be cached between intervals.
     * You may want to increase this value to optimise loop times or set a "fixed" interval for IMU reads.
     * <p>
     * By default, the refresh rate is 0 meaning every invocation will read the IMU.
     *
     * @param interval the interval at which the IMU will read at for all future IMU accessors
     * @since 7.0.0
     */
    public void setRefreshRate(@NonNull Measure<Time> interval) {
        executor.setInterval(interval);
    }

    /**
     * Update the class fields and method return values with the newest information from the IMU.
     * <p>
     * This method will no-op if updates have been delegated via {@link #startThread()}.
     */
    @Override
    public void run() {
        if (threadName != null) return;
        executor.run();
    }

    /**
     * Call to delegate the manual data updating of the IMU to a thread managed by {@link Threads}.
     * Manual calls to {@link #run()} will be ignored as they will be dispatched by the thread.
     * This mirrors behaviour found in a {@link BunyipsSubsystem}.
     * <p>
     * <b>WARNING:</b> Multi-threading the IMU may not come with any performance gains and can degrade
     * the performance and reliability of your OpModes due to how I2C calls are propagated.
     * This method is exposed only for highly advanced operations and opens a wide range of technical
     * complexity when dealing with hardware threading.
     * You must ensure you know what you're doing before you multithread.
     * <p>
     * The thread will run at full speed as-is, ignoring the refresh rate interval as set by {@link #setRefreshRate(Measure)}.
     */
    public void startThread() {
        startThread(Seconds.zero());
    }

    /**
     * Call to delegate the manual data updating of the IMU to a thread managed by {@link Threads}.
     * Manual calls to {@link #run()} will be ignored as they will be dispatched by the thread.
     * This mirrors behaviour found in a {@link BunyipsSubsystem}.
     * <p>
     * <b>WARNING:</b> Multi-threading the IMU may not come with any performance gains and can degrade
     * the performance and reliability of your OpModes due to how I2C calls are propagated.
     * This method is exposed only for highly advanced operations and opens a wide range of technical
     * complexity when dealing with hardware threading.
     * You must ensure you know what you're doing before you multithread.
     * <p>
     * Threaded operation ignores the refresh rate as set by {@link #setRefreshRate(Measure)}
     *
     * @param loopSleepDuration the duration to sleep this thread after every loop to save resources
     */
    public void startThread(@NonNull Measure<Time> loopSleepDuration) {
        if (threadName != null) return;
        // Run at least once to ensure last acquired time is updated
        internalUpdate();
        threadName = "IMUEx-Threaded-" + hashCode();
        Threads.startLoop(threadName, loopSleepDuration, this::internalUpdate);
    }

    /**
     * Call to stop the thread started by {@link #startThread()}.
     */
    public void stopThread() {
        if (threadName == null) return;
        Threads.stop(threadName);
        threadName = null;
    }

    private void internalUpdate() {
        tryInit("AUTO_INIT");

        // Get raw orientations from the IMU to give us full information
        // This will be where the data is retrieved from the IMU, and will be the blocking part of the loop
        Orientation rawOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        AngularVelocity angleVels = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        double yawDeg = rawOrientation.thirdAngle;
        double yawDelta = yawDeg - lastYawDeg;
        if (Math.abs(yawDelta) >= 180) {
            // IMU returns angles in signed form, need to undo this as we'll clamp this value ourselves
            yawDelta = -Math.signum(yawDelta) * (360 - Math.abs(yawDelta));
        }
        lastYawDeg = yawDeg;

        angleSumDeg += yawDelta * yawDeltaMultiplier;
        Measure<Angle> totalYaw = Degrees.of(angleSumDeg).plus(yawOffset);
        Measure<Angle> signedClamp = Mathf.wrapDelta(totalYaw);

        // Only apply YawDomain to the yaw field
        if (domain == YawDomain.SIGNED) {
            yaw = signedClamp;
        } else if (domain == YawDomain.UNSIGNED) {
            yaw = Mathf.wrap(totalYaw);
        } else {
            // Unrestricted domain
            yaw = totalYaw;
        }

        // Always apply signed domain to raw orientation (required for yaw multiplier/offset)
        rawOrientation.thirdAngle = (float) signedClamp.in(Degrees);

        lastAcquisitionTimeNanos = rawOrientation.acquisitionTime;
        quaternion = Quaternion.fromMatrix(rawOrientation.getRotationMatrix(), rawOrientation.acquisitionTime);

        // These fields are also bound by the [-180, 180) degree domain but can be converted with Mathf utilities.
        // IMUEx provides a built-in utility for the yaw, as it is a common use case, and usually you wouldn't need
        // to use these fields in a different domain. Note that the yaw field is the only element affected
        // by the yaw domain restriction.
        pitch = Degrees.of(rawOrientation.firstAngle);
        roll = Degrees.of(rawOrientation.secondAngle);

        yawVel = DegreesPerSecond.of(angleVels.zRotationRate);
        pitchVel = DegreesPerSecond.of(angleVels.xRotationRate);
        rollVel = DegreesPerSecond.of(angleVels.yRotationRate);
    }

    /**
     * The various modes that the {@link #yaw} field can represent the current robot yaw as.
     * Note this domain does not apply to the {@link IMU} interface methods.
     * If you wish to convert between these domains, see the utilities available in {@link Mathf}.
     */
    public enum YawDomain {
        /**
         * Default behaviour. Angle is wrapped between [-180, 180) degrees, or [-π, π) radians.
         */
        SIGNED,
        /**
         * Angle is wrapped between [0, 360) degrees, or [0, 2π) radians.
         */
        UNSIGNED,
        /**
         * Angle is unrestricted between (-∞, ∞) units.
         */
        UNRESTRICTED
    }
}
