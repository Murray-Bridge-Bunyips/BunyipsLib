package au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Degrees;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.DegreesPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Velocity;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Threads;

/**
 * Drop-in replacement for an Inertial Measurement Unit that provides WPIUnit data and the capabilities of automatic updates
 * similar to (but not implementing) a subsystem multi-thread.
 * <p>
 * This class supports different readings of IMU measurement, such as an unrestricted domain
 * on heading reads, while also being able to provide IMU units in terms of WPIUnits.
 * <p>
 * This class is also down-castable to the Universal IMU Interface. Note that all data in this class is retrieved
 * via the {@link #run()} method, which opens the possibility for threading the IMU via {@link #startThread()}, similar
 * to a {@link BunyipsSubsystem}.
 * <p>
 * Since this class is down-castable, it also can be used as a {@link HardwareDevice} and serves
 * as a drop-in replacement for the {@link IMU} interface.
 * <p>
 * Field-exposed angles from this class are intrinsic. Read more in the {@link YawPitchRollAngles} and {@link Orientation} classes.
 *
 * @author Lucas Bubner, 2024
 * @since 4.0.0
 */
public class IMUEx implements IMU, Runnable {
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

    private String threadName = null;
    private YawDomain domain = YawDomain.SIGNED;
    private Measure<Angle> yawOffset = Degrees.zero();
    private double yawDeltaMultiplier = 1;
    private double angleSumDeg;
    private double lastYawDeg;

    /**
     * Wrap an IMU to use in IMUEx.
     *
     * @param imu the imu to wrap
     */
    public IMUEx(IMU imu) {
        this.imu = imu;
    }

    @Override
    public boolean initialize(Parameters parameters) {
        return imu.initialize(parameters);
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
     * Data will be automatically updated when this method is called, unless it is being updated asynchronously.
     *
     * @inheritDoc
     */
    @Override
    @SuppressWarnings("DataFlowIssue")
    public YawPitchRollAngles getRobotYawPitchRollAngles() {
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
     * Data will be automatically updated when this method is called, unless it is being updated asynchronously.
     *
     * @inheritDoc
     */
    @Override
    public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
        return getRobotOrientationAsQuaternion().toOrientation(reference, order, angleUnit);
    }

    /**
     * Note: This Orientation will not respect the currently set {@link YawDomain} to ensure consistency across usages
     * of the Universal IMU Interface.
     * <p>
     * Data will be automatically updated when this method is called, unless it is being updated asynchronously.
     *
     * @inheritDoc
     */
    @Override
    public Quaternion getRobotOrientationAsQuaternion() {
        run();
        return quaternion;
    }

    @Override
    @SuppressWarnings("DataFlowIssue")
    public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
        run();
        return new AngularVelocity(AngleUnit.DEGREES, (float) pitchVel.magnitude(), (float) rollVel.magnitude(), (float) yawVel.magnitude(), lastAcquisitionTimeNanos);
    }

    /**
     * Gets the current yaw domain, which is the domain this IMU is reporting {@link #yaw} at.
     *
     * @return the yaw domain that can be used to view the current domain and convert between
     */
    public YawDomain getYawDomain() {
        return domain;
    }

    /**
     * Sets the domain range of what the {@link #yaw} field will return. By default, this is set to the expected
     * behaviour of [-180, 180) degrees, but can be adjusted here to one of the {@link YawDomain} options.
     * Note that this yaw domain will only apply to the {@link #yaw} field, not in the IMU interface overrides.
     *
     * @param newDomain the new domain of the {@link #yaw} property
     */
    public void setYawDomain(YawDomain newDomain) {
        if (newDomain == null) return;
        domain = newDomain;
    }

    /**
     * @return the current yaw offset as respected by this class and its methods
     */
    public Measure<Angle> getYawOffset() {
        return yawOffset;
    }

    /**
     * Set an offset of the yaw value that will apply for <i>all</i> future yaw reads in this class.
     *
     * @param yawOffset the yaw offset that will be added, applies to all IMU-related reads
     */
    public void setYawOffset(Measure<Angle> yawOffset) {
        if (yawOffset == null) return;
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
     * Update the class fields and method return values with newest information from the IMU.
     * This method will no-op if updates have been delegated via {@link #startThread()}.
     */
    @Override
    public void run() {
        if (threadName != null) return;
        internalUpdate();
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
     * The thread will run at full speed as-is.
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
     *
     * @param loopSleepDuration the duration to sleep this thread after every loop to save resources
     */
    public void startThread(Measure<Time> loopSleepDuration) {
        if (threadName != null) return;
        // Run at least once to ensure last acquired time is updated
        internalUpdate();
        threadName = "IMUEx-Threaded-" + hashCode();
        Threads.startLoop(this::internalUpdate, threadName, loopSleepDuration);
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
        Measure<Angle> signedClamp = Mathf.angleModulus(totalYaw);

        // Only apply YawDomain to the yaw field
        if (domain == YawDomain.SIGNED) {
            yaw = signedClamp;
        } else if (domain == YawDomain.UNSIGNED) {
            yaw = Mathf.normaliseAngle(totalYaw);
        } else {
            // Unrestricted domain
            yaw = totalYaw;
        }

        // Always apply signed domain to raw orientation (required for yaw multiplier/offset)
        rawOrientation.thirdAngle = (float) signedClamp.in(Degrees);

        lastAcquisitionTimeNanos = rawOrientation.acquisitionTime;
        quaternion = Quaternion.fromMatrix(rawOrientation.getRotationMatrix(), rawOrientation.acquisitionTime);

        // These fields are also bound by the [-180, 180) degree domain but can be converted with Mathf utilities.
        // IMUEx provides a built in utility for the yaw, as it is a common use case and usually you wouldn't need
        // to use these fields in a different domain. Note that the yaw field is the only element affected
        // by the yaw domain restriction.
        pitch = Degrees.of(rawOrientation.firstAngle);
        roll = Degrees.of(rawOrientation.secondAngle);

        yawVel = DegreesPerSecond.of(angleVels.zRotationRate);
        pitchVel = DegreesPerSecond.of(angleVels.xRotationRate);
        rollVel = DegreesPerSecond.of(angleVels.yRotationRate);
    }

    @Override
    public Manufacturer getManufacturer() {
        return imu.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return imu.getDeviceName();
    }

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
