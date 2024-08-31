package org.murraybridgebunyips.bunyipslib.subsystems;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Degrees;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.DegreesPerSecond;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.external.Mathf;
import org.murraybridgebunyips.bunyipslib.external.units.Angle;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Velocity;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * IMU utility class that will wrap an Inertial Measurement Unit to provide data and automatic updates
 * of the IMU angles as part of a subsystem. This subsystem has no meaningful {@link Task} to schedule.
 * <p>
 * This class supports different readings of IMU measurement, such as an unrestricted domain
 * on heading reads, while also being able to provide IMU units in terms of WPIUnits.
 * <p>
 * This class is also down-castable to the Universal IMU Interface. Note that all data in this class is read
 * in the {@link #update()} method, which opens the possibility for threading the IMU via {@link #startThread}.
 * <p>
 * Field-exposed angles from this class are intrinsic. Read more in the {@link YawPitchRollAngles} and {@link Orientation} classes.
 *
 * @author Lucas Bubner, 2024
 * @since 4.0.0
 */
public class IMUOp extends BunyipsSubsystem implements IMU {
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

    @NonNull
    private YawDomain domain = YawDomain.SIGNED;
    private Measure<Angle> yawOffset = Degrees.zero();
    private double angleSumDeg;
    private double lastYawDeg;

    /**
     * Wrap an IMU to use in IMUOp.
     *
     * @param imu the imu to wrap
     */
    public IMUOp(IMU imu) {
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
     *
     * @inheritDoc
     */
    @Override
    @SuppressWarnings("DataFlowIssue")
    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        if (lastAcquisitionTimeNanos == null) {
            Dbg.warn("IMUOp subsystem wrapping an IMU is not being updated. An automatic call to update() has been propagated. Further results from this class will be stale if not updated.");
            update();
        }
        return new YawPitchRollAngles(
                AngleUnit.DEGREES,
                // Note: Exposed yaw is controlled by the yaw domain, and may not conform to the requirement as listed by the YawPitchRollAngles class.
                // Therefore, this is why the yaw property of this specific method will not respect the YawDomain, and instead access
                // the raw quaternion to generate the instance of YawPitchRollAngles.
                quaternion.toOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle,
                pitch.in(Degrees),
                roll.in(Degrees),
                lastAcquisitionTimeNanos
        );
    }

    /**
     * Note: This Orientation will not respect the currently set {@link YawDomain} to ensure consistency across usages
     * of the Universal IMU Interface.
     *
     * @inheritDoc
     */
    @Override
    public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
        if (lastAcquisitionTimeNanos == null) {
            Dbg.warn("IMUOp subsystem wrapping an IMU is not being updated. An automatic call to update() has been propagated. Further results from this class will be stale if not updated.");
            update();
        }
        return quaternion.toOrientation(reference, order, angleUnit);
    }

    /**
     * Note: This Orientation will not respect the currently set {@link YawDomain} to ensure consistency across usages
     * of the Universal IMU Interface.
     *
     * @inheritDoc
     */
    @Override
    public Quaternion getRobotOrientationAsQuaternion() {
        if (lastAcquisitionTimeNanos == null) {
            Dbg.warn("IMUOp subsystem wrapping an IMU is not being updated. An automatic call to update() has been propagated. Further results from this class will be stale if not updated.");
            update();
        }
        return quaternion;
    }

    @Override
    @SuppressWarnings("DataFlowIssue")
    public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
        if (lastAcquisitionTimeNanos == null) {
            Dbg.warn("IMUOp subsystem wrapping an IMU is not being updated. An automatic call to update() has been propagated. Further results from this class will be stale if not updated.");
            update();
        }
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
     * @param yawOffset the yaw offset that will be added to the actual yaw
     */
    public void setYawOffset(Measure<Angle> yawOffset) {
        if (yawOffset == null) return;
        this.yawOffset = yawOffset;
    }

    @Override
    protected void periodic() {
        // Get raw orientations from the IMU to give us full information
        // This will be where the data is retrieved from the IMU, and will be the blocking part of the loop
        Orientation rawOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        AngularVelocity angleVels = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        // Add the yaw offset to apply to all sources
        rawOrientation.thirdAngle += (float) yawOffset.in(Degrees);
        rawOrientation.thirdAngle = AngleUnit.normalizeDegrees(rawOrientation.thirdAngle);

        lastAcquisitionTimeNanos = rawOrientation.acquisitionTime;
        quaternion = Quaternion.fromMatrix(rawOrientation.getRotationMatrix(), rawOrientation.acquisitionTime);

        double yawDeg = rawOrientation.thirdAngle;
        double yawDelta = yawDeg - lastYawDeg;
        if (Math.abs(yawDelta) >= 180) {
            // IMU returns angles in signed form, need to undo this as we'll clamp this value ourselves
            yawDelta = -Math.signum(yawDelta) * (360 - Math.abs(yawDelta));
        }
        lastYawDeg = yawDeg;

        angleSumDeg += yawDelta;
        Measure<Angle> totalYaw = Degrees.of(angleSumDeg);

        if (domain == YawDomain.SIGNED) {
            yaw = Mathf.angleModulus(totalYaw);
        } else if (domain == YawDomain.UNSIGNED) {
            yaw = Mathf.normaliseAngle(totalYaw);
        } else {
            // Unrestricted domain
            yaw = totalYaw;
        }

        // These fields are also bound by the [-180, 180) degree domain but can be converted with Mathf utilities.
        // IMUOp provides a built in utility for the yaw, as it is a common use case and usually you wouldn't need
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
        disable();
        imu.close();
    }

    /**
     * The various modes that the {@link #yaw} field can represent the current robot yaw as.
     * Note this domain does not apply to the IMU interface methods.
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
