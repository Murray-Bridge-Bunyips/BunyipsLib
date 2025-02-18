package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages.TwoDeadWheelInputsMessage;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.DriveModel;

/**
 * Standard two dead wheel localizer that uses one parallel and perpendicular encoder in combination
 * with the IMU to localize the robot.
 * <a href="https://github.com/Murray-Bridge-Bunyips/BunyipsLib/blob/4279ed68842ae57d79056dc93efb48a8682380ef/src/main/java/org/murraybridgebunyips/bunyipslib/roadrunner/drive/localizers/TwoWheelLocalizer.java">Source</a>
 *
 * @since 6.0.0
 */
public class TwoWheelLocalizer implements Localizer {
    /**
     * The parallel encoder.
     */
    public final Encoder par;
    /**
     * The perpendicular encoder.
     */
    public final Encoder perp;
    /**
     * The IMU used for heading information.
     */
    public final IMU imu;
    /**
     * Used two-wheel localizer parameters.
     */
    public final Params params;
    private final DriveModel driveModel;
    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;
    private double lastRawHeadingVel, headingVelOffset;
    private boolean initialized;

    /**
     * Create a new TwoWheelLocalizer that will run on two parallel and one perpendicular encoders, with
     * an IMU for heading information.
     *
     * @param driveModel the drive model to use for kinematics
     * @param params     the parameters for the dead wheel localizer
     * @param par        the parallel encoder
     * @param perp       the perpendicular encoder
     * @param imu        the IMU to use for heading
     */
    public TwoWheelLocalizer(@NonNull DriveModel driveModel, @NonNull Params params, @Nullable RawEncoder par, @Nullable RawEncoder perp, @Nullable IMU imu) {
        this.driveModel = driveModel;
        this.params = params;
        this.par = par != null ? new OverflowEncoder(par) : null;
        this.perp = perp != null ? new OverflowEncoder(perp) : null;
        this.imu = imu;

        // Wake up the IMU if it's an IMUEx
        if (imu != null)
            imu.getRobotOrientationAsQuaternion();

        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", params);
    }

    @NonNull
    public Twist2dDual<Time> update() {
        if (imu == null || par == null || perp == null)
            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );

        PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);

        FlightRecorder.write("TWO_DEAD_WHEEL_INPUTS", new TwoDeadWheelInputsMessage(parPosVel, perpPosVel, angles, angularVelocity));

        Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));
        double headingVel = angularVelocity.zRotationRate;

        if (!initialized) {
            initialized = true;

            lastParPos = parPosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        int parPosDelta = parPosVel.position - lastParPos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        // TODO: localizers here complain of null
        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[]{
                                parPosDelta - params.parYTicks * headingDelta,
                                parPosVel.velocity - params.parYTicks * headingVel,
                        }).times(driveModel.inPerTick),
                        new DualNum<Time>(new double[]{
                                perpPosDelta - params.perpXTicks * headingDelta,
                                perpPosVel.velocity - params.perpXTicks * headingVel,
                        }).times(driveModel.inPerTick)
                ),
                new DualNum<>(new double[]{
                        headingDelta,
                        headingVel,
                })
        );

        lastParPos = parPosVel.position;
        lastPerpPos = perpPosVel.position;
        lastHeading = heading;

        return twist;
    }

    /**
     * Parameters for the two dead wheel localizer.
     */
    public static class Params {
        /**
         * y position of the parallel encoder (in tick units)
         */
        public double parYTicks = 0.0;

        /**
         * x position of the perpendicular encoder (in tick units)
         */
        public double perpXTicks = 0.0;

        /**
         * Utility builder for the two dead wheel localizer parameters.
         */
        public static class Builder {
            private final Params params = new Params();

            /**
             * Set the y position of the parallel encoder.
             *
             * @param parYTicks the y position of the parallel encoder (in tick units), as determined by tuning the deadwheel localizer
             * @return the builder
             */
            @NonNull
            public Builder setParYTicks(double parYTicks) {
                params.parYTicks = parYTicks;
                return this;
            }

            /**
             * Set the x position of the perpendicular encoder.
             *
             * @param perpXTicks the x position of the perpendicular encoder (in tick units), as determined by tuning the deadwheel localizer
             * @return the builder
             */
            @NonNull
            public Builder setPerpXTicks(double perpXTicks) {
                params.perpXTicks = perpXTicks;
                return this;
            }

            /**
             * Build the parameters.
             *
             * @return the parameters
             */
            @NonNull
            public Params build() {
                return params;
            }
        }
    }
}
