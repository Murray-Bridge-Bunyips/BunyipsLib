package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OTOSKt;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages.PoseMessage;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;

/**
 * Standard SparkFun OTOS localizer.
 * <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/06d2cb08df827b88cf0286246c44d936c41f6a31/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OTOSLocalizer.java">Source</a>
 *
 * @since 7.0.0
 */
public class OTOSLocalizer implements Localizer {
    /**
     * OTOS used for localization.
     */
    public final SparkFunOTOS otos;
    /**
     * Used OTOS parameters.
     */
    public final Params params;

    private Pose2d lastPose = Geometry.zeroPose();

    /**
     * Create a new OTOS localizer that uses this OTOS.
     *
     * @param otos the OTOS to use. Do note this localizer must use linear and angular units of inches and radians
     *             to conform to the standard geometry standards of RoadRunner. Do not modify them yourself.
     *             Constructing this localizer will perform a <b>blocking</b> IMU calibration, which will last for
     *             roughly 600 milliseconds.
     */
    public OTOSLocalizer(@NonNull Params params, @NonNull SparkFunOTOS otos) {
        this.params = params;
        this.otos = otos;

        // RR standard units. Modifying them will result in unit conversions issues.
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        // Blocking
        otos.calibrateImu();

        otos.setLinearScalar(params.linearScalar);
        otos.setAngularScalar(params.angularScalar);
        otos.setOffset(params.offset);

        otos.resetTracking();

        FlightRecorder.write("OTOS_PARAMS", params);
    }

    @NonNull
    @Override
    public Twist2dDual<Time> update() {
        if (otos == null)
            return new Twist2dDual<>(Vector2dDual.constant(Geometry.zeroVec(), 2), DualNum.constant(0, 2));

        SparkFunOTOS.Pose2D position = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D velocity = new SparkFunOTOS.Pose2D();
        // Acceleration is unused, but we burst read so we only need to do it once
        otos.getPosVelAcc(position, velocity, new SparkFunOTOS.Pose2D());

        Pose2d pose = OTOSKt.toRRPose(position);
        PoseVelocity2d vel = new PoseVelocity2d(OTOSKt.toRRPose(velocity).position, velocity.h);

        FlightRecorder.write("OTOS_POSE", new PoseMessage(pose));
        FlightRecorder.write("OTOS_VELOCITY", new PoseMessage(vel));

        // TODO: needs testing
        Vector2d positionDelta = pose.position.minus(lastPose.position);
        double headingDelta = pose.heading.minus(lastPose.heading);
        lastPose = pose;

        return new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<>(new double[]{positionDelta.x, vel.linearVel.x}),
                        new DualNum<>(new double[]{positionDelta.y, vel.linearVel.y})
                ),
                new DualNum<>(new double[]{headingDelta, vel.angVel})
        );
    }

    /**
     * Parameters for the OTOS.
     */
    public static class Params {
        /**
         * Multiplicative scale for linear tracking to correct for inaccuracies.
         */
        public double linearScalar = 1.0;
        /**
         * Multiplicative scale for angular tracking to correct for inaccuracies.
         */
        public double angularScalar = 1.0;
        /**
         * The offset of the OTOS relative to the center of the robot.
         * <p>
         * Measured in Cartesian coordinates (+x right inches, +y forward inches, +h ccw radians) under
         * the RR default scaling units (inches, radians).
         */
        public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);

        public static class Builder {
            private final Params params = new Params();
            private final ArrayList<Double> linearScalarMeasurements = new ArrayList<>();
            private final ArrayList<Double> angularScalarMeasurements = new ArrayList<>();

            /**
             * Set the compensation factor for linear scaling. Can be any value between 0.872 to 1.127 in increments
             * of 0.001 (0.1%). It is recommended to tune the angular scalar before the linear scalar through
             * the RoadRunner tuners.
             * <p>
             * <b>Note</b>: Calling this function multiple times will take the average between other calls that have set
             * the linear scalar. This allows for multiple tuning session results to average in on the most accurate value.
             *
             * @param linearScalar the multiplicative linear scalar
             * @return this builder
             */
            @NonNull
            public Builder setLinearScalar(double linearScalar) {
                linearScalarMeasurements.add(linearScalar);
                return this;
            }

            /**
             * Set the compensation factor for linear scaling as a result of a tuning session based on measured vs
             * reported distance. This is the method the tuners use and will auto-convert the given parameters into
             * a measurement.
             * <p>
             * Assumes the default unit of inches when converting.
             * <p>
             * <b>Note</b>: Calling this function multiple times will take the average between other calls that have set
             * the linear scalar. This allows for multiple tuning session results to average in on the most accurate value.
             *
             * @param realDistance     the tape-measure real world value the robot has been pushed
             * @param reportedDistance the distance the OTOS thinks it has travelled
             * @return this builder
             */
            @NonNull
            public Builder setMeasuredLinearScalar(@NonNull Measure<Distance> realDistance, @NonNull Measure<Distance> reportedDistance) {
                linearScalarMeasurements.add(realDistance.in(Inches) / reportedDistance.in(Inches));
                return this;
            }

            /**
             * Set the compensation factor for angular scaling. Can be any value between 0.872 to 1.127 in increments
             * of 0.001 (0.1%). It is recommended to tune this scalar first through the RoadRunner tuners.
             * <p>
             * <b>Note</b>: Calling this function multiple times will take the average between other calls that have set
             * the angular scalar. This allows for multiple tuning session results to average in on the most accurate value.
             *
             * @param angularScalar the multiplicative angular scalar
             * @return this builder
             */
            @NonNull
            public Builder setAngularScalar(double angularScalar) {
                angularScalarMeasurements.add(angularScalar);
                return this;
            }

            /**
             * Set the compensation factor for angular scaling as a result of a tuning session based on measured vs
             * reported turns. This is the method the tuners use but also returns directly as a scalar. You can
             * use either method you wish to achieve a tuned result.
             * <p>
             * Assumes the default unit of radians when converting.
             * <p>
             * <b>Note</b>: Calling this function multiple times will take the average between other calls that have set
             * the angular scalar. This allows for multiple tuning session results to average in on the most accurate value.
             *
             * @param angleRotated     the actual amount of angle your OTOS has been rotated
             * @param reportedRotation the angle your OTOS reports it has rotated
             * @return this builder
             */
            @NonNull
            public Builder setMeasuredAngularScalar(@NonNull Measure<Angle> angleRotated, @NonNull Measure<Angle> reportedRotation) {
                angularScalarMeasurements.add(angleRotated.in(Radians) / reportedRotation.in(Radians));
                return this;
            }

            /**
             * Specifies an offset for the sensor relative to the center of the robot.
             * <p>
             * Measured in default geometry units, which is inches and radians.
             * <p>
             * One of the RoadRunner tuners can assist in finding this offset.
             *
             * @param offset the offset of the robot in (+x right, +y forward, +h ccw)
             * @return this builder
             */
            public Builder setOffset(SparkFunOTOS.Pose2D offset) {
                params.offset = offset;
                return this;
            }

            /**
             * Specifies the forward component for the sensor offset relative to the center of the robot.
             * <p>
             * Units are auto-converted into inches, which is the default distance unit.
             *
             * @param forwardOffset the distance +forward the OTOS is placed relative to the center of the robot.
             * @return this builder
             */
            public Builder setForwardOffset(Measure<Distance> forwardOffset) {
                params.offset = new SparkFunOTOS.Pose2D(params.offset.x, forwardOffset.in(Inches), params.offset.h);
                return this;
            }

            /**
             * Specifies the strafe component for the sensor offset relative to the center of the robot.
             * <p>
             * Units are auto-converted into inches, which is the default distance unit.
             *
             * @param leftOffset the distance +left the OTOS is placed relative to the center of the robot.
             * @return this builder
             */
            public Builder setLeftOffset(Measure<Distance> leftOffset) {
                params.offset = new SparkFunOTOS.Pose2D(leftOffset.negate().in(Inches), params.offset.y, params.offset.h);
                return this;
            }

            /**
             * Specifies the rotational component for the sensor offset relative to the center of the robot.
             * <p>
             * Units are auto-converted into radians, which is the default angular unit.
             *
             * @param ccwOffset the rotation +counter-clockwise of the OTOS placed relative to the forward direction of your robot
             * @return this builder
             */
            public Builder setAngularOffset(Measure<Angle> ccwOffset) {
                params.offset = new SparkFunOTOS.Pose2D(params.offset.x, params.offset.y, ccwOffset.in(Radians));
                return this;
            }

            /**
             * Build the parameters.
             *
             * @return the parameters
             */
            @NonNull
            public Params build() {
                params.linearScalar = !linearScalarMeasurements.isEmpty()
                        ? linearScalarMeasurements.stream().reduce(0.0, Double::sum) / linearScalarMeasurements.size()
                        : 0.0;
                params.angularScalar = !angularScalarMeasurements.isEmpty()
                        ? angularScalarMeasurements.stream().reduce(0.0, Double::sum) / linearScalarMeasurements.size()
                        : 0.0;
                return params;
            }
        }
    }
}