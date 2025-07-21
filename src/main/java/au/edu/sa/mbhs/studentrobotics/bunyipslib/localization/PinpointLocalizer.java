package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages.PoseMessage;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.DriveModel;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;

/**
 * Standard goBILDA® Pinpoint Computer localizer for two-wheel configurations.
 * <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/06d2cb08df827b88cf0286246c44d936c41f6a31/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/PinpointLocalizer.java">Source</a>
 * <p>
 * <b>Note:</b> The Pinpoint localizer uses the {@link TwoWheelLocalizer.Params} object as it uses the same principles and
 * requires the same hardware.
 *
 * @since 7.0.0
 */
public class PinpointLocalizer implements Localizer {
    /**
     * The Pinpoint Computer in use.
     */
    public final GoBildaPinpointDriver pinpoint;
    /**
     * Used two-wheel and Pinpoint direction localizer parameters.
     */
    public final Params params;

    private Pose2d lastPose = Geometry.zeroPose();

    /**
     * Create a new Pinpoint localizer operating on two internally connected dead wheels.
     *
     * @param driveModel the drive model to use for kinematics
     * @param params     parameters to use, note this set of parameters is an extension of the two-wheel localizer parameters.
     *                   Additional parameters exclusive to Pinpoint are the initial directions, which should be set here
     *                   instead of directly on the driver to ensure tuning support. These directions are auto-applied to the driver.
     * @param pinpoint   the Pinpoint Computer from HardwareMap. Will be IMU and position reset on construction. To set wheel directions,
     *                   set them in the {@code params} object, <b>NOT</b> on this object directly.
     */
    public PinpointLocalizer(@NonNull DriveModel driveModel, @NonNull PinpointLocalizer.Params params, @Nullable GoBildaPinpointDriver pinpoint) {
        this.pinpoint = pinpoint;
        this.params = params;

        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", params);
        if (pinpoint == null)
            return;

        // Directions are now applied, it is important to ensure the user does not do this themselves to not break tuning
        pinpoint.setEncoderDirections(params.initialParDirection, params.initialPerpDirection);

        // Pinpoint uses millimeters as its standard unit
        pinpoint.setEncoderResolution(1 / driveModel.inPerTick, DistanceUnit.INCH);
        pinpoint.setOffsets(driveModel.inPerTick * params.parYTicks, driveModel.inPerTick * params.perpXTicks, DistanceUnit.INCH);

        pinpoint.resetPosAndIMU();
    }

    @NonNull
    @Override
    public Twist2dDual<Time> update() {
        if (pinpoint != null)
            pinpoint.update();
        if (pinpoint == null || pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY)
            return new Twist2dDual<>(Vector2dDual.constant(Geometry.zeroVec(), 2), DualNum.constant(0, 2));

        Pose2d pose = new Pose2d(
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getHeading(AngleUnit.RADIANS)
        );
        PoseVelocity2d vel = new PoseVelocity2d(
                // Note: Pinpoint velocity is in the global reference frame
                // https://discord.com/channels/225450307654647808/356086067033538570/1340040945470804022
                pose.heading.inverse().times(new Vector2d(pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH))),
                pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
        );

        FlightRecorder.write("PINPOINT_POSE", new PoseMessage(pose));
        FlightRecorder.write("PINPOINT_VELOCITY", new PoseMessage(vel));

        Vector2d positionDelta = pose.heading.inverse().times(pose.position.minus(lastPose.position));
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
     * Parameters for the Pinpoint. Note this extends the two-wheel localizer and supplies two additional fields
     * to set the direction of these wheels.
     */
    public static class Params extends TwoWheelLocalizer.Params {
        // Since the driver does not track the direction, we have to, so we track the initial directions here. This
        // provides tuning support and is accessed by the tuning OpMode.
        /**
         * The initial tracking direction of the parallel wheel. This is not updated when the {@link GoBildaPinpointDriver}
         * direction is updated, but will be applied to the driver at time of localizer construction.
         */
        public GoBildaPinpointDriver.EncoderDirection initialParDirection;
        /**
         * The initial tracking direction of the perpendicular wheel. This is not updated when the {@link GoBildaPinpointDriver}
         * direction is updated, but will be applied to the driver at time of localizer construction.
         */
        public GoBildaPinpointDriver.EncoderDirection initialPerpDirection;

        /**
         * Utility builder for the two wheel and Pinpoint directions parameters.
         */
        public static class Builder extends TwoWheelLocalizer.Params.Builder {
            /**
             * Create a new builder.
             */
            public Builder() {
                params = new PinpointLocalizer.Params();
            }

            /**
             * Sets the initial tracking direction of the parallel wheel, which will be sent to the Pinpoint driver
             * on construction of the localizer. This initial direction will be stored in the parameters object and is used for tuning.
             * <p>
             * During initialisation, <b>you must</b> set the direction of your Pinpoint wheels here using these methods,
             * otherwise tuning will not work. <b>Do not</b> set the directions of your wheels directly on the driver, it is applied here.
             *
             * @param initialParDirection the direction of the parallel wheel such that forward movement is positive reading
             * @return the builder
             */
            @NonNull
            public TwoWheelLocalizer.Params.Builder setInitialParDirection(GoBildaPinpointDriver.EncoderDirection initialParDirection) {
                ((Params) params).initialParDirection = initialParDirection;
                return this;
            }

            /**
             * Sets the initial tracking direction of the perpendicular wheel, which will be sent to the Pinpoint driver
             * on construction of the localizer. This initial direction will be stored in the parameters object and is used for tuning.
             * <p>
             * During initialisation, <b>you must</b> set the direction of your Pinpoint wheels here using these methods,
             * otherwise tuning will not work. <b>Do not</b> set the directions of your wheels directly on the driver, it is applied here.
             *
             * @param initialParDirection the direction of the perpendicular wheel such that left movement is positive reading
             * @return the builder
             */
            @NonNull
            public TwoWheelLocalizer.Params.Builder setInitialPerpDirection(GoBildaPinpointDriver.EncoderDirection initialParDirection) {
                ((Params) params).initialParDirection = initialParDirection;
                return this;
            }

            /**
             * Build the parameters.
             *
             * @return the parameters
             */
            @NonNull
            public PinpointLocalizer.Params build() {
                return (PinpointLocalizer.Params) params;
            }
        }
    }
}
