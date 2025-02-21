package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.GoBildaPinpointDriver;
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
    private static final double INCH_MM_CONVERSION_FACTOR = 25.4;
    /**
     * The Pinpoint Computer in use.
     */
    public final GoBildaPinpointDriver pinpoint;

    private Pose2d lastPose = Geometry.zeroPose();

    /**
     * Create a new Pinpoint localizer operating on two internally connected dead wheels.
     *
     * @param driveModel the drive model to use for kinematics
     * @param params     parameters to use, note since they are the same as if you were to use two wheels alone, the two wheel localizer
     *                   parameters object is being used.
     * @param pinpoint   the Pinpoint Computer from HardwareMap. Will be IMU and position reset on construction.
     */
    public PinpointLocalizer(DriveModel driveModel, TwoWheelLocalizer.Params params, GoBildaPinpointDriver pinpoint) {
        this.pinpoint = pinpoint;

        // Pinpoint uses millimeters as its standard unit
        double mmPerTick = INCH_MM_CONVERSION_FACTOR * driveModel.inPerTick;
        pinpoint.setEncoderResolution(1 / mmPerTick);
        pinpoint.setOffsets(mmPerTick * params.parYTicks, mmPerTick * params.perpXTicks);

        pinpoint.resetPosAndIMU();

        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", params);
    }

    @NonNull
    @Override
    public Twist2dDual<Time> update() {
        if (pinpoint != null)
            pinpoint.update();
        if (pinpoint == null || pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY)
            return new Twist2dDual<>(Vector2dDual.constant(Geometry.zeroVec(), 2), DualNum.constant(0, 2));

        Pose2d pose = new Pose2d(
                pinpoint.getPosX() / INCH_MM_CONVERSION_FACTOR,
                pinpoint.getPosY() / INCH_MM_CONVERSION_FACTOR,
                pinpoint.getHeading()
        );
        PoseVelocity2d vel = new PoseVelocity2d(
                Rotation2d.exp(pinpoint.getHeading()) // TODO: test
                        .inverse()
                        .times(new Vector2d(pinpoint.getVelX() / INCH_MM_CONVERSION_FACTOR, pinpoint.getVelY() / INCH_MM_CONVERSION_FACTOR)),
                pinpoint.getHeadingVelocity()
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
}
