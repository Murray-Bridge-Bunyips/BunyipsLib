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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

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
    public PinpointLocalizer(@NonNull DriveModel driveModel, @NonNull TwoWheelLocalizer.Params params, @Nullable GoBildaPinpointDriver pinpoint) {
        this.pinpoint = pinpoint;

        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", params);
        if (pinpoint == null)
            return;

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
}
