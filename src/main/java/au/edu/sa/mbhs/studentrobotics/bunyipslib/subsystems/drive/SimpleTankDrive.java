package au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.Localizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Storage;

/**
 * A minimal Tank drive implementation that does not include a RoadRunner configuration.
 * This is useful for implementations where localizer information is not required at all times, such as in TeleOp,
 * and RoadRunner features are not required to be active.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public class SimpleTankDrive extends BunyipsSubsystem implements Moveable {
    private final List<DcMotor> leftMotors;
    private final List<DcMotor> rightMotors;
    @Nullable
    private Localizer localizer;
    @Nullable
    private Pose2d localizerAccumulatedPose;
    @Nullable
    private PoseVelocity2d localizerVelo;
    private PoseVelocity2d target = Geometry.zeroVel();

    /**
     * Construct a new SimpleTankDrive.
     *
     * @param leftMotors  all motors on the left side of the robot (e.g. {@code Arrays.asList(leftFront, leftBack)})
     * @param rightMotors all motors on the right side of the robot (e.g. {@code Arrays.asList(rightFront, rightBack)})
     */
    public SimpleTankDrive(List<DcMotor> leftMotors, List<DcMotor> rightMotors) {
        assertParamsNotNull(leftMotors, rightMotors);
        leftMotors.forEach(this::assertParamsNotNull);
        rightMotors.forEach(this::assertParamsNotNull);

        this.leftMotors = leftMotors;
        this.rightMotors = rightMotors;
    }

    @Override
    protected void periodic() {
        // TODO: test
        if (localizer != null) {
            Twist2dDual<Time> twist = localizer.update();
            // Auto set to the last known position if the user has not defined one themselves
            if (localizerAccumulatedPose == null)
                localizerAccumulatedPose = Storage.memory().lastKnownPosition;
            // Accumulate the poses
            localizerAccumulatedPose = localizerAccumulatedPose.plus(twist.value());
            localizerVelo = twist.velocity().value();
            Storage.memory().lastKnownPosition = localizerAccumulatedPose;
        }

        TankKinematics.WheelVelocities<Time> wheelVels = new TankKinematics(2)
                .inverse(PoseVelocity2dDual.constant(target, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        for (DcMotor m : leftMotors) {
            m.setPower(wheelVels.left.get(0) / maxPowerMag);
        }
        for (DcMotor m : rightMotors) {
            m.setPower(wheelVels.right.get(0) / maxPowerMag);
        }
    }

    @Override
    protected void onDisable() {
        leftMotors.forEach(m -> m.setPower(0));
        rightMotors.forEach(m -> m.setPower(0));
    }

    @Override
    public void setPower(@NonNull PoseVelocity2d target) {
        this.target = target;
    }

    @Nullable
    @Override
    public Pose2d getPoseEstimate() {
        return localizerAccumulatedPose;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d newPose) {
        localizerAccumulatedPose = newPose;
    }

    @Nullable
    @Override
    public PoseVelocity2d getPoseVelocity() {
        return localizerVelo;
    }
}
