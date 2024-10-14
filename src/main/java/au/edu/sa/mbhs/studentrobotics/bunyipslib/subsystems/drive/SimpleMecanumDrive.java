package au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.hardware.DcMotor;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.Localizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Storage;

/**
 * A minimal Mecanum drive implementation that does not include a RoadRunner configuration.
 * This is useful for implementations where localizer information is not required at all times, such as in TeleOp,
 * and RoadRunner features are not required to be active.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public class SimpleMecanumDrive extends BunyipsSubsystem implements Moveable {
    private final DcMotor leftFront;
    private final DcMotor leftBack;
    private final DcMotor rightBack;
    private final DcMotor rightFront;
    @Nullable
    private Localizer localizer;
    @Nullable
    private Pose2d localizerAccumulatedPose;
    @Nullable
    private PoseVelocity2d localizerVelo;
    private PoseVelocity2d target = Geometry.zeroVel();

    /**
     * Construct a new SimpleMecanumDrive.
     *
     * @param leftFront  the front left motor
     * @param leftBack   the back left motor
     * @param rightBack  the back right motor
     * @param rightFront the front right motor
     */
    public SimpleMecanumDrive(DcMotor leftFront, DcMotor leftBack, DcMotor rightBack, DcMotor rightFront) {
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        this.rightFront = rightFront;
    }

    @Override
    protected void periodic() {
        if (localizer != null) {
            Twist2dDual<Time> twist = localizer.update();
            // Auto set to the last known position if the user has not defined one themselves
            if (localizerAccumulatedPose == null)
                localizerAccumulatedPose = Storage.memory().lastKnownPosition;
            // Accumulate the poses
            localizerAccumulatedPose = localizerAccumulatedPose.plus(twist.value());
            localizerVelo = twist.velocity().value();
        }

        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1)
                .inverse(PoseVelocity2dDual.constant(target, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
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
