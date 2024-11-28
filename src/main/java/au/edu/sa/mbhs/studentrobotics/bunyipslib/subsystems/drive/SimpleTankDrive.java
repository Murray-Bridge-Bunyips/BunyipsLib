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
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.Localizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.accumulators.Accumulator;
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
    private Accumulator accumulator;
    private PoseVelocity2d target = Geometry.zeroVel();

    /**
     * Construct a new SimpleTankDrive.
     *
     * @param leftMotors  all motors on the left side of the robot (e.g. {@code Arrays.asList(leftFront, leftBack)})
     * @param rightMotors all motors on the right side of the robot (e.g. {@code Arrays.asList(rightFront, rightBack)})
     */
    public SimpleTankDrive(@NonNull List<DcMotor> leftMotors, @NonNull List<DcMotor> rightMotors) {
        assertParamsNotNull(leftMotors, rightMotors);
        leftMotors.forEach((m) -> {
            if (assertParamsNotNull(m))
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        });
        rightMotors.forEach((m) -> {
            if (assertParamsNotNull(m))
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        });

        this.leftMotors = leftMotors;
        this.rightMotors = rightMotors;
    }

    /**
     * Set the localizer for this drive.
     *
     * @param localizer the localizer to use
     * @return this
     */
    @NonNull
    public SimpleTankDrive withLocalizer(@NonNull Localizer localizer) {
        this.localizer = localizer;
        return this;
    }

    /**
     * Set the pose accumulator this drive instance should use.
     * If not defined, a default {@link Accumulator} will be used when a Localizer is attached.
     *
     * @param accumulator the new accumulator to use
     * @return this
     */
    @NonNull
    public SimpleTankDrive withAccumulator(@NonNull Accumulator accumulator) {
        if (this.accumulator != null)
            this.accumulator.copyTo(accumulator);
        this.accumulator = accumulator;
        return this;
    }

    @Nullable
    public Localizer getLocalizer() {
        return localizer;
    }

    @Override
    protected void periodic() {
        if (localizer != null) {
            Twist2dDual<Time> twist = localizer.update();

            // Auto set to the last known position if the user has not defined one themselves
            if (accumulator == null) {
                accumulator = new Accumulator(Storage.memory().lastKnownPosition);
            }
            accumulator.accumulate(twist);

            opMode(o -> o.telemetry.add("Localizer: X:%in(%/s) Y:%in(%/s) %°(%/s)",
                    Mathf.round(accumulator.getPose().position.x, 1),
                    Mathf.round(accumulator.getVelocity().linearVel.x, 1),
                    Mathf.round(accumulator.getPose().position.y, 1),
                    Mathf.round(accumulator.getVelocity().linearVel.y, 1),
                    Mathf.round(Math.toDegrees(accumulator.getPose().heading.toDouble()), 1),
                    Mathf.round(Math.toDegrees(accumulator.getVelocity().angVel), 1)
            ).color("gray").small());
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

        opMode(o -> o.telemetry.add("%: %\\% %, %\\% %", this,
                Math.round(Math.min(100, Math.abs(target.linearVel.x * 100))), target.linearVel.x >= 0 ? "↑" : "↓",
                Math.round(Math.min(100, Math.abs(target.angVel * 100))), target.angVel >= 0 ? "↺" : "↻"
        ));
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
    public Pose2d getPose() {
        return accumulator != null ? accumulator.getPose() : null;
    }

    @Override
    public void setPose(@NonNull Pose2d newPose) {
        if (accumulator != null)
            accumulator.setPose(newPose);
    }

    @Nullable
    @Override
    public PoseVelocity2d getVelocity() {
        return accumulator != null ? accumulator.getVelocity() : null;
    }
}
