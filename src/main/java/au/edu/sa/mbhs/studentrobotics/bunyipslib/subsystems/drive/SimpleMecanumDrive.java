package au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive;


import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.hardware.DcMotor;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.Localizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.accumulators.Accumulator;
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
    /**
     * Forward speed.
     */
    public double speedX = 0;
    /**
     * Left speed.
     */
    public double speedY = 0;
    /**
     * Counter-clockwise rotational speed.
     */
    public double speedR = 0;
    @Nullable
    private Localizer localizer;
    @Nullable
    private Accumulator accumulator;
    private Priority priority = Priority.NORMALISED;

    /**
     * Construct a new SimpleMecanumDrive.
     *
     * @param leftFront  the front left motor
     * @param leftBack   the back left motor
     * @param rightBack  the back right motor
     * @param rightFront the front right motor
     */
    public SimpleMecanumDrive(DcMotor leftFront, DcMotor leftBack, DcMotor rightBack, DcMotor rightFront) {
        assertParamsNotNull(leftFront, leftBack, rightBack, rightFront);
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
            if (accumulator == null) {
                accumulator = new Accumulator(Storage.memory().lastKnownPosition);
            }
            accumulator.accumulate(twist);

            opMode(o -> o.telemetry.add("Localizer: X:%in(%/s) Y:%in(%/s) %deg(%/s)",
                    Mathf.round(accumulator.getPoseEstimate().position.x, 1),
                    Mathf.round(accumulator.getPoseVelocity().linearVel.x, 1),
                    Mathf.round(accumulator.getPoseEstimate().position.y, 1),
                    Mathf.round(accumulator.getPoseVelocity().linearVel.y, 1),
                    Mathf.round(Math.toDegrees(accumulator.getPoseEstimate().heading.toDouble()), 1),
                    Mathf.round(Math.toDegrees(accumulator.getPoseVelocity().angVel), 1)
            ).color("gray"));
        }

        double leftFrontPower, leftBackPower, rightBackPower, rightFrontPower;

        if (priority == Priority.ROTATIONAL) {
            // Raw powers for translation and rotation in configuration (front left, front right, back left, back right)
            double[] translationValues = {
                    speedY + speedX,
                    speedY - speedX,
                    speedY - speedX,
                    speedY + speedX
            };
            double[] rotationValues = {
                    speedR,
                    -speedR,
                    speedR,
                    -speedR
            };

            // Try to find the maximum power possible we can allocate to translation by scaling translation depending
            // on the desired rotation. This always ensures our rotational velocity will be achieved at the cost of
            // reduced translational velocity.
            double scaleFactor = 1.0;

            for (int i = 0; i < 4; i++) {
                double combinedPower = translationValues[i] + rotationValues[i];
                // Rescale translation to give more power to rotation if it exceeds motor limits
                if (Math.abs(combinedPower) > 1) {
                    double availablePower = 1 - Math.min(1, Math.abs(rotationValues[i]));
                    double requiredScale = availablePower / Math.abs(translationValues[i]);

                    // Update scaleFactor to the lowest required value to stay within motor limits
                    if (requiredScale < scaleFactor) {
                        scaleFactor = requiredScale;
                    }
                }
            }

            // Apply scaling
            leftFrontPower = translationValues[0] * scaleFactor + rotationValues[0];
            rightFrontPower = translationValues[1] * scaleFactor + rotationValues[1];
            leftBackPower = translationValues[2] * scaleFactor + rotationValues[2];
            rightBackPower = translationValues[3] * scaleFactor + rotationValues[3];
        } else {
            // Calculate motor powers
            leftFrontPower = speedY + speedX + speedR;
            rightFrontPower = speedY - speedX - speedR;
            leftBackPower = speedY - speedX + speedR;
            rightBackPower = speedY + speedX - speedR;

            double maxPower = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower), Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))));
            // If the maximum number is greater than 1.0, then normalise by that number
            if (maxPower > 1.0) {
                leftFrontPower = leftFrontPower / maxPower;
                rightFrontPower = rightFrontPower / maxPower;
                leftBackPower = leftBackPower / maxPower;
                rightBackPower = rightBackPower / maxPower;
            }
        }

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);

        opMode(o -> o.telemetry.add("%: %\\% %, %\\% %, %\\% %", this,
                Math.round(speedX * 100), speedX >= 0 ? "↑" : "↓",
                Math.round(speedY * 100), speedY >= 0 ? "←" : "→",
                Math.round(speedR * 100), speedR >= 0 ? "↺" : "↻"
        ));
    }

    /**
     * @param priority the drive power priority
     * @return this
     */
    public SimpleMecanumDrive setPriority(Priority priority) {
        this.priority = priority;
        return this;
    }

    /**
     * Swap the priority of the drive system.
     *
     * @return this
     */
    public SimpleMecanumDrive swapPriority() {
        priority = priority == Priority.NORMALISED ? Priority.ROTATIONAL : Priority.NORMALISED;
        return this;
    }

    /**
     * Set the localizer for this drive.
     *
     * @param localizer the localizer to use
     * @return this
     */
    public SimpleMecanumDrive withLocalizer(Localizer localizer) {
        this.localizer = localizer;
        return this;
    }

    /**
     * Set the pose accumulator this drive instance should use.
     * If not defined, a default {@link Accumulator} will be used when a Localizer is attached.
     *
     * @param accumulator the  accumulator to use
     * @return this
     */
    public SimpleMecanumDrive withAccumulator(@NonNull Accumulator accumulator) {
        if (this.accumulator != null)
            this.accumulator.copyTo(accumulator);
        this.accumulator = accumulator;
        return this;
    }

    @Override
    protected void onDisable() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    @Override
    public void setPower(@NonNull PoseVelocity2d target) {
        speedX = target.linearVel.x;
        speedY = target.linearVel.y;
        speedR = target.angVel;
    }

    @Nullable
    @Override
    public Pose2d getPoseEstimate() {
        return accumulator != null ? accumulator.getPoseEstimate() : null;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d newPose) {
        if (accumulator != null)
            accumulator.setPoseEstimate(newPose);
    }

    @Nullable
    @Override
    public PoseVelocity2d getPoseVelocity() {
        return accumulator != null ? accumulator.getPoseVelocity() : null;
    }

    /**
     * Mecanum control prioritisation.
     */
    public enum Priority {
        /**
         * Calculate translational speeds first.
         */
        NORMALISED,
        /**
         * Calculate rotational speeds first, and use the rest for translation.
         */
        ROTATIONAL
    }
}
