package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches;

import android.util.Pair;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.UnaryFunction;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;

/**
 * Gamepad drive that can define certain "exclusion points" on the field with a range of effect radius, disallowing
 * movement and presence in these zones by pushing the drive vector away on proximity.
 * <p>
 * Multiple zones can be created and a response curve can be generated via a normalised push scaling function.
 *
 * @author Lucas Bubner, 2026
 * @since 8.2.0
 */
public class BlockableAreaDriveTask extends FieldOrientableDriveTask {
    private final Supplier<PoseVelocity2d> vel;
    private final ArrayList<Pair<Vector2d, Double>> blockableAreas = new ArrayList<>();
    private UnaryFunction pushScaling = x -> 12 * x * x;

    /**
     * Constructor for BlockableAreaDriveTask.
     *
     * @param targetVecNormalised The supplier for Robot velocity input, magnitude [-1, 1] of max robot velocity
     * @param drive               The holonomic drive to use, which you must ensure is holonomic as strafe commands will be
     *                            called unlike the differential control task. This task will be auto-attached to this BunyipsSubsystem
     *                            if possible.
     */
    public BlockableAreaDriveTask(@NonNull Supplier<PoseVelocity2d> targetVecNormalised, @NonNull Moveable drive) {
        super.drive = drive;
        if (drive instanceof BunyipsSubsystem)
            on((BunyipsSubsystem) drive, false);
        vel = targetVecNormalised;
        named("Holonomic Control (Blockable Areas)");
    }

    /**
     * Constructor for BlockableAreaDriveTask using a default Mecanum binding.
     * Left stick controls translation, right stick controls rotation.
     *
     * @param driver The gamepad to use for driving in a standard configuration
     * @param drive  The holonomic drive to use, which you must ensure is holonomic as strafe commands will be
     *               called unlike the differential control task. This task will be auto-attached to this BunyipsSubsystem
     *               if possible.
     */
    public BlockableAreaDriveTask(@NonNull Gamepad driver, @NonNull Moveable drive) {
        this(() -> Controls.vel(driver.left_stick_x, driver.left_stick_y, driver.right_stick_x), drive);
    }

    /**
     * Primary builder-like method to add a new exclusion point with radius that will be respected by localization avoidance.
     *
     * @param fieldOrigin        the point on the field to start the exclusion radius
     * @param radiusAreaOfEffect the radius extending from the field origin to decreasingly apply exclusion "push" to
     * @return this task
     */
    public BlockableAreaDriveTask add(@NonNull Vector2d fieldOrigin, @NonNull Measure<Distance> radiusAreaOfEffect) {
        blockableAreas.add(new Pair<>(fieldOrigin, radiusAreaOfEffect.in(Inches)));
        return this;
    }

    /**
     * Clears all exclusion points added by {@link #add(Vector2d, Measure)}.
     *
     * @return this task
     */
    public BlockableAreaDriveTask clear() {
        blockableAreas.clear();
        return this;
    }

    /**
     * Specifies a function to scale the pushing coefficient used for velocity correction.
     * <p>
     * For example, a stronger push would facilitate a higher strength. By default, the push scaling is {@code y=12x^2},
     * giving sharp pushing at higher values and aggressive smoothing near the boundary.
     *
     * @param strengthModifier passes in a normalised scalar in [0, 1] representing the distance from the edge of an applicable radius to the field origin,
     *                         and expects a new scalar to scale the correction by based on this distance or other means. For example, a constant strength
     *                         of 1 will use all force necessary to push away from the outer zone.
     * @return this task
     */
    public BlockableAreaDriveTask withPushScaling(UnaryFunction strengthModifier) {
        pushScaling = strengthModifier;
        return this;
    }

    @Override
    protected void periodic() {
        Pose2d poseEstimate = drive.getPose();
        if (poseEstimate == null)
            throw new IllegalStateException("BlockableAreaDriveTask requires a localizer to be attached to the drive system!");
        PoseVelocity2d userInput = applyOrientation(vel.get());
        Canvas overlay = dashboard.fieldOverlay();
        for (Pair<Vector2d, Double> area : blockableAreas) {
            Vector2d diff = poseEstimate.position.minus(area.first);
            double mag = diff.norm();
            if (mag < 1.0e-6) {
                // Vector is too small as we hit floating point limitations, instead we pick an arbitrary outwards direction
                diff = new Vector2d(1, 0);
                mag = 1;
            }
            if (mag < area.second) {
                // Use the direction vector as the basis and scale by the distance (minimum distance, max push)
                double push = pushScaling.apply(1 - (mag / area.second));
                PoseVelocity2d correction = poseEstimate.heading.inverse()
                        .times(new PoseVelocity2d(diff.div(mag).times(push), 0));
                userInput = new PoseVelocity2d(userInput.linearVel.plus(correction.linearVel), userInput.angVel);
                overlay.setStroke("#00c911")
                        .strokeLine(poseEstimate.position.x, poseEstimate.position.y,
                                poseEstimate.position.x + correction.linearVel.x * mag,
                                poseEstimate.position.y + correction.linearVel.y * mag)
                        .setFill("#dd2c0066");
            } else {
                overlay.setFill("#ffa99466");
            }
            overlay.fillCircle(area.first.x, area.first.y, area.second);
        }
        drive.setPower(userInput);
    }

    @Override
    protected void onFinish() {
        drive.setPower(Geometry.zeroVel());
    }
}
