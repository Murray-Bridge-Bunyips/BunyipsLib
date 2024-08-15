package org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * SwitchableLocalizer is a composite localizer that allows self-tests to be performed, and to allow a "fallback"
 * localizer to kick in if the self-test fails.
 *
 * @author Lucas Bubner, 2024
 */
//@Config
/* public TODO: WIP */ class SwitchableLocalizer implements Localizer {
    /**
     * Whether the fallback localizer should be used.
     */
    public static boolean USING_FALLBACK_LOCALIZER = false;
    private final Localizer main;
    private final Localizer fallback;
    /**
     * Available tasks for self-tests on this localizer.
     */
    public Tasks tasks = new Tasks();
    private Pose2d poseEstimate = new Pose2d();
    private Pose2d poseVel = new Pose2d();

    /**
     * Create a new SwitchableLocalizer.
     *
     * @param main     the main localizer to use
     * @param fallback the fallback localizer to use, activated if the self-test task fails, or by the user.
     */
    public SwitchableLocalizer(Localizer main, Localizer fallback) {
        USING_FALLBACK_LOCALIZER = false;
        this.main = main;
        this.fallback = fallback;
    }

    /**
     * Switch to the main localizer for all pose data.
     */
    public void switchToMain() {
        USING_FALLBACK_LOCALIZER = false;
    }

    /**
     * Switch to the backup localizer for all pose data.
     */
    public void switchToFallback() {
        USING_FALLBACK_LOCALIZER = true;
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        poseEstimate = pose2d;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return poseVel;
    }

    @Override
    public void update() {
        // Update all localizers regardless
        main.update();
        fallback.update();

        if (USING_FALLBACK_LOCALIZER) {
            poseEstimate = fallback.getPoseEstimate();
            poseVel = fallback.getPoseVelocity();
            return;
        }
        poseEstimate = main.getPoseEstimate();
        poseVel = main.getPoseVelocity();
    }

    /**
     * Self-test tasks for SwitchableLocalizer.
     */
    public class Tasks {
        /**
         * Auto observes and tests the main localizer and falls back to the fallback localizer if the test fails.
         *
         * @return a task to perform the main localizer test - will change localizer status when this task finishes.
         */
        public Task autoTestMainLocalizer() {
            return new Task() {
                private Telemetry.Item telemetry;

                @Override
                protected void init() {
                    // TODO: in progress
                    telemetry = opMode.telemetry.add("").bold();
                }

                @Override
                protected void periodic() {
//                    if (opMode.gamepad1.left_bumper) {
//
//                    }
                }

                @Override
                protected boolean isTaskFinished() {
                    // Timeout or manual exit only
                    return false;
                }
            };
        }
    }
}
