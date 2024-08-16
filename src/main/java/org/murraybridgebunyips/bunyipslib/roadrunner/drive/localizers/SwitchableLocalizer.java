package org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Radians;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.murraybridgebunyips.bunyipslib.Cartesian;
import org.murraybridgebunyips.bunyipslib.external.Mathf;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * SwitchableLocalizer is a composite localizer that allows self-tests to be performed, and to allow a "fallback"
 * localizer to kick in if the self-test fails.
 *
 * @author Lucas Bubner, 2024
 */
@Config
public class SwitchableLocalizer implements Localizer {
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
     * Switch to the main localizer for all pose data. Note that pose data is not shared between localizers.
     */
    public void switchToMain() {
        USING_FALLBACK_LOCALIZER = false;
    }

    /**
     * Switch to the backup localizer for all pose data. Note that pose data is not shared between localizers.
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
        // Update all localizers regardless, these localizers will supply their own pose estimates and
        // should not mix together.
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
                private Vector2d capture;

                private boolean forwardCheck;
                private boolean strafeCheck;

                private void update(String text) {
                    telemetry.setValue("\nLocalizer Auto Test in progress.\n" +
                            "<font color='gray'>gamepad1.left_bumper to early abort and fail the test.\n" +
                            "gamepad1.right_bumper to force pass the test.</font>\n" + text + "\n" + main.getPoseEstimate() + "\n");
                }

                @Override
                protected void init() {
                    telemetry = opMode.telemetry.addRetained("Initialising Localizer Self Test...").bold();
                    Pose2d mainPose = main.getPoseEstimate();
                    capture = Cartesian.rotate(Cartesian.fromPose(mainPose).vec(), Radians.of(mainPose.getHeading()).negate());
                }

                @Override
                protected void periodic() {
                    main.update();
                    if (opMode.gamepad1.left_bumper) {
                        forwardCheck = false;
                        strafeCheck = false;
                        finish();
                        return;
                    }
                    if (opMode.gamepad1.right_bumper) {
                        forwardCheck = true;
                        strafeCheck = true;
                        finish();
                        return;
                    }
                    Pose2d curr = main.getPoseEstimate();
                    // Always calculate error forward, using cartesian coordinates
                    Vector2d error = capture.minus(Cartesian.rotate(Cartesian.fromPose(curr).vec(), Radians.of(curr.getHeading()).negate()));
                    if (!forwardCheck) {
                        update("<font color='red'>Please move the robot forward a minimum of 10 inches.</font>");
                        if (!Mathf.isNear(error.getY(), capture.getY(), 10)) {
                            forwardCheck = true;
                        }
                        return;
                    }
                    if (!strafeCheck) {
                        update("<font color='yellow'>Please move the robot sideways a minimum of 10 inches.</font>");
                        if (!Mathf.isNear(error.getX(), capture.getX(), 10)) {
                            strafeCheck = true;
                        }
                        return;
                    }
                    update("<font color='green'>Self check passed.</font>");
                }

                @Override
                protected void onFinish() {
                    telemetry.setRetained(false);
                    if (!forwardCheck || !strafeCheck) {
                        opMode.telemetry.log("<font color='yellow'>Localizer test failed. Falling back to backup localizer.</font>");
                        USING_FALLBACK_LOCALIZER = true;
                        return;
                    }
                    USING_FALLBACK_LOCALIZER = false;
                    opMode.telemetry.log("<font color='green'>Localizer test passed.</font>");
                }

                @Override
                protected boolean isTaskFinished() {
                    // Timeout or manual exit only
                    return false;
                }
            }.withName("Auto Test Main Localizer");
        }

        /**
         * Tests the main localizer and falls back to the fallback localizer if the user wishes to fail the test.
         *
         * @return a task to test the localizer manually using human judgement, will propagate result on user input
         */
        public Task manualTestMainLocalizer() {
            return new Task() {
                private Telemetry.Item telemetry;
                private Vector2d capture;

                @Override
                protected void init() {
                    telemetry = opMode.telemetry.addRetained("Initialising Localizer Test...").bold();
                    Pose2d mainPose = main.getPoseEstimate();
                    capture = Cartesian.rotate(Cartesian.fromPose(mainPose).vec(), Radians.of(mainPose.getHeading()).negate());
                }

                @Override
                protected void periodic() {
                    main.update();
                    if (opMode.gamepad1.left_bumper) {
                        USING_FALLBACK_LOCALIZER = true;
                        opMode.telemetry.log("<font color='yellow'>Localizer test failed. Falling back to backup localizer.</font>");
                        telemetry.setValue("");
                        finish();
                        return;
                    }
                    if (opMode.gamepad1.right_bumper) {
                        USING_FALLBACK_LOCALIZER = false;
                        opMode.telemetry.log("<font color='green'>Localizer test passed.</font>");
                        telemetry.setValue("");
                        finish();
                        return;
                    }
                    Pose2d curr = main.getPoseEstimate();
                    telemetry.setValue("\nLocalizer Test in progress.\n" +
                            "<font color='gray'>gamepad1.left_bumper to fail the test.\n" +
                            "gamepad1.right_bumper to pass the test. Pass the test if the below values are increasing properly.</font>\n" +
                            "Raw: " + curr + "\n" +
                            "Error (rotated): " + capture.minus(Cartesian.rotate(Cartesian.fromPose(curr).vec(), Radians.of(curr.getHeading()).negate())) + "\n");
                }

                @Override
                protected void onFinish() {
                    telemetry.setRetained(false);
                }

                @Override
                protected boolean isTaskFinished() {
                    // User input only
                    return false;
                }
            }.withName("Manual Test Main Localizer");
        }
    }
}
