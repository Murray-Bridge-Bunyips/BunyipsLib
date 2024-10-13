package org.murraybridgebunyips.bunyipslib.localization;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Radians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.murraybridgebunyips.bunyipslib.util.Cartesian;
import org.murraybridgebunyips.bunyipslib.util.Geometry;
import org.murraybridgebunyips.bunyipslib.external.Mathf;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * SwitchableLocalizer is a composite localizer that allows self-tests to be performed, and to allow a "fallback"
 * localizer to kick in if the self-test fails.
 *
 * @author Lucas Bubner, 2024
 * @since 4.0.0
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

    @Override
    public Twist2dDual<Time> update() {
        if (USING_FALLBACK_LOCALIZER) {
            return fallback.update();
        }
        return main.update();
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
                private Pose2d pose = Geometry.zeroPose();
                private Telemetry.Item telemetry;
                private Vector2d capture;

                private boolean forwardCheck;
                private boolean strafeCheck;

                private void update(String text) {
                    telemetry.setValue("\nLocalizer Auto Test in progress.\n" +
                            "<font color='gray'>gamepad1.left_bumper to early abort and fail the test.\n" +
                            "gamepad1.right_bumper to force pass the test.</font>\n" + text + "\n" + pose + "\n");
                }

                @Override
                protected void init() {
                    telemetry = require(opMode).telemetry.addRetained("Initialising Localizer Self Test...").bold();
                    pose = pose.plus(main.update().value());
                    capture = Cartesian.rotate(Cartesian.fromPose(pose).position, Radians.of(pose.heading.toDouble()).negate());
                }

                @Override
                protected void periodic() {
                    pose = pose.plus(main.update().value());
                    if (require(opMode).gamepad1.left_bumper) {
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
                    // Always calculate error forward, using cartesian coordinates
                    Vector2d error = capture.minus(Cartesian.rotate(Cartesian.fromPose(pose).position, Radians.of(pose.heading.toDouble()).negate()));
                    if (!forwardCheck) {
                        update("<font color='red'>Please move the robot forward a minimum of 10 inches.</font>");
                        if (!Mathf.isNear(error.y, capture.y, 10)) {
                            forwardCheck = true;
                        }
                        return;
                    }
                    if (!strafeCheck) {
                        update("<font color='yellow'>Please move the robot sideways a minimum of 10 inches.</font>");
                        if (!Mathf.isNear(error.x, capture.x, 10)) {
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
                        require(opMode).telemetry.log("<font color='yellow'>Localizer test failed. Falling back to backup localizer.</font>");
                        USING_FALLBACK_LOCALIZER = true;
                        return;
                    }
                    USING_FALLBACK_LOCALIZER = false;
                    require(opMode).telemetry.log("<font color='green'>Localizer test passed.</font>");
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
                private Pose2d pose = Geometry.zeroPose();
                private Telemetry.Item telemetry;
                private Vector2d capture;

                @Override
                protected void init() {
                    telemetry = require(opMode).telemetry.addRetained("Initialising Localizer Test...").bold();
                    pose = pose.plus(main.update().value());
                    capture = Cartesian.rotate(Cartesian.fromPose(pose).position, Radians.of(pose.heading.toDouble()).negate());
                }

                @Override
                protected void periodic() {
                    pose = pose.plus(main.update().value());
                    if (require(opMode).gamepad1.left_bumper) {
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
                    telemetry.setValue("\nLocalizer Test in progress.\n" +
                            "<font color='gray'>gamepad1.left_bumper to fail the test.\n" +
                            "gamepad1.right_bumper to pass the test. Pass the test if the below values are increasing properly.</font>\n" +
                            "Raw: " + pose + "\n" +
                            "Error (rotated): " + capture.minus(Cartesian.rotate(Cartesian.fromPose(pose).position, Radians.of(pose.heading.toDouble()).negate())) + "\n");
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
