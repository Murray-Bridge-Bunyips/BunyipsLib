package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsLib;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.DualTelemetry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text;

/**
 * SwitchableLocalizer is a composite localizer that allows self-tests to be performed, and to allow a "fallback"
 * localizer to kick in if the self-test fails.
 *
 * @author Lucas Bubner, 2024
 * @since 4.0.0
 */
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
    @NonNull
    public Tasks tasks = new Tasks();

    /**
     * Create a new SwitchableLocalizer.
     *
     * @param main     the main localizer to use
     * @param fallback the fallback localizer to use, activated if the self-test task fails, or by the user.
     */
    public SwitchableLocalizer(@NonNull Localizer main, @NonNull Localizer fallback) {
        USING_FALLBACK_LOCALIZER = false;
        this.main = main;
        this.fallback = fallback;
        Dashboard.enableConfig(getClass());
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
        @NonNull
        public Task autoTestMainLocalizer() {
            return new Task() {
                private final OpMode opMode = BunyipsLib.getOpMode();
                private double SELF_TEST_VELOCITY_INCHES = 7;

                private Telemetry.Item telemetry;
                private double minX, minY;
                private double maxX, maxY;

                private boolean forwardCheck, backwardCheck;
                private boolean leftCheck, rightCheck;

                /**
                 * Set the velocity in inches for the self-test to pass.
                 *
                 * @param velocity velocity magnitude to pass the test for each translation axis
                 */
                public void setSelfTestVelocityInches(double velocity) {
                    SELF_TEST_VELOCITY_INCHES = velocity;
                }

                @Override
                protected void init() {
                    telemetry = DualTelemetry.smartAdd(true, "Initialising Localizer Self Test...");
                    if (telemetry instanceof DualTelemetry.HtmlItem item)
                        item.bold();
                }

                @Override
                protected void periodic() {
                    PoseVelocity2d vel = main.update().velocity().value();
                    minX = Math.min(minX, vel.linearVel.x);
                    minY = Math.min(minY, vel.linearVel.y);
                    maxX = Math.max(maxX, vel.linearVel.x);
                    maxY = Math.max(maxY, vel.linearVel.y);
                    if (opMode.gamepad1.left_bumper) {
                        forwardCheck = false;
                        backwardCheck = false;
                        leftCheck = false;
                        rightCheck = false;
                        finish();
                        return;
                    }
                    if (opMode.gamepad1.right_bumper) {
                        forwardCheck = true;
                        backwardCheck = true;
                        leftCheck = true;
                        rightCheck = true;
                        finish();
                        return;
                    }

                    forwardCheck = maxX > SELF_TEST_VELOCITY_INCHES;
                    backwardCheck = minX < -SELF_TEST_VELOCITY_INCHES;
                    leftCheck = maxY > SELF_TEST_VELOCITY_INCHES;
                    rightCheck = minY < -SELF_TEST_VELOCITY_INCHES;

                    telemetry.setValue(Text.builder()
                            .append("\nLocalizer Auto Test in progress.\n")
                            .append("<font color='gray'>gamepad1.left_bumper to early abort and fail the test.\n")
                            .append("gamepad1.right_bumper to force pass the test.</font>\n")
                            .append("<font color='%'>[%] Forward Check</font>\n", forwardCheck ? "green" : "red", forwardCheck ? "X" : " ")
                            .append("<font color='%'>[%] Backward Check</font>\n", backwardCheck ? "green" : "red", backwardCheck ? "X" : " ")
                            .append("<font color='%'>[%] Left Check</font>\n", leftCheck ? "green" : "red", leftCheck ? "X" : " ")
                            .append("<font color='%'>[%] Right Check</font>\n", rightCheck ? "green" : "red", rightCheck ? "X" : " ")
                            .append("Min Velocities: ")
                            .append("x:%,y:%\n", Mathf.round(minX, 1), Mathf.round(minY, 1))
                            .append("Max Velocities: ")
                            .append("x:%,y:%\n", Mathf.round(maxX, 1), Mathf.round(maxY, 1))
                            .toString());
                }

                @Override
                protected void onFinish() {
                    telemetry.setRetained(false);
                    if (!forwardCheck || !backwardCheck || !leftCheck || !rightCheck) {
                        DualTelemetry.smartLog("<font color='yellow'>Localizer test failed. Falling back to backup localizer.</font>");
                        USING_FALLBACK_LOCALIZER = true;
                        return;
                    }
                    USING_FALLBACK_LOCALIZER = false;
                    DualTelemetry.smartLog("<font color='green'>Localizer test passed.</font>");
                }

                @Override
                protected boolean isTaskFinished() {
                    // Manual exit or if all checks are complete
                    return forwardCheck && backwardCheck && leftCheck && rightCheck;
                }
            }.named("Auto Test Main Localizer");
        }

        /**
         * Tests the main localizer and falls back to the fallback localizer if the user wishes to fail the test.
         *
         * @return a task to test the localizer manually using human judgement, will propagate result on user input
         */
        @NonNull
        public Task manualTestMainLocalizer() {
            return new Task() {
                private final OpMode opMode = BunyipsLib.getOpMode();
                private Telemetry.Item telemetry;
                private double minX, minY, minAng;
                private double maxX, maxY, maxAng;

                @Override
                protected void init() {
                    telemetry = DualTelemetry.smartAdd(true, "Initialising Localizer Self Test...");
                    if (telemetry instanceof DualTelemetry.HtmlItem item)
                        item.bold();
                }

                @Override
                protected void periodic() {
                    PoseVelocity2d vel = main.update().velocity().value();
                    minX = Math.min(minX, vel.linearVel.x);
                    minY = Math.min(minY, vel.linearVel.y);
                    minAng = Math.min(minAng, vel.angVel);
                    maxX = Math.max(maxX, vel.linearVel.x);
                    maxY = Math.max(maxY, vel.linearVel.y);
                    maxAng = Math.max(maxAng, vel.angVel);
                    if (opMode.gamepad1.left_bumper) {
                        USING_FALLBACK_LOCALIZER = true;
                        DualTelemetry.smartLog("<font color='yellow'>Localizer test failed. Falling back to backup localizer.</font>");
                        telemetry.setValue("");
                        finish();
                        return;
                    }
                    if (opMode.gamepad1.right_bumper) {
                        USING_FALLBACK_LOCALIZER = false;
                        DualTelemetry.smartLog("<font color='green'>Localizer test passed.</font>");
                        telemetry.setValue("");
                        finish();
                        return;
                    }
                    telemetry.setValue(Text.builder()
                            .append("\nLocalizer Test in progress.\n")
                            .append("<font color='gray'>gamepad1.left_bumper to fail the test.\n")
                            .append("gamepad1.right_bumper to pass the test. Pass the test if the below minimum and maximum velocities are updating.</font>\n")
                            .append("Min Velocities: ")
                            .append("x:%,y:%,r:%\n", Mathf.round(minX, 1), Mathf.round(minY, 1), Mathf.round(minAng, 1))
                            .append("Max Velocities: ")
                            .append("x:%,y:%,r:%\n", Mathf.round(maxX, 1), Mathf.round(maxY, 1), Mathf.round(maxAng, 1))
                            .toString());
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
            }.named("Manual Test Main Localizer");
        }
    }
}
