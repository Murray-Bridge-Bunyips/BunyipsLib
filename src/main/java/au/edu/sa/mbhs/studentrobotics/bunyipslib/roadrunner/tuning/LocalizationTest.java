package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Objects;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.RoadRunnerDrive;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.TankDrive;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;

/**
 * Internal RoadRunner LocalizationTest tuning OpMode.
 *
 * @since 6.0.0
 */
public final class LocalizationTest extends LinearOpMode {
    private final RoadRunnerDrive drive;

    LocalizationTest(RoadRunnerDrive drive) {
        this.drive = drive;
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            drive.setPower(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            drive instanceof TankDrive ? 0.0 : -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            Pose2d poseEstimate = Objects.requireNonNull(drive.getPoseEstimate(), "Non-null localizer required to localize.");

            telemetry.addData("x", poseEstimate.position.x);
            telemetry.addData("y", poseEstimate.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(poseEstimate.heading.toDouble()));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Dashboard.drawRobot(packet.fieldOverlay(), poseEstimate);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}