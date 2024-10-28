package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Objects;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.RoadRunnerDrive;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.TankDrive;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;

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
        MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            drive.setPower(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            drive instanceof TankDrive ? 0.0 : -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
            drive.periodic();
            telemetry.addData("Pose2d", Geometry.toUserString(Objects.requireNonNull(drive.getPose())).replace("Pose2d", ""));
            telemetry.addData("PoseVelocity2d", Objects.requireNonNull(drive.getVelocity()).toString().replace("PoseVelocity2d", ""));
            telemetry.update();
        }
    }
}