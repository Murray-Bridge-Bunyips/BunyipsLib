package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.Localizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.ThreeWheelLocalizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.TwoWheelLocalizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.RoadRunnerDrive;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.ConditionalTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups.DeadlineTaskGroup;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;

/**
 * Internal RoadRunner ManualFeedbackTuner tuning OpMode.
 *
 * @since 6.0.0
 */
public final class ManualFeedbackTuner extends LinearOpMode {
    /**
     * Distance in inches to travel.
     */
    public static double DISTANCE = 64;

    private final RoadRunnerDrive drive;

    ManualFeedbackTuner(RoadRunnerDrive drive) {
        this.drive = drive;
    }

    @Override
    public void runOpMode() {
        Localizer localizer = drive.getLocalizer();
        if (localizer instanceof TwoWheelLocalizer l) {
            if (l.params.perpXTicks == 0 && l.params.parYTicks == 0) {
                throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
            }
        } else if (localizer instanceof ThreeWheelLocalizer l) {
            if (l.params.perpXTicks == 0 && l.params.par0YTicks == 0 && l.params.par1YTicks == 1) {
                throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
            }
        }

        Dashboard.USING_SYNCED_PACKETS = true;
        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(
                    new DeadlineTaskGroup(
                            new ConditionalTask(
                                    Task.task().periodic(() -> drive.setPower(Controls.vel(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)))
                                            .until(() -> !gamepad1.right_bumper)
                                            .then(() -> drive.setPose(Geometry.zeroPose())),
                                    drive.makeTrajectory(new Pose2d(0, 0, 0))
                                            .lineToX(DISTANCE)
                                            .lineToX(0)
                                            .build()
                                            .until(() -> gamepad1.right_bumper),
                                    () -> gamepad1.right_bumper
                            ),
                            Task.task().periodic(() -> {
                                if (drive instanceof BunyipsSubsystem)
                                    ((BunyipsSubsystem) drive).update();
                                else
                                    drive.periodic();
                            }),
                            Task.task().periodic(Dashboard::sendAndClearSyncedPackets)
                    )
            );
        }
    }
}