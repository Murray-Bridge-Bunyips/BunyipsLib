package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.tuning;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.RoadRunnerDrive;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.ContinuousTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups.DeadlineTaskGroup;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;

/**
 * Internal RoadRunner SplineTest tuning OpMode.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public final class SplineTest extends LinearOpMode {
    private final RoadRunnerDrive drive;

    SplineTest(RoadRunnerDrive drive) {
        this.drive = drive;
    }

    @Override
    public void runOpMode() {
        drive.setPose(Geometry.zeroPose());

        Dashboard.USING_SYNCED_PACKETS = true;
        waitForStart();

        Actions.runBlocking(
                new DeadlineTaskGroup(
                        drive.makeTrajectory()
                                .splineTo(new Vector2d(30, 30), Math.PI / 2)
                                .splineTo(new Vector2d(0, 60), Math.PI)
                                .build(),
                        new ContinuousTask(drive::periodic),
                        new ContinuousTask(Dashboard::sendAndClearSyncedPackets)
                )
        );
    }
}