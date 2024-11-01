package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.Localizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.ThreeWheelLocalizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.TwoWheelLocalizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.RoadRunnerDrive;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.ContinuousTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups.DeadlineTaskGroup;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;

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
        if (localizer instanceof TwoWheelLocalizer) {
            TwoWheelLocalizer l = (TwoWheelLocalizer) localizer;
            if (l.params.perpXTicks == 0 && l.params.parYTicks == 0) {
                throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
            }
        } else if (localizer instanceof ThreeWheelLocalizer) {
            ThreeWheelLocalizer l = (ThreeWheelLocalizer) localizer;
            if (l.params.perpXTicks == 0 && l.params.par0YTicks == 0 && l.params.par1YTicks == 1) {
                throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
            }
        }

        Dashboard.USING_SYNCED_PACKETS = true;
        waitForStart();

        while (opModeIsActive()) {
            // TODO: pause and stop ability for this opmode
            Actions.runBlocking(
                    new DeadlineTaskGroup(
                            drive.makeTrajectory(new Pose2d(0, 0, 0))
                                    .lineToX(DISTANCE)
                                    .lineToX(0)
                                    .build(),
                            new ContinuousTask(drive::periodic),
                            new ContinuousTask(Dashboard::sendAndClearSyncedPackets)
                    )
            );
        }
    }
}