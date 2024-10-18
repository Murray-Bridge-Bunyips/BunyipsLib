package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.Localizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.ThreeWheelLocalizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.TwoWheelLocalizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.RoadRunnerDrive;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.MecanumDrive;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.TankDrive;

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
        Localizer localizer;
        if (drive instanceof TankDrive) {
            localizer = ((TankDrive) drive).getLocalizer();
        } else if (drive instanceof MecanumDrive) {
            localizer = ((MecanumDrive) drive).getLocalizer();
        } else {
            throw new RuntimeException("Unknown drive type!");
        }
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

        waitForStart();

        while (opModeIsActive()) {
            // TODO: actions makePath not implemented yet
//            Actions.runBlocking(
//                    drive.actionBuilder(new Pose2d(0, 0, 0))
//                            .lineToX(DISTANCE)
//                            .lineToX(0)
//                            .build());
        }
    }
}