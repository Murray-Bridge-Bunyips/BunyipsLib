package org.murraybridgebunyips.bunyipslib.tasks;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Controller;
import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.pid.PIDController;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.tasks.bases.ForeverTask;
import org.murraybridgebunyips.bunyipslib.vision.data.AprilTagData;
import org.murraybridgebunyips.bunyipslib.vision.data.ContourData;
import org.murraybridgebunyips.bunyipslib.vision.processors.AprilTag;

import java.util.List;
import java.util.Optional;

/**
 * Task to move to and align to an AprilTag
 *
 * @param <T> the drivetrain to use (must implement RoadRunnerDrive for X pose forward info/FCD)
 * @author Lucas Bubner, 2024
 */
@Config
public class MoveToAprilTagTask<T extends BunyipsSubsystem> extends ForeverTask {
    public static double FORWARD_TRANSLATIONAL_kP;
    public static double FORWARD_TRANSLATIONAL_kI;
    public static double FORWARD_TRANSLATIONAL_kD;
    public static double STRAFE_TRANSLATIONAL_kP;
    public static double STRAFE_TRANSLATIONAL_kI;
    public static double STRAFE_TRANSLATIONAL_kD;
    public static double ROTATIONAL_kP;
    public static double ROTATIONAL_kI;
    public static double ROTATIONAL_kD;

    private final RoadRunnerDrive drive;
    private final AprilTag aprilTag;
    private final Gamepad gamepad;
    private final PIDController forwardTranslationController;
    private final PIDController strafeTranslationController;
    private final PIDController rotationController;

    public MoveToAprilTagTask(Gamepad gamepad, T drive, AprilTag aprilTag, PIDController forwardTranslationController, PIDController strafeTranslationController, PIDController rotationController) {
        super(drive, false);
        if (!(drive instanceof RoadRunnerDrive))
            throw new EmergencyStop("MoveToPixelTask must be used with a drivetrain with X forward Pose/IMU info");
        this.drive = (RoadRunnerDrive) drive;
        this.aprilTag = aprilTag;
        this.gamepad = gamepad;
        this.forwardTranslationController = forwardTranslationController;
        this.strafeTranslationController = strafeTranslationController;
        this.rotationController = rotationController;
        FORWARD_TRANSLATIONAL_kP = forwardTranslationController.getP();
        FORWARD_TRANSLATIONAL_kI = forwardTranslationController.getI();
        FORWARD_TRANSLATIONAL_kD = forwardTranslationController.getD();
        STRAFE_TRANSLATIONAL_kP = strafeTranslationController.getP();
        STRAFE_TRANSLATIONAL_kI = strafeTranslationController.getI();
        STRAFE_TRANSLATIONAL_kD = strafeTranslationController.getD();
        ROTATIONAL_kP = rotationController.getP();
        ROTATIONAL_kI = rotationController.getI();
        ROTATIONAL_kD = rotationController.getD();
    }

    @Override
    public void init() {
        if (!aprilTag.isAttached())
            throw new RuntimeException("Vision processor was initialised without being attached to the vision system");
    }

    @Override
    public void periodic() {
        // FtcDashboard live tuning
        forwardTranslationController.setPID(FORWARD_TRANSLATIONAL_kP, FORWARD_TRANSLATIONAL_kI, FORWARD_TRANSLATIONAL_kD);
        strafeTranslationController.setPID(STRAFE_TRANSLATIONAL_kP, STRAFE_TRANSLATIONAL_kI, STRAFE_TRANSLATIONAL_kD);
        rotationController.setPID(ROTATIONAL_kP, ROTATIONAL_kI, ROTATIONAL_kD);

        Pose2d pose = Controller.makeRobotPose(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.right_stick_x);

        List<AprilTagData> data = aprilTag.getData();

        Optional<AprilTagData> target = data.stream().filter(p -> p.getId() == 2).findFirst();
        if (!target.isPresent()) {
            drive.setWeightedDrivePower(pose);
            return;
        }

        // TODO: Optimise
        drive.setWeightedDrivePower(
                new Pose2d(
                        forwardTranslationController.calculate(target.get().getRange(), 1.0),
                        strafeTranslationController.calculate(target.get().getBearing(), 0.0),
                        -rotationController.calculate(target.get().getYaw(), 0.0)
                )
        );
    }

    @Override
    public void onFinish() {
//        drive.setSpeedUsingController(0, 0, 0);
    }
}
