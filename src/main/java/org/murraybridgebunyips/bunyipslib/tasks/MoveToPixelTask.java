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
import org.murraybridgebunyips.bunyipslib.vision.data.ContourData;
import org.murraybridgebunyips.bunyipslib.vision.processors.MultiYCbCrThreshold;

import java.util.List;

/**
 * Task to move to and align to a pixel using the vision system.
 *
 * @param <T> the drivetrain to use (must implement RoadRunnerDrive for X pose forward info/FCD)
 * @author Lucas Bubner, 2024
 */
@Config
public class MoveToPixelTask<T extends BunyipsSubsystem> extends ForeverTask {
    public static double TRANSLATIONAL_kP;
    public static double TRANSLATIONAL_kI;
    public static double TRANSLATIONAL_kD;
    public static double ROTATIONAL_kP;
    public static double ROTATIONAL_kI;
    public static double ROTATIONAL_kD;

    private final RoadRunnerDrive drive;
    private final MultiYCbCrThreshold processors;
    private final Gamepad gamepad;
    private final PIDController translationController;
    private final PIDController rotationController;
    private final TrackingParameters trackingParameters;

    public static class TrackingParameters {
        /**
         * The height of the camera from the ground in metres
         */
        public final double CAMERA_HEIGHT_METRES;
        /**
         * The height of the target from the ground in metres
         */
        public final double TARGET_HEIGHT_METRES;
        /**
         * The angle of the camera from the ground in degrees
         */
        public final double CAMERA_PITCH_DEGREES;

        /**
         * The vertical field of view of the camera in degrees
         */
        public final double CAMERA_VERTICAL_FOV_DEGREES;

        /**
         * How far the robot should be from the target in metres
         */
        public final double GOAL_TARGET_METRES;

        public TrackingParameters(double CAMERA_HEIGHT_METRES, double TARGET_HEIGHT_METRES, double CAMERA_PITCH_DEGREES, double CAMERA_VERTICAL_FOV_DEGREES, double GOAL_TARGET_METRES) {
            this.CAMERA_HEIGHT_METRES = CAMERA_HEIGHT_METRES;
            this.TARGET_HEIGHT_METRES = TARGET_HEIGHT_METRES;
            this.CAMERA_PITCH_DEGREES = CAMERA_PITCH_DEGREES;
            this.CAMERA_VERTICAL_FOV_DEGREES = CAMERA_VERTICAL_FOV_DEGREES;
            this.GOAL_TARGET_METRES = GOAL_TARGET_METRES;
        }
    }

    public MoveToPixelTask(Gamepad gamepad, T drive, MultiYCbCrThreshold processors, PIDController translationController, PIDController rotationController, TrackingParameters trackingParameters) {
        super(drive, false);
        if (!(drive instanceof RoadRunnerDrive))
            throw new EmergencyStop("MoveToPixelTask must be used with a drivetrain with X forward Pose/IMU info");
        this.drive = (RoadRunnerDrive) drive;
        this.processors = processors;
        this.gamepad = gamepad;
        this.translationController = translationController;
        this.rotationController = rotationController;
        TRANSLATIONAL_kP = translationController.getP();
        TRANSLATIONAL_kI = translationController.getI();
        TRANSLATIONAL_kD = translationController.getD();
        ROTATIONAL_kP = rotationController.getP();
        ROTATIONAL_kI = rotationController.getI();
        ROTATIONAL_kD = rotationController.getD();
        this.trackingParameters = trackingParameters;
    }

    @Override
    public void init() {
        if (!processors.isAttached())
            throw new RuntimeException("Vision processor was initialised without being attached to the vision system");
    }

    @Override
    public void periodic() {
        // FtcDashboard live tuning
        translationController.setPID(TRANSLATIONAL_kP, TRANSLATIONAL_kI, TRANSLATIONAL_kD);
        rotationController.setPID(ROTATIONAL_kP, ROTATIONAL_kI, ROTATIONAL_kD);

        Pose2d pose = Controller.makeRobotPose(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.right_stick_x);

        List<ContourData> data = processors.getData();
        ContourData biggestContour = ContourData.getLargest(data);

        if (biggestContour != null) {
            double distance = getDistance(biggestContour.getPitch());
            drive.setWeightedDrivePower(
                    new Pose2d(
                            translationController.calculate(distance, trackingParameters.GOAL_TARGET_METRES),
                            pose.getY(),
                            rotationController.calculate(biggestContour.getYaw(), 0.0)
                    )
            );
        } else {
            drive.setWeightedDrivePower(pose);
        }
    }

    private double getDistance(double pitch) {
        // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
        // tan(pitch) = (h2 - h1) / d
        // d = (h2 - h1) / tan(pitch)
        // pitch = target pitch * angle of view + camera pitch
        return (trackingParameters.TARGET_HEIGHT_METRES - trackingParameters.CAMERA_HEIGHT_METRES)
                / Math.tan(Math.toRadians((pitch * trackingParameters.CAMERA_VERTICAL_FOV_DEGREES) + trackingParameters.CAMERA_PITCH_DEGREES));
    }

    @Override
    public void onFinish() {
//        drive.setSpeedUsingController(0, 0, 0);
    }
}
