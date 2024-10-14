package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliseconds;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Objects;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsOpMode;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Dbg;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;

/**
 * Runnable component to periodically update the pose estimate heading to the reading of the IMU.
 * Useful in three-wheel odometry where periodic heading resets are desired.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public class IMURelocalization implements Runnable {
    private final ElapsedTime timer = new ElapsedTime();
    private final double relocalizationIntervalMs;
    private final Localizable localizable;
    private final IMU imu;

    /**
     * Create a new IMURelocalization runner.
     *
     * @param localizable            the drive instance or localizer storer to update
     * @param imu                    the imu to use for updates
     * @param relocalizationInterval the interval at which the IMU is read and updates are propagated
     */
    public IMURelocalization(Localizable localizable, IMU imu, Measure<Time> relocalizationInterval) {
        this.localizable = localizable;
        this.imu = imu;
        relocalizationIntervalMs = relocalizationInterval.in(Milliseconds);
        BunyipsOpMode.ifRunning(opMode -> {
            opMode.onActiveLoop(this);
            Dbg.logd(getClass(), "Update executor has been auto-attached to BunyipsOpMode.");
        });
    }

    /**
     * Create a new IMURelocalization runner.
     *
     * @param localizable            the drive instance or localizer storer to update
     * @param imu                    the imu to use for updates
     * @param relocalizationInterval the interval at which the IMU is read and updates are propagated
     * @return a new instance of IMURelocalization
     */
    public static IMURelocalization enable(Localizable localizable, IMU imu, Measure<Time> relocalizationInterval) {
        return new IMURelocalization(localizable, imu, relocalizationInterval);
    }

    @Override
    public void run() {
        if (timer.milliseconds() >= relocalizationIntervalMs) {
            Pose2d current = Objects.requireNonNull(localizable.getPoseEstimate(), "A localizer must be attached to the drive instance to use IMURelocalization!");
            localizable.setPoseEstimate(new Pose2d(
                    current.position, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
            ));
            timer.reset();
        }
    }
}
