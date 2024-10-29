package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.accumulators;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliseconds;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;

/**
 * Accumulator that periodically updates the heading field of the pose estimate to the reading of an IMU.
 * Useful for three-wheel odometry, reducing I2C calls while maintaining accuracy.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public class PeriodicIMUAccumulator extends Accumulator {
    private final ElapsedTime timer = new ElapsedTime();
    private final double relocalizationIntervalMs;
    private final IMU imu;
    private final Rotation2d origin;

    /**
     * Create a new PeriodicIMUAccumulator.
     *
     * @param imu                    the imu to use for updates
     * @param relocalizationInterval the interval at which the IMU is read and updates are propagated
     */
    public PeriodicIMUAccumulator(@NonNull IMU imu, @NonNull Measure<Time> relocalizationInterval) {
        this.imu = imu;
        origin = pose.heading;
        relocalizationIntervalMs = relocalizationInterval.in(Milliseconds);
        imu.resetYaw();
    }

    @Override
    public void accumulate(@NonNull Twist2dDual<com.acmerobotics.roadrunner.Time> twist) {
        super.accumulate(twist);

        if (timer.milliseconds() >= relocalizationIntervalMs) {
            pose = new Pose2d(pose.position, origin.plus(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
            timer.reset();
        }
    }
}
