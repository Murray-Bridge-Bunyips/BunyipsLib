package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.accumulators;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Periodic;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
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
    private final Periodic reloc;
    private final IMU imu;
    private Rotation2d origin;

    /**
     * Create a new PeriodicIMUAccumulator.
     *
     * @param imu                    the imu to use for updates
     * @param relocalizationInterval the interval at which the IMU is read and updates are propagated
     */
    public PeriodicIMUAccumulator(@Nullable IMU imu, @NonNull Measure<Time> relocalizationInterval) {
        this.imu = imu;
        origin = pose.heading;
        reloc = new Periodic(relocalizationInterval, () -> {
            if (imu != null)
                pose = new Pose2d(pose.position, origin.plus(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
        });
        if (imu != null)
            imu.resetYaw();
    }

    /**
     * Sets the angle origin to use. By default this is set to the IMU field on instantiation.
     *
     * @param origin the origin angle for accumulation
     */
    public void setOrigin(@NonNull Measure<Angle> origin) {
        this.origin = Rotation2d.exp(origin.in(Radians));
    }

    @Override
    public void accumulate(@NonNull Twist2dDual<com.acmerobotics.roadrunner.Time> twist) {
        super.accumulate(twist);
        reloc.run();
    }
}
