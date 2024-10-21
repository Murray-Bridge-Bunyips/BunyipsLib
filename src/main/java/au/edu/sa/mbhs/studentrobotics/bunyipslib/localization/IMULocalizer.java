package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;

/**
 * A simple wrapper that will wrap an Inertial Measurement Unit to supply heading-only poses as a {@link Localizer}.
 * This is useful for localizer implementations that do not rely on encoders, and instead only
 * require the heading information as given by a gyroscope.
 *
 * @author Lucas Bubner, 2024
 * @since 5.1.0
 */
public class IMULocalizer implements Localizer {
    private final IMU imu;
    private Rotation2d lastHeading;
    private boolean init;

    /**
     * Create a new IMULocalizer.
     *
     * @param imu the imu to use
     */
    public IMULocalizer(@NonNull IMU imu) {
        this.imu = imu;
    }

    @NonNull
    @Override
    public Twist2dDual<Time> update() {
        Rotation2d currentHeading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        if (!init) {
            init = true;
            lastHeading = currentHeading;
            return new Twist2dDual<>(
                    Vector2dDual.constant(Geometry.zeroVec(), 2),
                    DualNum.constant(0, 2)
            );
        }
        double headingVelo = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
        double headingDelta = currentHeading.minus(lastHeading);
        lastHeading = currentHeading;
        return new Twist2dDual<>(
                Vector2dDual.constant(Geometry.zeroVec(), 2),
                new DualNum<>(new double[]{headingDelta, headingVelo})
        );
    }
}
