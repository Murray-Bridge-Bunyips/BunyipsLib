package org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.murraybridgebunyips.bunyipslib.external.Mathf;

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

    private double heading;
    private double headingVelo;
    private double lastHeading;

    /**
     * Create a new IMULocalizer.
     *
     * @param imu the imu to use
     */
    public IMULocalizer(IMU imu) {
        this.imu = imu;
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return new Pose2d(0, 0, heading);
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        heading = pose2d.getHeading();
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return new Pose2d(0, 0, headingVelo);
    }

    @Override
    public void update() {
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double delta = Mathf.inputModulus(currentHeading - lastHeading, -Math.PI, Math.PI);
        heading = Mathf.inputModulus(heading + delta, 0, 2 * Math.PI);

        headingVelo = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
        lastHeading = heading;
    }
}
