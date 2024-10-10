package org.murraybridgebunyips.bunyipslib.drive;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.murraybridgebunyips.bunyipslib.roadrunner.drive.DriveConstants;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.MecanumCoefficients;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers.ThreeWheelLocalizer;
import org.murraybridgebunyips.bunyipslib.roadrunner.util.Deadwheel;

import java.util.List;

/**
 * {@link MecanumDrive} with three tracking dead wheels for localization.
 * This is a convenience class to update the localizer on construction. It functions exactly as a {@link MecanumDrive}.
 *
 * @author Lucas Bubner, 2023
 * @see MecanumDrive
 * @since 1.0.0-pre
 */
public class TriDeadwheelMecanumDrive extends MecanumDrive {
    /**
     * Create a new TriDeadwheelMecanumDrive.
     *
     * @param constants                The drive constants
     * @param mecanumCoefficients      The mecanum coefficients
     * @param imu                      The IMU
     * @param frontLeft                The front left motor
     * @param frontRight               The front right motor
     * @param backLeft                 The back left motor
     * @param backRight                The back right motor
     * @param localizerCoefficients    The three deadwheel localizer coefficients
     * @param enc_left                 The left y encoder
     * @param enc_right                The right y encoder
     * @param enc_x                    The x encoder
     * @param lastTrackingEncPositions The last tracking encoder positions
     * @param lastTrackingEncVels      The last tracking encoder velocities
     */
    public TriDeadwheelMecanumDrive(DriveConstants constants, MecanumCoefficients mecanumCoefficients, @Nullable IMU imu, DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, ThreeWheelLocalizer.Coefficients localizerCoefficients, Deadwheel enc_left, Deadwheel enc_right, Deadwheel enc_x, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(constants, mecanumCoefficients, imu, frontLeft, frontRight, backLeft, backRight);
        if (!assertParamsNotNull(localizerCoefficients, enc_left, enc_right, enc_x, lastTrackingEncPositions, lastTrackingEncVels))
            return;
        setLocalizer(new ThreeWheelLocalizer(localizerCoefficients, enc_left, enc_right, enc_x, lastTrackingEncPositions, lastTrackingEncVels).withRelocalizingIMU(imu));
        updatePoseFromMemory();
    }

    /**
     * Create a new TriDeadwheelMecanumDrive.
     *
     * @param constants             The drive constants
     * @param mecanumCoefficients   The mecanum coefficients
     * @param imu                   The IMU.
     * @param frontLeft             The front left motor
     * @param frontRight            The front right motor
     * @param backLeft              The back left motor
     * @param backRight             The back right motor
     * @param localizerCoefficients The three deadwheel localizer coefficients
     * @param enc_left              The left y encoder
     * @param enc_right             The right y encoder
     * @param enc_x                 The x encoder
     */
    public TriDeadwheelMecanumDrive(DriveConstants constants, MecanumCoefficients mecanumCoefficients, @Nullable IMU imu, DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, ThreeWheelLocalizer.Coefficients localizerCoefficients, Deadwheel enc_left, Deadwheel enc_right, Deadwheel enc_x) {
        super(constants, mecanumCoefficients, imu, frontLeft, frontRight, backLeft, backRight);
        if (!assertParamsNotNull(localizerCoefficients, enc_left, enc_right, enc_x)) return;
        setLocalizer(new ThreeWheelLocalizer(localizerCoefficients, enc_left, enc_right, enc_x).withRelocalizingIMU(imu));
        updatePoseFromMemory();
    }

    /**
     * Enable overflow compensation if your encoders exceed 32767 counts / second.
     *
     * @return this
     * @deprecated overflow compensation is available as an option in the ThreeWheelLocalizer coefficients
     */
    @Deprecated
    public TriDeadwheelMecanumDrive enableOverflowCompensation() {
        ThreeWheelLocalizer localizer = (ThreeWheelLocalizer) getLocalizer();
        if (localizer != null)
            localizer.enableOverflowCompensation();
        return this;
    }
}
