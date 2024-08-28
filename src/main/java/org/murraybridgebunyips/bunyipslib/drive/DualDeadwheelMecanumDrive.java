package org.murraybridgebunyips.bunyipslib.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.murraybridgebunyips.bunyipslib.roadrunner.drive.DriveConstants;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.MecanumCoefficients;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers.TwoWheelLocalizer;
import org.murraybridgebunyips.bunyipslib.roadrunner.util.Deadwheel;

/**
 * {@link MecanumDrive} with two tracking dead wheels for localization.
 * This is a convenience class to update the localizer on construction. It functions exactly as a {@link MecanumDrive}.
 *
 * @author Lucas Bubner, 2023
 * @see MecanumDrive
 * @since 1.0.0-pre
 */
public class DualDeadwheelMecanumDrive extends MecanumDrive {
    /**
     * Constructs a new DualDeadwheelMecanumDrive.
     *
     * @param constants             the drive constants
     * @param mecanumCoefficients   the mecanum coefficients
     * @param imu                   the IMU to use
     * @param frontLeft             the front left motor
     * @param frontRight            the front right motor
     * @param backLeft              the back left motor
     * @param backRight             the back right motor
     * @param localizerCoefficients the deadwheel localizer coefficients
     * @param parallel              the parallel deadwheel encoder
     * @param perpendicular         the perpendicular deadwheel encoder
     */
    public DualDeadwheelMecanumDrive(DriveConstants constants, MecanumCoefficients mecanumCoefficients, IMU imu, DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, TwoWheelLocalizer.Coefficients localizerCoefficients, Deadwheel parallel, Deadwheel perpendicular) {
        super(constants, mecanumCoefficients, imu, frontLeft, frontRight, backLeft, backRight);
        if (!assertParamsNotNull(localizerCoefficients, parallel, perpendicular)) return;
        setLocalizer(new TwoWheelLocalizer(localizerCoefficients, parallel, perpendicular, this));
        updatePoseFromMemory();
    }

    /**
     * Enable overflow compensation if your encoders exceed 32767 counts / second.
     *
     * @return this
     */
    public DualDeadwheelMecanumDrive enableOverflowCompensation() {
        TwoWheelLocalizer localizer = (TwoWheelLocalizer) getLocalizer();
        if (localizer != null)
            localizer.enableOverflowCompensation();
        return this;
    }
}
