package au.edu.sa.mbhs.studentrobotics.bunyipslib.integrated;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Dbg;

/**
 * Reset OpMode to clear all motors of their last known positions.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public final class ResetEncoders extends LinearOpMode {
    @Override
    public void runOpMode() {
        for (DcMotor motor : hardwareMap.getAll(DcMotor.class)) {
            Dbg.log(getClass(), "Resetting encoder on % from % ticks ...", motor.getConnectionInfo(), motor.getCurrentPosition());
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        terminateOpModeNow();
    }
}
