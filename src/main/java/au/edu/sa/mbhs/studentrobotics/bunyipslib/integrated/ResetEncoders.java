package au.edu.sa.mbhs.studentrobotics.bunyipslib.integrated;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.Field;
import java.util.Map;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Dbg;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.EmergencyStop;

/**
 * Reset OpMode to clear all motors of their last known positions.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public final class ResetEncoders extends LinearOpMode {
    @Override
    @SuppressWarnings("unchecked")
    public void runOpMode() {
        Map<String, DcMotor> map;
        try {
            Field mapping = HardwareMap.DeviceMapping.class.getDeclaredField("map");
            mapping.setAccessible(true);
            map = (Map<String, DcMotor>) mapping.get(hardwareMap.dcMotor);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new EmergencyStop("Failed to access HardwareMap fields!");
        }
        if (map != null) {
            for (Map.Entry<String, DcMotor> motor : map.entrySet()) {
                DcMotor m = motor.getValue();
                Dbg.log(getClass(), "Resetting encoder on %(%) from % ticks ...", motor.getKey(), m.getConnectionInfo(), m.getCurrentPosition());
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        terminateOpModeNow();
    }
}
