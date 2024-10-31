package au.edu.sa.mbhs.studentrobotics.bunyipslib.integrated;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Reset OpMode to clear any BunyipsOpMode-set robot controller lights.
 *
 * @author Lucas Bubner, 2024
 * @since 3.4.0
 */
// Using same group as FtcDashbord to fit with other constantly enabled OpModes
@TeleOp(name = "Reset Robot Controller Lights", group = "dash")
public final class ResetRobotControllerLights extends LinearOpMode {
    @Override
    public void runOpMode() {
        hardwareMap.getAll(LynxModule.class).forEach((c) ->
                c.setPattern(LynxModule.blinkerPolicy.getIdlePattern(c)));
        terminateOpModeNow();
    }
}
