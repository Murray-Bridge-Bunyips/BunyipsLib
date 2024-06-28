package org.murraybridgebunyips.bunyipslib.integrated;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Reset OpMode to clear any BunyipsOpMode-set robot controller lights.
 *
 * @author Lucas Bubner, 2024
 */
@TeleOp(name = "Reset Robot Controller Lights", group="BunyipsLib")
public final class ResetRobotControllerLights extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap.getAll(LynxModule.class).forEach((c) ->
            c.setPattern(LynxModule.blinkerPolicy.getIdlePattern(c)));
        // TODO: check if this will complain about a stop before waitForStart
        requestOpModeStop();
    }
}
