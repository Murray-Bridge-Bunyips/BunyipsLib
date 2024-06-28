package org.murraybridgebunyips.bunyipslib.integrated;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;

/**
 * Reset OpMode to clear any BunyipsOpMode-set robot controller lights.
 *
 * @author Lucas Bubner, 2024
 */
@TeleOp(name = "Reset Robot Controller Lights", group="BunyipsLib")
public final class ResetRobotControllerLights extends BunyipsOpMode {
    @Override
    protected void onInit() {
        getRobotControllers().forEach((c) ->
            c.setPattern(LynxModule.blinkerPolicy.getIdlePattern(c)));
        exit();
    }

    @Override
    protected void activeLoop() {
        // no-op
    }
}
