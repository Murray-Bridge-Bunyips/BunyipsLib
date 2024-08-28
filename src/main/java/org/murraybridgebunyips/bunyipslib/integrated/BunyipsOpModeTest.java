package org.murraybridgebunyips.bunyipslib.integrated;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;

/**
 * Universal OpMode to test BunyipsOpMode functionality
 *
 * @since 1.0.0-pre
 */
@TeleOp(name = "BunyipsOpMode", group = "BunyipsLib")
@Disabled
public final class BunyipsOpModeTest extends BunyipsOpMode {
    @Override
    protected void onInit() {
        telemetry.addRetained("======= BunyipsOpMode =======");
    }

    @Override
    protected void activeLoop() {
        telemetry.add(getTimer().toString());
    }
}
