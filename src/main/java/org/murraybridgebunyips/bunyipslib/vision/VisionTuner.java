package org.murraybridgebunyips.bunyipslib.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.vision.Vision;
import org.murraybridgebunyips.bunyipslib.vision.processors.centerstage.WhitePixel;

public class VisionTuner extends BunyipsOpMode {
    int thresholdIndex = 1;

    @Override
    protected void onInit() {
        try {
            WebcamName webcam = (WebcamName) hardwareMap.get("webcam");
            Vision vision = new Vision(webcam);
            WhitePixel whitePixel = new WhitePixel();
        } catch (IllegalArgumentException e) {
            throw new EmergencyStop("VisionTest is missing a webcam called 'webcam'!");
        }
    }

    @Override
    protected void activeLoop() {
        // FIXME: This code is crap code, hopefully temp code.
        //  It's hard to expand upon, and could be optimised

        // TODO:
        //  Make it so it only changes when RB is held. If RB is released, it keeps the value as is.
        //  The value and the variable it was currently writing to should be shown in telemetry.

        float theEquation = (gamepad1.left_stick_y + gamepad1.left_stick_x) * 2;

        if (gamepad1.dpad_right) {
            if (thresholdIndex > 6)
                thresholdIndex = 0;
            else {
                thresholdIndex++;
            }
        }

        if (gamepad1.dpad_left) {
            if (thresholdIndex < 1)
                thresholdIndex = 5;
            else {
                thresholdIndex--;
            }

        switch (thresholdIndex) {
            case 1:
                WhitePixel.LOWER_Y = theEquation;
                break;
            case 2:
                WhitePixel.LOWER_CB = theEquation;
                break;
            case 3:
                WhitePixel.LOWER_CR = theEquation;
                break;
            case 4:
                WhitePixel.UPPER_Y = theEquation;
                break;
            case 5:
                WhitePixel.UPPER_CB = theEquation;
                break;
            case 6:
                WhitePixel.UPPER_CR = theEquation;
                break;
        }

        }
    }
}
