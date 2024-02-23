package org.murraybridgebunyips.bunyipslib.vision;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.vision.processors.ColourThreshold;
import org.murraybridgebunyips.bunyipslib.vision.processors.centerstage.GreenPixel;
import org.murraybridgebunyips.bunyipslib.vision.processors.centerstage.PurplePixel;
import org.murraybridgebunyips.bunyipslib.vision.processors.centerstage.WhitePixel;
import org.murraybridgebunyips.bunyipslib.vision.processors.centerstage.YellowPixel;

@TeleOp(name = "Vision Tuner")
public class VisionTuner extends BunyipsOpMode {
    int thresholdIndex = 1;
    int pixelIndex = 1;
    int theEquation = 0;

    ColourThreshold whitePixel;
    ColourThreshold purplePixel;
    ColourThreshold yellowPixel;
    ColourThreshold greenPixel;
    ColourThreshold[] pixels;
    ColourThreshold currentPixel;

    private boolean upPressed;
    private boolean downPressed;
    private boolean leftPressed;
    private boolean rightPressed;

    @Override
    protected void onInit() {
        try {
            WebcamName webcam = (WebcamName) hardwareMap.get("webcam");
            Vision vision = new Vision(webcam);
            whitePixel = new WhitePixel();
            purplePixel = new PurplePixel();
            yellowPixel = new YellowPixel();
            greenPixel = new GreenPixel();

            pixels = new ColourThreshold[]{whitePixel, purplePixel, yellowPixel, greenPixel};
            currentPixel = whitePixel;  // Set it to white pixel by default
        } catch (IllegalArgumentException e) {
            throw new EmergencyStop("VisionTest is missing a webcam called 'webcam'!");
        }
    }

    @Override
    protected void activeLoop() {
        // FIXME: This code is crap code, hopefully temp code.
        //  It's hard to expand upon, and could be optimised

        // TODO:
        //  The value and the variable it was currently writing to should be shown in telemetry.

        if (gamepad1.right_bumper) {
            float theEquation = (gamepad1.left_stick_y + gamepad1.left_stick_x) * 2;
        }

        if (gamepad1.dpad_up && !upPressed && !downPressed) {
            if (pixelIndex == 1) {
                pixelIndex = 4;
            } else {
                pixelIndex--;
            }
        }

        if (gamepad1.dpad_down && !upPressed && !downPressed) {
            if (pixelIndex == 4) {
                pixelIndex = 1;
            } else {
                pixelIndex++;
            }
        }

        if (gamepad1.dpad_left && !leftPressed && !rightPressed) {
            if (thresholdIndex == 1)
                thresholdIndex = 5;
            else {
                thresholdIndex--;
            }
        }

        if (gamepad1.dpad_right && !leftPressed && !rightPressed) {
            if (thresholdIndex == 5)
                thresholdIndex = 0;
            else {
                thresholdIndex++;
            }
        }

        switch (thresholdIndex) {
            case 1:
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                break;
            case 5:
                break;
            case 6:
                break;
        }

        upPressed = gamepad1.dpad_up;
        downPressed = gamepad1.dpad_down;
        leftPressed = gamepad1.dpad_right;
        rightPressed = gamepad1.dpad_right;

        addTelemetry(thresholdIndex);
        addTelemetry(currentPixel);
        addTelemetry(pixels);
    }
}
