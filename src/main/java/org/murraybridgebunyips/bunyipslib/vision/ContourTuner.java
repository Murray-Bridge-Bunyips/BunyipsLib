package org.murraybridgebunyips.bunyipslib.vision;

import static org.murraybridgebunyips.bunyipslib.Text.round;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.Text;
import org.murraybridgebunyips.bunyipslib.vision.processors.ColourThreshold;
import org.opencv.core.Scalar;

/**
 * A tuning OpMode for calibrating the vision system's colour thresholding using controller input.
 *
 * @author Lucas Bubner, 2024
 */
public abstract class ContourTuner extends BunyipsOpMode {
    private final double[] scalars = new double[6];
    private ColourThreshold[] processors;
    private int processorIdx;
    private int scalarIdx;
    private String[] channelNames;
    private Vision vision;

    /**
     * The camera from HardwareMap to use for vision processing.
     *
     * @return the camera to use
     */
    protected abstract CameraName setCamera();

    /**
     * The processors to tune in this OpMode.
     *
     * @return the processors to tune, will be able to switch between them during runtime
     */
    protected abstract ColourThreshold[] setThresholdsToTune();

    @Override
    protected final void onInit() {
        vision = new Vision(setCamera());
        processors = setThresholdsToTune();
        if (processors.length == 0)
            throw new EmergencyStop("No processors to tune");
        // Initialise all processors but start only one
        vision.init(processors);
        vision.startPreview();
        changeToProcessor(0);
    }

    private void changeToProcessor(int index) {
        // Stop old processor
        vision.stop(processors[processorIdx]);
        // Repopulate scalars with new processor's scalars
        for (int i = 0; i < 3; i++)
            scalars[i] = processors[index].getLower().val[i];
        for (int i = 3; i < 5; i++)
            scalars[i] = processors[index].getUpper().val[i];
        // Start new processor
        vision.start(processors[index]);
        vision.setPreview(processors[index]);
        // Cache the namespaces of the channel (e.g. "Red", "Green", "Blue")
        channelNames = new String[]{
                processors[index].colourSpace.getChannelName(0),
                processors[index].colourSpace.getChannelName(1),
                processors[index].colourSpace.getChannelName(2)
        };
    }

    @Override
    protected final void activeLoop() {
        // Scalar selection
        if (gamepad1.getDebounced(Controls.DPAD_DOWN)) {
            scalarIdx++;
            if (scalarIdx > 5)
                scalarIdx = 5;
        } else if (gamepad1.getDebounced(Controls.DPAD_UP)) {
            scalarIdx--;
            if (scalarIdx < 0)
                scalarIdx = 0;
        }

        // Processor selection
        if (gamepad1.getDebounced(Controls.RIGHT_BUMPER)) {
            processorIdx++;
            if (processorIdx >= processors.length)
                processorIdx = 0;
            changeToProcessor(processorIdx);
        } else if (gamepad1.getDebounced(Controls.LEFT_BUMPER)) {
            processorIdx--;
            if (processorIdx < 0)
                processorIdx = processors.length - 1;
            changeToProcessor(processorIdx);
        }

        // Scalar adjustment
        scalars[scalarIdx] -= gamepad1.lsy;
        scalars[scalarIdx] = round(scalars[scalarIdx], 2);

        telemetry.add("LB/RB: Select processor to tune").small();
        telemetry.add("Dpad-Up: Move scalar index up").small();
        telemetry.add("Dpad-Down: Move scalar index down").small();
        telemetry.add("Left Y: Adjust selected scalar").small();
        telemetry.add("A: Print current values to Logcat").small();
        telemetry.addNewLine();

        telemetry.add("Tuning '%'", processors[processorIdx]).h4().bold();

        // Display scalars and highlight the selected scalar
        for (int i = 0; i < 5; i++) {
            int finalI = i;
            telemetry.add("%[%]: %", i < 3 ? "Lower" : "Upper", channelNames[i % 3], scalars[i], 2)
                    .color("green").bold().applyStylesIf(() -> scalarIdx == finalI);
        }

        // Update the processor's scalars
        processors[processorIdx].setLower(new Scalar(scalars[0], scalars[1], scalars[2]));
        processors[processorIdx].setUpper(new Scalar(scalars[3], scalars[4], scalars[5]));

        // Save functionality
        if (gamepad1.a) {
            String lower = Text.format("['%' lower] Scalar(%, %, %)", processors[processorIdx], scalars[0], scalars[1], scalars[2]);
            String upper = Text.format("['%' upper] Scalar(%, %, %)", processors[processorIdx], scalars[3], scalars[4], scalars[5]);
            Dbg.log(lower + "\n" + upper);
            telemetry.log(lower + "\n" + upper);
        }
    }
}
