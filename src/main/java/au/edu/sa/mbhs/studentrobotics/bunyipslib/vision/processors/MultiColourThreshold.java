package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors;

import android.graphics.Canvas;
import android.graphics.Paint;
import android.util.Pair;
import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;

import java.util.ArrayList;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.Processor;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data.ContourData;

/**
 * A processor that applies multiple ColourThreshold processors to the same frame and
 * combines their draw results. The MASK variable can be used to select which processor's mask
 * is drawn to the frame.
 *
 * @author Lucas Bubner, 2024
 * @see ColourThreshold
 * @since 1.0.0-pre
 */
@Config
public class MultiColourThreshold extends Processor<ContourData> {
    /**
     * The index of the processor mask to draw to the frame.
     */
    public static int MASK;
    private final ArrayList<Pair<ColourThreshold, Mat>> colourProcessors = new ArrayList<>();

    /**
     * Create a new MultiColourThreshold with the given processors.
     *
     * @param thresholdProcessors the colour processors to use
     */
    public MultiColourThreshold(@NonNull ColourThreshold... thresholdProcessors) {
        for (ColourThreshold processor : thresholdProcessors) {
            colourProcessors.add(new Pair<>(processor, new Mat()));
        }
    }

    protected void onAttach() {
        for (Pair<ColourThreshold, Mat> processor : colourProcessors) {
            processor.first.delegate(this);
        }
    }

    @NonNull
    @Override
    public String toString() {
        return "multicolourthreshold";
    }

    @Override
    protected void update() {
        for (Pair<ColourThreshold, Mat> processor : colourProcessors) {
            processor.first.clearData();
            processor.first.update();
            data.addAll(processor.first.getData());
            processor.second.release();
        }
    }

    @Override
    protected void onProcessFrame(@NonNull Mat frame, long captureTimeNanos) {
        for (Pair<ColourThreshold, Mat> processor : colourProcessors) {
            frame.copyTo(processor.second);
            processor.first.onProcessFrame(processor.second, captureTimeNanos);
        }
        if (!colourProcessors.isEmpty() && MASK >= 1 && MASK <= colourProcessors.size()) {
            colourProcessors.get(MASK - 1).second.copyTo(frame);
        }
    }

    @Override
    protected void onFrameDraw(@NonNull Canvas canvas) {
        for (Pair<ColourThreshold, Mat> processor : colourProcessors) {
            processor.first.onFrameDraw(canvas);
        }
        Size cameraDimensions = getCameraDimensions();
        if (cameraDimensions == null) return;
        // Display mask name on camera feed
        canvas.drawText(
                MASK >= 1 && MASK <= colourProcessors.size() ? colourProcessors.get(MASK - 1).first.toString() : "",
                10,
                cameraDimensions.getHeight() - 10,
                new Paint() {{
                    setColor(0xFFFFFFFF);
                    setStrokeWidth(30);
                    setTextSize(20);
                    setAntiAlias(true);
                }}
        );
    }
}
