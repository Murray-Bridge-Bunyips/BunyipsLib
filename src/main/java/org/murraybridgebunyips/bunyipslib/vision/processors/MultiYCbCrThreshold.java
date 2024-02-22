package org.murraybridgebunyips.bunyipslib.vision.processors;

import android.graphics.Canvas;
import android.graphics.Paint;
import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;

import org.murraybridgebunyips.bunyipslib.vision.Processor;
import org.murraybridgebunyips.bunyipslib.vision.Vision;
import org.murraybridgebunyips.bunyipslib.vision.data.ContourData;
import org.opencv.core.Mat;

import java.util.ArrayList;

/**
 * A processor that applies multiple YCbCrColourThreshold processors to the same frame and
 * combines their draw results. The MASK variable can be used to select which processor's mask
 * is drawn to the frame.
 *
 * @author Lucas Bubner, 2024
 * @see YCbCrColourThreshold
 */
@Config
public class MultiYCbCrThreshold extends Processor<ContourData> {
    public static int MASK;
    private final ArrayList<Pair<YCbCrColourThreshold, Mat>> colourProcessors = new ArrayList<>();

    public MultiYCbCrThreshold(YCbCrColourThreshold... thresholdProcessors) {
        for (YCbCrColourThreshold processor : thresholdProcessors) {
            colourProcessors.add(new Pair<>(processor, new Mat()));
        }
    }

    @Override
    public String getName() {
        return "multiycbcr";
    }

    @Override
    public void update() {
        for (Pair<YCbCrColourThreshold, Mat> processor : colourProcessors) {
            processor.first.clearData();
            processor.first.update();
            data.addAll(processor.first.getData());
            processor.second.release();
        }
    }

    @Override
    public void onProcessFrame(Mat frame, long captureTimeNanos) {
        for (Pair<YCbCrColourThreshold, Mat> processor : colourProcessors) {
            frame.copyTo(processor.second);
            processor.first.onProcessFrame(processor.second, captureTimeNanos);
        }
        if (!colourProcessors.isEmpty() && MASK >= 1 && MASK <= colourProcessors.size()) {
            colourProcessors.get(MASK - 1).second.copyTo(frame);
        }
    }

    @Override
    public void onFrameDraw(Canvas canvas) {
        for (Pair<YCbCrColourThreshold, Mat> processor : colourProcessors) {
            processor.first.onFrameDraw(canvas);
        }
        // Display mask name on camera feed
        canvas.drawText(
                MASK >= 1 && MASK <= colourProcessors.size() ? colourProcessors.get(MASK - 1).first.getName() : "",
                10,
                Vision.CAMERA_HEIGHT - 10,
                new Paint() {{
                    setColor(0xFFFFFFFF);
                    setStrokeWidth(30);
                    setTextSize(20);
                    setAntiAlias(true);
                }}
        );
    }
}
