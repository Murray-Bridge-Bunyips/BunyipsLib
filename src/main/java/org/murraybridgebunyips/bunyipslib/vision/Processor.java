package org.murraybridgebunyips.bunyipslib.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.murraybridgebunyips.bunyipslib.vision.data.VisionData;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Base class for all vision processors using the Vision system.
 * <p>
 * A processor will be attached to a Vision instance and will be called to process frames,
 * allowing you to access your data here using the .getData() method. This makes it useful
 * for tasks to access the latest data from the vision system, without needing to directly
 * interface with the Vision instance.
 *
 * @author Lucas Bubner, 2023
 */
public abstract class Processor<T extends VisionData> implements VisionProcessor, CameraStreamSource {

    /**
     * List of all vision data detected since the last stateful update
     */
    protected final List<T> data = Collections.synchronizedList(new ArrayList<>());

    /**
     * Bitmap for use with FtcDashboard and Bitmap processing
     */
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    protected volatile Object userContext;
    private volatile Mat frame = new Mat();

    /**
     * Whether the camera stream should be processed with a vertical and horizontal flip
     */
    private boolean isFlipped;

    private boolean isAttached;

    /**
     * Determine whether the processor is attached to a Vision instance.
     * Checking this is useful for processors that have been passed into tasks but cannot
     * be checked by looking directly at the vision system.
     */
    public boolean isAttached() {
        return isAttached;
    }

    // Package-private, set internally by Processor
    void setAttached(boolean attached) {
        isAttached = attached;
    }

    public boolean isFlipped() {
        return isFlipped;
    }

    public void setFlipped(boolean flipped) {
        isFlipped = flipped;
    }

    /**
     * Unique identifier for the processor. This will be used to identify the processor
     * in the Vision system and in the FtcDashboard processor switcher.
     */
    public abstract String getName();

    /**
     * Get the list of vision data. You should use this method as the primary way to access
     * the latest vision data from the processor, otherwise you run the risk of
     * concurrent modification exceptions.
     *
     * @return list of all vision data detected since the last stateful update
     */
    public ArrayList<T> getData() {
        synchronized (data) {
            // Return a copy of the data to prevent concurrent modification
            return new ArrayList<>(data);
        }
    }

    /**
     * Manually clear the data list.
     */
    public void clearData() {
        synchronized (data) {
            data.clear();
        }
    }

    /**
     * Called to update new data from the vision system, which involves interpreting,
     * collecting, or otherwise processing new vision data per frame.
     * <p>
     * This method should refresh `this.data` with the latest information from the vision system to
     * be accessed with your methods on .getData().T (your VisionData class). `this.data` is
     * automatically cleared upon each iteration, so opt to using realtime data in this method.
     * This method will be called automatically once attached to a Vision instance.
     */
    public abstract void update();

    /**
     * Called by the vision system to process a frame
     *
     * @param frame            the frame to process
     * @param captureTimeNanos the time the frame was captured
     * @return the processed frame context, which will be passed to onFrameDraw
     */
    public abstract Object onProcessFrame(Mat frame, long captureTimeNanos);

    @Override
    public final Object processFrame(Mat f, long captureTimeNanos) {
        // Copy the frame to prevent it from being modified across processors
        frame = f.clone();
        if (isFlipped)
            Core.flip(frame, frame, -1);
        // Run user processing
        Object procFrame = onProcessFrame(frame, captureTimeNanos);
        // Convert to a bitmap for FtcDashboard and DS feed
        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);
        // Run user drawing
        onFrameDraw(new Canvas(lastFrame.get()), procFrame);
        // Run user update
        synchronized (data) {
            data.clear();
            update();
        }
        // We're done with the copied frame, we can release it immediately
        frame.release();
        return procFrame;
    }

    /**
     * Called by the vision system to draw on the frame.
     *
     * @param canvas      the canvas to draw on
     * @param userContext the user context returned from onProcessFrame, used to pass data between methods;
     *                    this field is used with native processors such as AprilTag.
     */
    public abstract void onFrameDraw(Canvas canvas, Object userContext);

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    /**
     * Use {@link #onFrameDraw(Canvas, Object)} instead, which passes a canvas and user context (ret. val from onProcessFrame).
     * <p>
     * Width and height should be accessed with Vision.CAMERA_WIDTH and Vision.CAMERA_HEIGHT, and
     * scaleBmpPxToCanvasPx and scaleCanvasDensity should be assumed as 1.0f.
     */
    @Override
    public final void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // no-op
    }

    // Optional init method from VisionProcessor
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }
}
