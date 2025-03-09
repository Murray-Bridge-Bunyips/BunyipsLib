package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dbg;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data.VisionData;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.MultiColourThreshold;

/**
 * Base class for all vision processors using the Vision system.
 * <p>
 * A processor will be attached to a Vision instance and will be called to process frames,
 * allowing you to access your data here using the {@link #getData()} method. This makes it useful
 * for tasks to access the latest data from the vision system, without needing to directly
 * interface with the Vision instance.
 *
 * @param <T> the type of VisionData to be processed
 * @author Lucas Bubner, 2023
 * @since 1.0.0-pre
 */
public abstract class Processor<T extends VisionData> implements VisionProcessor, CameraStreamSource {
    /**
     * List of all vision data detected since the last stateful update.
     * Should be updated with the most up-to-date data, as it will be cleared before every update.
     */
    protected final List<T> data = Collections.synchronizedList(new ArrayList<>());

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    private boolean isFlipped;
    // Camera dimensions are used as a measure to determine if a camera is attached to this processor
    private Size cameraDimensions = null;
    private boolean isRunning;

    // Package-private, set internally by Vision
    void detach() {
        cameraDimensions = null;
        isRunning = false;
    }

    void attach(Size size) {
        cameraDimensions = size;
        onAttach();
    }

    /**
     * Delegate this processor to another processor. This will mimic setting the resolution to the one of the parent processor,
     * thereby forcing the enabled and attached states to true. Ensure your parent processor is attached prior to delegation,
     * as resolution data will be available. Note that status checks to this delegated processor may not represent actual camera operation,
     * as these properties are now proxied but not updated.
     * <p>
     * This method is not designed to be used by the end-user. Only use if you're absolutely sure what you're doing.
     *
     * @param delegateTo The parent class that this processor should delegate to
     * @see MultiColourThreshold MultiColourThreshold - line 41
     */
    @SuppressWarnings("rawtypes")
    public void delegate(@NonNull Processor delegateTo) {
        if (!delegateTo.isAttached()) {
            Dbg.error(getClass(), "Delegation failed, parent processor (%) is not attached to a Vision instance.", delegateTo);
            return;
        }
        Dbg.logd(getClass(), "Processor has been delegated via %.", delegateTo);
        cameraDimensions = delegateTo.cameraDimensions;
        // We will also set the running status to true as well, since we need to delegate the processor status to the class
        // that is managing this processor. This ensures any checks that rely on these types of checks pass, as they are
        // technically running but by the proxy class.
        isRunning = true;
    }

    /**
     * Determine whether the processor is attached to a Vision instance or is initialised.
     * <p>
     * Checking this is useful for processors that have been passed into tasks but cannot
     * be checked by looking directly at the vision system.
     */
    public boolean isAttached() {
        return cameraDimensions != null;
    }

    /**
     * Determine whether the processor is currently started on a Vision instance.
     * This will also check if the processor is attached.
     * This will not reflect whether the Vision instance is streaming, only if the processor has been started.
     * <p>
     * Checking this is useful for processors that have been passed into tasks but cannot
     * be checked by looking directly at the vision system.
     */
    public boolean isRunning() {
        return cameraDimensions != null && isRunning;
    }

    void setRunning(boolean running) {
        if (running)
            onRunning();
        isRunning = running;
    }

    /**
     * Whether the camera stream should be processed with a vertical and horizontal flip
     *
     * @return whether {@link #setFlipped} was called and the camera was flipped
     */
    public boolean isFlipped() {
        return isFlipped;
    }

    /**
     * Rotate and flip the vision processor.
     *
     * @param flipped whether to flip the processor.
     */
    public void setFlipped(boolean flipped) {
        isFlipped = flipped;
    }

    /**
     * Get the attached camera dimensions/resolution that this processor is running on.
     *
     * @return the dimensions in pixels of the attached camera instance, or null if there is no attached instance
     */
    @Nullable
    public Size getCameraDimensions() {
        return cameraDimensions;
    }

    /**
     * Unique identifier for the processor. This will be used to identify the processor
     * in the Vision system and in the FtcDashboard processor switcher.
     */
    @NonNull
    public abstract String getId();

    @Override
    @NonNull
    public final String toString() {
        return getId();
    }

    /**
     * Get the list of vision data. You should use this method as the primary way to access
     * the latest vision data from the processor from an OpMode, otherwise you run the risk of
     * concurrent modification exceptions. This does not apply to within a processor as the
     * methods are synchronized.
     *
     * @return list of all vision data detected since the last stateful update
     */
    @NonNull
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
     * Override this method to run any additional code that will be executed when
     * this processor is attached (via {@link Vision#init}) by a Vision instance.
     * <p>
     * This method is also called on standard initialisation of the processor if it has not been already done via Vision.
     */
    protected void onAttach() {
        // no-op
    }

    /**
     * Override this method to run any additional code that will be executed when
     * this processor starts streaming (via {@link Vision#start}) on a Vision instance.
     */
    protected void onRunning() {
        // no-op
    }

    /**
     * Called to update new data from the vision system, which involves interpreting,
     * collecting, or otherwise processing new vision data per frame.
     * <p>
     * This method should refresh {@link #data} with the latest information from the vision system to
     * be accessed later via {@link #getData()} (returning your derived {@link VisionData} class).
     * {@link #data} is automatically cleared upon each iteration, so opt to using realtime data in this method.
     * This method will be called automatically once attached to a Vision instance.
     */
    protected abstract void update();

    /**
     * Called by the vision system to process a frame.
     *
     * @param frame            the frame to process
     * @param captureTimeNanos the time the frame was captured
     */
    protected abstract void onProcessFrame(@NonNull Mat frame, long captureTimeNanos);

    @Nullable
    @Override
    public final Object processFrame(@NonNull Mat frame, long captureTimeNanos) {
        if (isFlipped)
            Core.flip(frame, frame, -1);
        // Run user processing
        onProcessFrame(frame, captureTimeNanos);
        // Convert to a bitmap for FtcDashboard and DS feed
        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        synchronized (data) {
            data.clear();
            // Run user data update
            update();
            // Run user drawing while still having a data lock
            onFrameDraw(new Canvas(b));
        }
        lastFrame.set(b);
        // User context is not needed, as processors that need it should use the data list or
        // hold a copy of the user context when supplied to them in onProcessFrame
        return null;
    }

    /**
     * Called by the vision system to draw on the frame.
     *
     * @param canvas the canvas to draw on
     */
    protected abstract void onFrameDraw(@NonNull Canvas canvas);

    @Override
    public void getFrameBitmap(@NonNull Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    /**
     * Use {@link #onFrameDraw(Canvas)} instead, which passes a canvas. Your userContext should
     * be instead acquired by updating the data list in {@link #update()}. If this is not possible,
     * you can simply hold a copy of the userContext when supplied to you in {@link #onProcessFrame(Mat, long)}.
     * <p>
     * Width and height should be accessed with {@link #getCameraDimensions()}, and
     * scaleBmpPxToCanvasPx and scaleCanvasDensity should be assumed as 1.0f.
     */
    @Override
    public final void onDrawFrame(@NonNull Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, @NonNull Object userContext) {
        // no-op
    }

    @Override
    public final void init(int width, int height, @Nullable CameraCalibration calibration) {
        if (cameraDimensions == null) {
            cameraDimensions = new Size(width, height);
            onAttach();
        }
        init(calibration);
    }

    /**
     * Optional method to implement to access the camera calibration data.
     * To access camera dimensions, use {@link #getCameraDimensions()}.
     *
     * @param calibration the camera calibration data
     */
    public void init(@Nullable CameraCalibration calibration) {
    }
}
