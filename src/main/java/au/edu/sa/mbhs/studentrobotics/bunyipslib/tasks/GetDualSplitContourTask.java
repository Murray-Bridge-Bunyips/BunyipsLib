package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Direction;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data.ContourData;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.ColourThreshold;

/**
 * A generic task to get the position of a contour based on two (inferred three) positions, where the contour in question
 * may be viewed in the two halves of the camera frame.
 * <p>
 * An example of this task is use with CENTERSTAGE's Team Prop detection, where the camera will face the Center and Right
 * Spike Marks, where if the prop is seen on the left, it is in the center, else the right. If no contour is detected, the
 * prop is assumed to be on the left as it cannot be seen.
 * <p>
 * This task will return a {@link Direction} on what side of the camera the contour is visible on, or {@link Direction#ZERO}.
 *
 * @author Lucas Bubner, 2024
 * @since 4.0.0
 */
public class GetDualSplitContourTask extends Task {
    private final ElapsedTime lockoutTimer = new ElapsedTime();
    private ColourThreshold colourThreshold;
    private Measure<Time> noDetectionPersistenceTime = Seconds.of(3);
    private volatile Direction position = Direction.ZERO;
    @Nullable
    private Telemetry.Item item;

    /**
     * Create a new GetDualSplitContourTask.
     *
     * @param colourThreshold the initialised and running colour threshold processor
     */
    public GetDualSplitContourTask(@NonNull ColourThreshold colourThreshold) {
        this();
        this.colourThreshold = colourThreshold;
    }

    /**
     * Create a new GetDualSplitContourTask.
     */
    public GetDualSplitContourTask() {
        withName("Get Dual Split Contour");
    }

    /**
     * Set the time for no detections to be persistent before the position is set to ZERO.
     *
     * @param time the time for no detections to be persistent before the position is set to ZERO from being LEFT
     *             or RIGHT
     * @return this
     */
    @NonNull
    public GetDualSplitContourTask withNoDetectionPersistenceTime(@NonNull Measure<Time> time) {
        noDetectionPersistenceTime = time;
        return this;
    }

    /**
     * Set the processor to use for the GetDualSplitContourTask.
     *
     * @param newProcessor the processor to use
     * @return this
     */
    @NonNull
    public GetDualSplitContourTask setProcessor(@NonNull ColourThreshold newProcessor) {
        if (!newProcessor.isRunning())
            throw new IllegalStateException("Processor not attached and running on an active vision processor");
        colourThreshold = newProcessor;
        return this;
    }

    /**
     * {@link #getPosition()} with custom mappings for the camera left, right, and none detections.
     * This is a polyfill for the legacy (renamed) GetTriPositionContourTask to not rewrite logical branches.
     *
     * @param camLeft     the direction to return if the contour is on the left side of the camera
     * @param camRight    the direction to return if the contour is on the right side of the camera
     * @param noDetection the direction to return if there are no contours on the camera for
     *                    minimum {@link #withNoDetectionPersistenceTime} time
     * @return the mapped direction
     */
    @NonNull
    public Direction getMappedPosition(@NonNull Direction camLeft, @NonNull Direction camRight, @NonNull Direction noDetection) {
        return position == Direction.LEFT ? camLeft : position == Direction.RIGHT ? camRight : noDetection;
    }

    /**
     * @return the position of the contour as seen by the camera, can either be {@link Direction#LEFT}, {@link Direction#RIGHT},
     * or {@link Direction#ZERO}.
     */
    @NonNull
    public Direction getPosition() {
        return position;
    }

    private String buildString() {
        return "Position(" + this + "): " + position;
    }

    @Override
    protected void init() {
        if (opMode != null)
            item = opMode.telemetry.addRetained(buildString()).getItem();
        if (colourThreshold == null)
            return;
        if (!colourThreshold.isRunning())
            throw new IllegalStateException("Processor not attached and running on an active vision processor");
    }

    @Override
    protected void periodic() {
        if (colourThreshold == null)
            return;
        ContourData biggestContour = ContourData.getLargest(colourThreshold.getData());
        if (biggestContour != null) {
            position = biggestContour.getYaw() > 0.5 ? Direction.RIGHT : Direction.LEFT;
            lockoutTimer.reset();
        } else if (lockoutTimer.seconds() >= noDetectionPersistenceTime.in(Seconds)) {
            position = Direction.ZERO;
        }
        if (item != null)
            item.setValue(buildString());
    }

    @Override
    protected void onFinish() {
        if (item != null)
            require(opMode).telemetry.remove(item);
    }

    @Override
    protected boolean isTaskFinished() {
        return false;
    }
}
