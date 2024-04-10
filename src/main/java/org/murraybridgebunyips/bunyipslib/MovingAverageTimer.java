package org.murraybridgebunyips.bunyipslib;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Milliseconds;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Nanoseconds;

import androidx.annotation.NonNull;

import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;

/**
 * Time utilities for robot operation.
 * <a href="https://github.com/Paladins-of-St-Pauls/GameChangers/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/paladins/utils/MovingAverageTimer.java">Source</a>
 *
 * @author Shaun, 11/06/2017
 */
public class MovingAverageTimer {
    // A ring buffer is used to keep track of a moving average
    private final int ringBufferSize;
    private final long[] loopTimeRingBuffer;
    private final String avgFormatStr;
    private final String toStringFormatStr;
    private final Time formatResolution;
    private int ringBufferIndex;
    private long loopCount;
    private long movingTotal;
    private long previousTime;
    private long runningTotal;
    private Measure<Time> movingAverage;
    private Measure<Time> minMovingAverage = Nanoseconds.of(Double.MAX_VALUE);
    private Measure<Time> maxMovingAverage = Nanoseconds.of(Double.MIN_VALUE);
    private Measure<Time> average;
    private Measure<Time> minAverage = Nanoseconds.of(Double.MAX_VALUE);
    private Measure<Time> maxAverage = Nanoseconds.of(Double.MIN_VALUE);
    private Measure<Time> minLoopTime = Nanoseconds.of(Double.MAX_VALUE);
    private Measure<Time> maxLoopTime = Nanoseconds.of(Double.MIN_VALUE);

    /**
     * Create a new MovingAverageTimer with a ring buffer size of 100 and a resolution of milliseconds.
     */
    public MovingAverageTimer() {
        this(100, Milliseconds);
    }

    /**
     * Create a new MovingAverageTimer with the specified buffer size and a resolution of milliseconds.
     *
     * @param bufSize the size of the ring buffer
     */
    public MovingAverageTimer(int bufSize) {
        this(bufSize, Milliseconds);
    }

    /**
     * Create a new MovingAverageTimer with the specified buffer size and resolution.
     *
     * @param bufSize    the size of the ring buffer
     * @param formatStringResolution the resolution of the timer for format strings
     */
    public MovingAverageTimer(int bufSize, Time formatStringResolution) {
        reset();
        ringBufferSize = bufSize;
        loopTimeRingBuffer = new long[ringBufferSize];

        String hdr = String.format("\n%-12s%-12s%-12s%-12s", "Loops", "TotalTime", "MovAvg", "Avg");

        avgFormatStr = "%3.3f" + formatStringResolution.symbol();
        toStringFormatStr = hdr + " " + formatStringResolution.name() + "\n%-12d%-12.3f%-12.3f%-12.3f\n min        %-12.3f%-12.3f%-12.3f\n max        %-12.3f%-12.3f%-12.3f";
        formatResolution = formatStringResolution;
    }

    /**
     * Reset the timer.
     */
    public void reset() {
        loopCount = 0;
        previousTime = System.nanoTime();
        movingTotal = 0;
        runningTotal = 0;
        movingAverage = Nanoseconds.zero();
        average = Nanoseconds.zero();
    }

    /**
     * Update the timer. Should be called once per loop.
     */
    public void update() {
        long now = System.nanoTime();
        long loopTime = now - previousTime;
        previousTime = now;

        if (loopCount > 0) {
            minLoopTime = Nanoseconds.of(Math.min(minLoopTime.in(Nanoseconds), loopTime));
            maxLoopTime = Nanoseconds.of(Math.max(maxLoopTime.in(Nanoseconds), loopTime));
        }

        // Adjust the running total
        movingTotal -= loopTimeRingBuffer[ringBufferIndex] + loopTime;
        runningTotal += loopTime;

        // Add the new value
        loopTimeRingBuffer[ringBufferIndex] = loopTime;

        // Wrap the current index
        ringBufferIndex++;
        ringBufferIndex %= ringBufferSize;

        loopCount++;

        if (loopCount < ringBufferSize) {
            if (loopCount == 0) {
                movingAverage = Nanoseconds.zero();
            } else {
                movingAverage = Nanoseconds.of((double) movingTotal / loopCount);
            }
            // Temporarily fill the min/max movingAverage
            minMovingAverage = Nanoseconds.of(Math.min(minMovingAverage.in(Nanoseconds), movingAverage.in(Nanoseconds)));
            maxMovingAverage = Nanoseconds.of(Math.max(maxMovingAverage.in(Nanoseconds), movingAverage.in(Nanoseconds)));

        } else {
            movingAverage = Nanoseconds.of((double) movingTotal / ringBufferSize);

            // Reset the min/max movingAverage values the each time the buffer is filled
            if (ringBufferIndex == 0) {
                minMovingAverage = movingAverage;
                maxMovingAverage = movingAverage;
            } else {
                minMovingAverage = Nanoseconds.of(Math.min(minMovingAverage.in(Nanoseconds), movingAverage.in(Nanoseconds)));
                maxMovingAverage = Nanoseconds.of(Math.max(maxMovingAverage.in(Nanoseconds), movingAverage.in(Nanoseconds)));
            }
        }

        average = Nanoseconds.of((double) runningTotal / loopCount);
        minAverage = Nanoseconds.of(Math.min(minAverage.in(Nanoseconds), average.in(Nanoseconds)));
        maxAverage = Nanoseconds.of(Math.min(maxAverage.in(Nanoseconds), average.in(Nanoseconds)));
    }

    /**
     * @return the number of loops that have been counted
     */
    public long count() {
        return loopCount;
    }

    /**
     * @param unit the time unit to return the loops per unit in
     * @return the number of loops per unit
     */
    public double loopsPer(Time unit) {
        return loopCount / (Nanoseconds.of(runningTotal).in(unit));
    }

    /**
     * @param unit the time unit to return the loops per unit in
     * @return the moving average loop time
     */
    public double movingAverage(Time unit) {
        return movingAverage.in(unit);
    }

    /**
     * @param unit the time unit to return the loops per unit in
     * @return the minimum moving average loop time
     */
    public double minMovingAverage(Time unit) {
        return minMovingAverage.in(unit);
    }

    /**
     * @param unit the time unit to return the loops per unit in
     * @return the maximum moving average loop time
     */
    public double maxMovingAverage(Time unit) {
        return maxMovingAverage.in(unit);
    }

    /**
     * @return A string representation of the moving average loop time based on the format resolution
     */
    public String movingAverageString() {
        return String.format(avgFormatStr, movingAverage);
    }

    /**
     * @param unit the time unit to return the loops per unit in
     * @return the average loop time
     */
    public double average(Time unit) {
        return average.in(unit);
    }

    /**
     * @param unit the time unit to return the loops per unit in
     * @return the minimum average loop time
     */
    public double minAverage(Time unit) {
        return minAverage.in(unit);
    }

    /**
     * @param unit the time unit to return the loops per unit in
     * @return the maximum average loop time
     */
    public double maxAverage(Time unit) {
        return maxAverage.in(unit);
    }

    /**
     * @param unit the time unit to return the loops per unit in
     * @return the minimum loop time
     */
    public double minLoopTime(Time unit) {
        return minLoopTime.in(unit);
    }

    /**
     * @param unit the time unit to return the loops per unit in
     * @return the maximum loop time
     */
    public double maxLoopTime(Time unit) {
        return maxLoopTime.in(unit);
    }

    /**
     * @param unit the time unit to return the loops per unit in
     * @return the total time elapsed
     */
    public double elapsedTime(Time unit) {
        return Nanoseconds.of(runningTotal).in(unit);
    }

    /**
     * @return A string representation of the average loop time based on the format resolution
     */
    public String averageString() {
        return String.format(avgFormatStr, average);
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(toStringFormatStr, loopCount, elapsedTime(formatResolution), movingAverage, average, minLoopTime(formatResolution), minMovingAverage, minAverage, maxLoopTime(formatResolution), maxMovingAverage, maxAverage);
    }
}
