package org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import java.util.ArrayList;
import java.util.List;

/**
 * A segment of a trajectory sequence.
 *
 * @since 1.0.0-pre
 */
public abstract class SequenceSegment {
    private final double duration;
    private final Pose2d startPose;
    private final Pose2d endPose;
    private final List<TrajectoryMarker> markers;

    protected SequenceSegment(
            double duration,
            Pose2d startPose, Pose2d endPose,
            List<TrajectoryMarker> markers
    ) {
        this.duration = duration;
        this.startPose = startPose;
        this.endPose = endPose;
        this.markers = new ArrayList<>(markers);
    }

    public double getDuration() {
        return duration;
    }

    public Pose2d getStartPose() {
        return startPose;
    }

    public Pose2d getEndPose() {
        return endPose;
    }

    public List<TrajectoryMarker> getMarkers() {
        return markers;
    }
}
