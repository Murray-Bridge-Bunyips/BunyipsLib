package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryActionFactory;
import com.acmerobotics.roadrunner.TurnActionFactory;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;

/**
 * Interface implemented by all RoadRunner drive classes.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public interface RoadRunnerDrive extends Moveable {
    /**
     * Create a new TrajectoryActionBuilder to begin building a new trajectory action.
     *
     * @param beginPose the starting pose of this trajectory
     * @return the new TrajectoryActionBuilder
     */
    TrajectoryActionBuilder actionBuilder(Pose2d beginPose);

    /**
     * Get a reference to a factory that can produce a {@link TimeTrajectory} follower task.
     *
     * @return a TrajectoryActionFactory
     */
    TrajectoryActionFactory newTrajectoryTask();

    /**
     * Get a reference to a factory that can produce a {@link TimeTurn} follower task.
     *
     * @return a TurnActionFactory
     */
    TurnActionFactory newTurnTask();

    /**
     * Get the drive model parameters of this RoadRunnerDrive.
     *
     * @return the used drive model parameters
     */
    DriveModel getDriveModel();

    /**
     * Get the motion profile parameters of this RoadRunnerDrive.
     *
     * @return the used motion profile parameters
     */
    MotionProfile getMotionProfile();
}
