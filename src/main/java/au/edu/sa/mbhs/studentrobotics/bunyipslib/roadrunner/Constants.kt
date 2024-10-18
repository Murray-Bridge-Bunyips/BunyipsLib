package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner

import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.TrajectoryActionFactory
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.TurnActionFactory
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.VelConstraint

/**
 * Data class storing all the required constants for using RoadRunner motion planning.
 *
 * @param driveModel the drive model to use
 * @param motionProfile the motion profile to use
 * @param turnActionFactory the turn action factory to use
 * @param trajectoryActionFactory the trajectory action factory to use
 * @param trajectoryBuilderParams the trajectory builder parameters to use
 * @param beginEndVel the beginning and ending velocity of the trajectory
 * @param baseTurnConstraints the base turn constraints to use
 * @param baseVelConstraint the base velocity constraint to use
 * @param baseAccelConstraint the base acceleration constraint to use
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
data class Constants(
    val driveModel: DriveModel,
    val motionProfile: MotionProfile,
    val turnActionFactory: TurnActionFactory,
    val trajectoryActionFactory: TrajectoryActionFactory,
    val trajectoryBuilderParams: TrajectoryBuilderParams,
    val beginEndVel: Double,
    val baseTurnConstraints: TurnConstraints,
    val baseVelConstraint: VelConstraint,
    val baseAccelConstraint: AccelConstraint,
)