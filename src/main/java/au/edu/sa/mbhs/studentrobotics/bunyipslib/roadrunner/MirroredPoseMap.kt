package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner

import com.acmerobotics.roadrunner.Arclength
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PoseMap

/**
 * Perform a reflection across the X-axis.
 * This reflects Red or Blue trajectories across the alliance plane, such as in the CENTERSTAGE field.
 *
 * (x,y,r) -> (x,-y,-r)
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
open class MirroredPoseMap : PoseMap {
    override fun map(pose: Pose2dDual<Arclength>) =
        Pose2dDual(pose.position.x, -pose.position.y, pose.heading.inverse())
}