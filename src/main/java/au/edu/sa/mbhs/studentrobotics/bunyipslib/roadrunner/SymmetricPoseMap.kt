package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner

import com.acmerobotics.roadrunner.Arclength
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PoseMap

/**
 * Perform a reflection across the X-axis and mirror through the Y-axis. This is useful
 * for symmetric fields where a Y-axis reflection is also required, such as the INTO THE DEEP field.
 *
 * (x,y,r) -> (-x,-y,r+180°)
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
open class SymmetricPoseMap : PoseMap {
    /**
     * Applies (x,y,r) -> (-x,-y,r+180°)
     */
    override fun map(pose: Pose2dDual<Arclength>) =
        Pose2dDual(-pose.position.x, -pose.position.y, pose.heading + Math.PI)
}