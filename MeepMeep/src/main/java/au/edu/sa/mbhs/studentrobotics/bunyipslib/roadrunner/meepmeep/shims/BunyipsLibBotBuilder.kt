package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims

import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Angle
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Distance
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.Inches
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.InchesPerSecond
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.InchesPerSecondPerSecond
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.RadiansPerSecond
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.RadiansPerSecondPerSecond
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Velocity
import com.acmerobotics.roadrunner.Pose2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.core.colorscheme.ColorScheme
import com.noahbres.meepmeep.roadrunner.Constraints
import com.noahbres.meepmeep.roadrunner.DriveTrainType
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity

@Suppress("KDocMissingDocumentation")
class BunyipsLibBotBuilder(private val meepMeep: MeepMeep) {
    private var constraints = Constraints(
        30.0, 30.0, Math.toRadians(60.0), Math.toRadians(60.0), 15.0
    )

    private var width = 18.0
    private var height = 18.0

    private var startPose = Pose2d(0.0, 0.0, 0.0)
    private var colorScheme: ColorScheme? = null
    private var opacity = 0.8

    private var driveTrainType = DriveTrainType.MECANUM

    fun setDimensions(widthIn: Double, heightIn: Double) = apply {
        width = widthIn
        height = heightIn
    }

    fun setWidth(width: Double, unit: Distance) = apply {
        this.width = unit.of(width).inUnit(Inches)
    }

    fun setHeight(height: Double, unit: Distance) = apply {
        this.height = unit.of(height).inUnit(Inches)
    }

    fun setStartPose(pose: Pose2d) = apply {
        this.startPose = pose
    }

    fun setConstraints(constraintsIn: Constraints) = apply {
        constraints = constraintsIn
    }

    fun setConstraints(
        maxVelIn: Double,
        maxAccelIn: Double,
        maxAngVelRad: Double,
        maxAngAccelRad: Double,
        trackWidthIn: Double
    ) = apply {
        constraints = Constraints(maxVelIn, maxAccelIn, maxAngVelRad, maxAngAccelRad, trackWidthIn)
    }

    fun setMaxVel(maxVel: Double, unit: Velocity<Distance>) = apply {
        constraints = constraints.copy(maxVel = unit.of(maxVel).inUnit(InchesPerSecond))
    }

    fun setMaxAccel(maxAccel: Double, unit: Velocity<Velocity<Distance>>) = apply {
        constraints = constraints.copy(maxAccel = unit.of(maxAccel).inUnit(InchesPerSecondPerSecond))
    }

    fun setMaxAngVel(maxAngVel: Double, unit: Velocity<Angle>) = apply {
        constraints = constraints.copy(maxAngVel = unit.of(maxAngVel).inUnit(RadiansPerSecond))
    }

    fun setMaxAngAccel(maxAngAccel: Double, unit: Velocity<Velocity<Angle>>) = apply {
        constraints = constraints.copy(maxAngAccel = unit.of(maxAngAccel).inUnit(RadiansPerSecondPerSecond))
    }

    fun setTrackWidth(trackWidth: Double, unit: Distance) = apply {
        constraints = constraints.copy(trackWidth = unit.of(trackWidth).inUnit(Inches))
    }

    fun setOpacity(opacity: Double) = apply {
        this.opacity = opacity
    }

    fun setDriveTrainType(driveTrainType: DriveTrainType) = apply {
        this.driveTrainType = driveTrainType
    }

    fun setColorScheme(scheme: ColorScheme) = apply {
        this.colorScheme = scheme
    }

    fun build() = RoadRunnerBotEntity(
        meepMeep,
        constraints,
        width, height,
        startPose, colorScheme ?: meepMeep.colorManager.theme, opacity,
        driveTrainType, false
    ).also {
        MeepMeepInternal.drive.__internalSetup(
            { p -> it.drive.actionBuilder(p) },
            { constraints },
            { driveTrainType },
            { a -> it.runAction(a) })
    }
}