package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.tuning

import com.acmerobotics.roadrunner.ftc.LazyImu
import com.qualcomm.robotcore.hardware.IMU

/**
 * Shims the [LazyImu] type from an [IMU].
 *
 * @author Lucas Bubner, 2025
 * @since 7.0.0
 */
class LazyImuShim internal constructor(private val imu: IMU) : IMU by imu, LazyImu {
    // We shim the IMU because we need to pass a LazyImu instance for tuning. We used to need to require the drive
    // instances store a LazyImu but since the interface has changed we can do this. We choose not to use LazyImu
    // at the drive level as the IMUEx class can do this lazy initialisation already, making it unnecessarily complex
    // for programmers to get the IMU type right. It is excluded from the public API surface as the user should never
    // have to interact with this shim.
    override fun get() = imu
}