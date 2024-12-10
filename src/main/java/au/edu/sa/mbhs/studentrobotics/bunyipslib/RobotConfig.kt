package au.edu.sa.mbhs.studentrobotics.bunyipslib

import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.IMUEx
import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.InvertibleTouchSensor
import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.Motor
import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.ServoEx
import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.SimpleRotator
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Storage
import com.acmerobotics.roadrunner.ftc.LazyImu
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import java.util.function.Consumer

/**
 * Abstract class to use as parent to the class you will define to mirror a "saved configuration"
 * on the Robot Controller, and to define any robot related constants or subsystems.
 * Supported for use in both BunyipsOpModes and any other normal SDK OpMode. This paradigm is known as RobotHardware.
 * ```
 *     private final YourConfig config = new YourConfig();
 * ```
 * In your OpMode's init method, call the `init` method of your config class, passing in the OpMode.
 * ```
 *     config.init(this);
 * ```
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
abstract class RobotConfig {
    private var hasInitCalled = false

    /**
     * OpMode supplied hardwareMap instance.
     */
    protected lateinit var hardwareMap: HardwareMap

    /**
     * This method is executed when the `init` method is called on this RobotConfig instance. In this method,
     * you have access to the `hardwareMap`, as well as utility methods such as `getHardware` to retrieve and configure
     * hardware. Devices and subsystems that you would like to expose should be listed as public fields in your config.
     */
    protected abstract fun onRuntime()

    /**
     * Uses the HardwareMap to fetch HardwareDevices and assign instances from `onRuntime`.
     * Should be called as the first line in your init cycle. This method can only be executed once.
     * @param opMode the OpMode instance - usually the `this` object when at the root OpMode.
     * @return the instance of the RobotConfig
     */
    fun init(opMode: OpMode): RobotConfig {
        if (hasInitCalled)
            throw IllegalStateException("RobotConfig instance was already initialised.")
        Storage.memory().hardwareErrors.clear()
        this.hardwareMap = opMode.hardwareMap
        if (opMode is BunyipsOpMode) {
            Exceptions.runUserMethod(opMode, ::onRuntime)
            opMode.t.add("<b>${javaClass.simpleName}</b>: Hardware initialised with ${if (Storage.memory().hardwareErrors.size > 0) "<font color='red'>${Storage.memory().hardwareErrors.size} error(s)</font>" else "<font color='green'>0 errors</font>"}.")
        } else {
            onRuntime()
            opMode.telemetry.addData(
                "",
                "${javaClass.simpleName}: Hardware initialised with ${Storage.memory().hardwareErrors.size} error(s).",
            )
        }
        for (error in Storage.memory().hardwareErrors) {
            if (opMode is BunyipsOpMode) {
                opMode.t.addRetained("<font color='red'><b>! MISSING DEVICE</b></font>: $error")
                opMode.t.addRetained("<font color='red'>error:</font> <i>$error</i> was not found in the current saved configuration.")
            } else {
                opMode.telemetry.addData("", "! MISSING DEVICE: $error").setRetained(true)
                opMode.telemetry.log().add("error: '$error' was not found in the current saved configuration.")
            }
        }
        hasInitCalled = true
        return this
    }

    /**
     * Implicit OpMode config initialisation for use in BunyipsOpModes. This will not work in normal SDK OpModes.
     *
     * Uses the HardwareMap to fetch HardwareDevices and assign instances from `onRuntime`.
     * Should be called as the first line in your init cycle.
     *
     * @throws UnsupportedOperationException if not called from a BunyipsOpMode.
     * @see init(opMode: OpMode)
     * @return the instance of the RobotConfig
     */
    fun init(): RobotConfig {
        try {
            // Access the singleton associated with a BunyipsOpMode, if we're not running one Kotlin
            // will throw a UninitializedPropertyAccessException, so we can tell the user off here.
            return init(BunyipsOpMode.instance)
        } catch (e: UninitializedPropertyAccessException) {
            throw UnsupportedOperationException("Argument-less .init() method is only supported in a BunyipsOpMode. Use .init(this) instead.")
        }
    }

    private val dcMotorCastable = listOf(RawEncoder::class.java, Ramping.DcMotor::class.java, Motor::class.java)

    @Suppress("UNCHECKED_CAST")
    private fun <T> getAndDynamicCast(name: String, device: Class<T>): T? {
        // These devices are known to replicate DcMotor functionality, and we can auto-fetch and safely cast
        // to the new type for use by the user. We can ignore any unchecked cast warnings as these are checked.
        dcMotorCastable.forEach { it ->
            if (it.isAssignableFrom(device)) {
                val motor = hardwareMap.get(DcMotor::class.java, name)
                // First check for DcMotorEx constructors that we can use
                if (it.constructors.any { it.parameterTypes.contains(DcMotorEx::class.java) })
                    return it.getConstructor(DcMotorEx::class.java).newInstance(motor) as T
                return it.getConstructor(DcMotor::class.java).newInstance(motor) as T
            }
        }
        // IMUEx is a drop-in replacement for IMU and can be fetched here too
        if (IMUEx::class.java.isAssignableFrom(device)) {
            val imu = hardwareMap.get(IMU::class.java, name)
            return IMUEx(imu) as T
        }
        // ServoEx is the equivalent of Motor in the sense that it is a full extension and can be fetched too
        if (ServoEx::class.java.isAssignableFrom(device)) {
            val servo = hardwareMap.get(Servo::class.java, name)
            return ServoEx(servo) as T
        }
        // SimpleRotator is a CRServo and DcMotorSimple drop-in replacement
        if (SimpleRotator::class.java.isAssignableFrom(device)) {
            val dms = hardwareMap.get(DcMotorSimple::class.java, name)
            return SimpleRotator(dms) as T
        }
        // InvertibleTouchSensor is a drop-in replacement for TouchSensor
        if (InvertibleTouchSensor::class.java.isAssignableFrom(device)) {
            val touchSensor = hardwareMap.get(TouchSensor::class.java, name)
            return InvertibleTouchSensor(touchSensor) as T
        }
        return hardwareMap.get(device, name)
    }

    /**
     * Convenience method for reading the device from the hardwareMap without having to check for exceptions.
     * This method can be passed a Runnable to run if the device is successfully configured, useful for setting up
     * directions or other configurations that will only run if the device was successfully found.
     *
     * Every failed device name with this method is saved to `Storage.memory().hardwareErrors`, which can be accessed during the
     * lifetime of the OpMode. These errors will also be reported to telemetry.
     *
     * @param name   name of device saved in the configuration file
     * @param device the class of the item to configure, `Motor.class`, `ServoEx.class`, `RawEncoder.class`, etc.
     * @param onSuccess a Runnable to run if the device is successfully configured, useful for setting up motor configs
     *                  without having to check for null explicitly.
     */
    @JvmOverloads
    protected fun <T> getHardware(
        name: String,
        device: Class<T>,
        onSuccess: Consumer<T> = Consumer { }
    ): T? {
        var hardwareDevice: T? = null
        var ok = false
        try {
            if (Storage.memory().hardwareErrors.contains(name)) return null
            hardwareDevice = getAndDynamicCast(name, device)
            // Paranoia check as custom device classes may not throw exceptions when not found
            if (hardwareDevice == null)
                throw NullPointerException()
            ok = true
        } catch (e: Exception) {
            Storage.memory().hardwareErrors.add(name)
            e.localizedMessage?.let { Dbg.warn(it) }
        }
        // Run the user callback if the device was successfully configured, outside the hardware device
        // catch block as exceptions raised in onSuccess will be mishandled. We also know that the device
        // is guaranteed to no longer be null. Errors raised in onSuccess() are caught by the init() handler in a BOM.
        if (ok)
            onSuccess.accept(hardwareDevice!!)
        return hardwareDevice
    }

    /**
     * Returns a [LazyImu] instance to use with RoadRunner drives. The difference between a regular IMU and a LazyImu
     * is that the LazyImu is auto-initialised only when it is required via the `get()` call.
     *
     * @param name the name of the IMU in HardwareMap
     * @param orientationOnRobot the IMU orientation on the robot
     */
    @JvmOverloads
    protected fun getLazyImu(name: String = "imu", orientationOnRobot: ImuOrientationOnRobot): LazyImu? {
        if (getHardware(name, IMU::class.java) == null) return null
        return LazyImu(hardwareMap, name, orientationOnRobot, 700)
    }
}
