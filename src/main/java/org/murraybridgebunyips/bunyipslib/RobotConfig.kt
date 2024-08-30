package org.murraybridgebunyips.bunyipslib

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import org.murraybridgebunyips.bunyipslib.roadrunner.util.Deadwheel
import java.util.function.Consumer

/**
 * Abstract class to use as parent to the class you will define to mirror a "saved configuration"
 * on the Robot Controller, and to define any robot related constants.
 * Supported for use in both BunyipsOpModes and any other normal SDK OpMode.
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
    /**
     * OpMode supplied hardwareMap instance.
     */
    protected lateinit var hardwareMap: HardwareMap

    /**
     * Here you will assign class instance variables to public HardwareDevices, and assign any runtime-related
     * settings and variables for use. This is called from the `init` method.
     */
    protected abstract fun onRuntime()

    /**
     * Uses the HardwareMap to fetch HardwareDevices and assign instances from `onRuntime`.
     * Should be called as the first line in your init cycle.
     * @param opMode the OpMode instance - usually the `this` object when at the root OpMode.
     * @return the instance of the RobotConfig
     */
    @Suppress("UNCHECKED_CAST")
    fun <T : RobotConfig> init(opMode: OpMode): T {
        Storage.memory().hardwareErrors.clear()
        Storage.memory().unusableComponents.clear()
        this.hardwareMap = opMode.hardwareMap
        onRuntime()
        if (opMode is BunyipsOpMode) {
            opMode.t.add(
                "<b>${this.javaClass.simpleName}</b>: Completed with ${if (Storage.memory().hardwareErrors.size > 0) "<font color='red'>${Storage.memory().hardwareErrors.size} error(s)</font>" else "<font color='green'>0 errors</font>"}.",
            )
        } else {
            opMode.telemetry.addData(
                "",
                "${this.javaClass.simpleName}: Completed with ${Storage.memory().hardwareErrors.size} error(s).",
            )
        }
        for (error in Storage.memory().hardwareErrors) {
            if (opMode is BunyipsOpMode) {
                opMode.t.addRetained("<font color='red'><b>! MISSING_DEVICE</b></font>: $error")
                opMode.t.addRetained("<font color='red'>error:</font> <i>$error</i> was not found in the current saved configuration.")
            } else {
                opMode.telemetry.addData("", "! MISSING_DEVICE: $error").setRetained(true)
                opMode.telemetry.log().add("error: '$error' was not found in the current saved configuration.")
            }
        }
        return this as T
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
    fun <T : RobotConfig> init(): T {
        try {
            // Access the singleton associated with a BunyipsOpMode, if we're not running one Kotlin
            // will throw a UninitializedPropertyAccessException, so we can tell the user off here.
            return init(BunyipsOpMode.instance)
        } catch (e: UninitializedPropertyAccessException) {
            throw UnsupportedOperationException("Argument-less RobotConfig.init() method is only supported in a BunyipsOpMode. Use RobotConfig.init(opMode) instead.")
        }
    }

    private val dcMotorCastable = listOf(Deadwheel::class.java, Ramping.DcMotor::class.java, Motor::class.java)

    @Suppress("UNCHECKED_CAST")
    private fun <T : HardwareDevice> getAndDynamicCast(name: String, device: Class<T>): T? {
        // These devices are known to replicate DcMotor functionality, and we can auto-fetch and safely cast
        // to the new type for use by the user. We can ignore any unchecked cast warnings as these are checked.
        dcMotorCastable.forEach {
            if (it.isAssignableFrom(device)) {
                val motor = hardwareMap.get(DcMotor::class.java, name)
                return it.getConstructor(DcMotor::class.java).newInstance(motor) as T
            }
        }
        return hardwareMap.get(device, name)
    }

    /**
     * Convenience method for reading the device from the hardwareMap without having to check for exceptions.
     * This method can be passed a Runnable to run if the device is successfully configured, useful for setting up
     * directions or other configurations that will only run if the device was successfully found.
     * Every hardware error with this method be saved to a static array, which can be accessed during the
     * lifetime of the opMode.
     *
     * @param name   name of device saved in the configuration file
     * @param device the class of the item to configure, extending HardwareDevice (DcMotorEx.class, ServoImplEx.class, etc.)
     * @param onSuccess a Runnable to run if the device is successfully configured, useful for setting up motor configs
     *                  without having to check for null explicitly.
     */
    @JvmOverloads
    protected fun <T : HardwareDevice> getHardware(
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
        // Run the user callback if the device was successfully configured, outside of the hardware device
        // catch block as exceptions raised in onSuccess will be mishandled. We also know that the device
        // is guaranteed to no longer be null.
        if (ok)
            onSuccess.accept(hardwareDevice!!)
        return hardwareDevice
    }
}
