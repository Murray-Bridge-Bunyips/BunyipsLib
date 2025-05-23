package au.edu.sa.mbhs.studentrobotics.bunyipslib

import au.edu.sa.mbhs.studentrobotics.bunyipslib.RobotConfig.AutoInit
import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.IMUEx
import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.InvertibleTouchSensor
import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.Motor
import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.ServoEx
import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.SimpleRotator
import au.edu.sa.mbhs.studentrobotics.bunyipslib.logic.Ramping
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dbg
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Exceptions
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Storage
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import dev.frozenmilk.sinister.loading.Preload
import dev.frozenmilk.sinister.sdk.apphooks.AppHookScanner
import dev.frozenmilk.sinister.staticInstancesOf
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
 *     config.init();
 * ```
 *
 * Alternatively as of v7.0.0, you can choose to use a singleton pattern where the [RobotConfig] will be initialised
 * just before any OpMode starts, via a [BunyipsLib] hook. See [AutoInit] for more details.
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
     *
     * Should be called as the first line in your init cycle if not using [AutoInit].
     * This method can only be executed once (further calls will no-op).
     *
     * As of BunyipsLib v7.0.0, passing the `opMode` parameter is no longer necessary as it will be retrieved
     * through a [BunyipsLib] utility.
     *
     * @return the instance of the RobotConfig
     */
    fun init(): RobotConfig {
        if (hasInitCalled) {
            Dbg.warn(RobotConfig::class.java, "instance (%) already initialised!", javaClass.simpleName)
            return this
        }
        if (globalInitCalled) {
            // Usually when two RobotConfig instances are initialised there has been a redefinition as there should
            // really only be one. We will warn and continue.
            Dbg.warn(
                RobotConfig::class.java,
                "a second RobotConfig instance (%) is being initialised when one has been initialised previously; please ensure this is intended behaviour!",
                javaClass.simpleName
            )
        }
        this.hardwareMap = BunyipsLib.opMode.hardwareMap
        Exceptions.runUserMethod(::onRuntime)
        DualTelemetry.smartAdd(
            "<b>${javaClass.simpleName}</b>",
            "Hardware initialised with ${if (Storage.memory().hardwareErrors.size > 0) "<font color='red'>${Storage.memory().hardwareErrors.size} error(s)</font>" else "<font color='green'>0 errors</font>"}."
        )
        Dbg.logd(javaClass, "hardware initialised with % error(s).", Storage.memory().hardwareErrors.size)
        for (error in Storage.memory().hardwareErrors) {
            DualTelemetry.smartAdd(true, error, "<font color='red'><b>MISSING DEVICE!</b></font>")
            DualTelemetry.smartLog("<font color='red'>error:</font> <i>$error</i> was not found in the current saved configuration.")
        }
        hasInitCalled = true
        globalInitCalled = true
        return this
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
        // It is also a lazy-loaded IMU compatible with RoadRunner through the LazyImu interface
        // Must be also initialised with orientation data by the user
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
     * @param onSuccess a Runnable to run if the device is successfully configured, useful for setting up configs
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
                throw NullPointerException("An error occurred configuring '$name' of type ${device.simpleName}.")
            ok = true
        } catch (e: Exception) {
            Storage.memory().hardwareErrors.add(name)
            e.localizedMessage?.let { Dbg.error(it) }
        }
        // Run the user callback if the device was successfully configured, outside the hardware device
        // catch block as exceptions raised in onSuccess will be mishandled. We also know that the device
        // is guaranteed to no longer be null. Errors raised in onSuccess() are caught by the init() handler in a BOM.
        if (ok)
            onSuccess.accept(hardwareDevice!!)
        return hardwareDevice
    }

    /**
     * Attaching this annotation to a [RobotConfig] derivative class will cause the [init] method to be executed
     * automatically on the initialisation of *any* OpMode.
     *
     * This is useful if you wish to make a "singleton" pattern for your [RobotConfig] class, where instantiating
     * and initialising this class can be handled automatically via a hook.
     *
     * **NOTE:** A singleton field on the classpath is required for your config, otherwise an instance can't be known to initialise.
     *
     * ```java
     * @RobotConfig.AutoInit // annotation attached
     * public class Robot extends RobotConfig {
     *     public static final Robot instance = new Robot(); // access your RobotConfig instance through a static field.
     *                                                       // a field exposing an instance like this *MUST* be present;
     *                                                       // the field name can be anything you choose and technically
     *                                                       // this field doesn't have to exist in this class.
     *                                                       // you can also use the kotlin `object` for a singleton.
     *
     *     @Override
     *     protected void onRuntime() {
     *       // will now be executed as a pre-init @Hook with access to OpMode information through BunyipsLib.getOpMode()
     *       // and the conventional BunyipsOpMode.getInstance() for BunyipsOpMode users
     *
     *       // calling init() manually will no-op now
     *     }
     * }
     * ```
     *
     * **WARNING:** Attaching `@Config` to your `RobotConfig` instance may cause the class to not show up on the dashboard,
     * as it will try to recursively add itself to the config. To combat this you will need to use inner static classes
     * or an entirely different class to hold your constants:
     * ```java
     * @RobotConfig.AutoInit
     * public class Robot extends RobotConfig {
     *     @Config
     *     public static class Constants {
     *         public static double SOME_CONSTANT = 1;
     *         // ...
     *     }
     *
     *     public static final Robot instance = new Robot();
     *     // ...
     * }
     * ```
     *
     * @since 7.0.0
     */
    @Preload
    @MustBeDocumented
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    annotation class AutoInit

    /**
     * Inhibits the effect of [AutoInit] on an [OpMode] derivative.
     * This is useful if you don't wish to initialise hardware automatically on this [OpMode].
     *
     * Manual [RobotConfig] calls will still function.
     *
     * Must be attached on the running [OpMode] to work.
     *
     * ```java
     * @TeleOp
     * @RobotConfig.InhibitAutoInit
     * public class MyMinimalTestOpMode extends BunyipsOpMode {
     *   // RobotConfig instance will *NOT* be automatically initialised when this OpMode is run.
     *   // Only has an impact if your RobotConfig is annotated with @AutoInit
     *   // ...
     * }
     * ```
     *
     * @since 7.0.0
     */
    @MustBeDocumented
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    annotation class InhibitAutoInit

    private object AutoInitScanner : AppHookScanner<RobotConfig>() {
        override val targets = BunyipsLib.StandardSearch()
        override fun scan(cls: Class<*>, registrationHelper: RegistrationHelper) {
            cls.staticInstancesOf(RobotConfig::class.java)
                .filter { cls.isAnnotationPresent(AutoInit::class.java) }
                .forEach { registrationHelper.register(it) }
        }
    }

    companion object {
        private var globalInitCalled = false

        @JvmStatic
        @Hook(on = Hook.Target.POST_STOP)
        private fun reset() {
            globalInitCalled = false
            AutoInitScanner.iterateAppHooks {
                it.hasInitCalled = false
            }
        }

        @JvmStatic
        @Hook(on = Hook.Target.PRE_INIT, priority = 2)
        private fun autoInit() {
            AutoInitScanner.iterateAppHooks {
                if (BunyipsLib.opMode.javaClass.isAnnotationPresent(InhibitAutoInit::class.java)) {
                    Dbg.log(
                        RobotConfig::class.java,
                        "auto-initialisation of % inhibited by `@RobotConfig.InhibitAutoInit`",
                        it.javaClass.simpleName
                    )
                    // Don't initialise
                    return@iterateAppHooks
                }
                Dbg.logd(RobotConfig::class.java, "auto-initialising: % ...", it.javaClass.simpleName)
                it.init()
            }
        }
    }
}
