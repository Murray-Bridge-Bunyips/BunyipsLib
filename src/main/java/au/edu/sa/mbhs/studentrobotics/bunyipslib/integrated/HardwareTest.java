package au.edu.sa.mbhs.studentrobotics.bunyipslib.integrated;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.TouchSensorMultiplexer;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsOpMode;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.EmergencyStop;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.TelemetryMenu;

/**
 * Dynamic OpMode to test hardware functionality of all non-I2C {@link HardwareDevice} objects.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
@TeleOp(name = "Hardware Tester", group = "BunyipsLib")
@Disabled
public final class HardwareTest extends BunyipsOpMode {
    private TelemetryMenu menu;

    /** @noinspection ExtractMethodRecommender*/
    @Override
    protected void onInit() {
        Map<String, List<HardwareDevice>> hardware;
        hardwareMap.logDevices();
        try {
            Field allDevicesMapField = HardwareMap.class.getDeclaredField("allDevicesMap");
            allDevicesMapField.setAccessible(true);
            //noinspection unchecked
            hardware = (Map<String, List<HardwareDevice>>) allDevicesMapField.get(hardwareMap);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new EmergencyStop("Failed to access HardwareMap!");
        }
        if (hardware == null || hardware.isEmpty())
            throw new EmergencyStop("No hardware devices found!");

        TelemetryMenu.MenuElement root = new TelemetryMenu.MenuElement("Hardware Tester", true);
        for (Map.Entry<String, List<HardwareDevice>> entry : hardware.entrySet()) {
            // Map each hardwareMap name to a category and add it to the root menu
            TelemetryMenu.MenuElement deviceMapping = new TelemetryMenu.MenuElement(entry.getKey(), false);
            // Each device in the category may have multiple mappings, so we add them to the category
            for (HardwareDevice device : entry.getValue()) {
                TelemetryMenu.MenuElement dev = new TelemetryMenu.MenuElement(device.getDeviceName(), false);
                // For each device, we add hardware-specific information
                TelemetryMenu.StaticItem deviceType = new TelemetryMenu.StaticItem("Manufacturer: " + device.getManufacturer());
                dev.addChild(deviceType);

                if (device instanceof DcMotorSimple) {
                    DcMotorSimple motor = (DcMotorSimple) device;
                    // First, we map DcMotorSimple interfaces (including CRServos) to supply power and change direction
                    TelemetryMenu.InteractiveToggle powerControl = new TelemetryMenu.InteractiveToggle("Power", false, a -> {
                        motor.setPower(a ? -gamepad1.lsy : 0);
                        return motor.getPower();
                    }).resetIf(() -> gamepad1.b);
                    TelemetryMenu.InteractiveToggle directionControl = new TelemetryMenu.InteractiveToggle("Direction", false, a -> {
                        motor.setDirection(a ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
                        return motor.getDirection();
                    }).withColours("white", "white");
                    dev.addChildren(powerControl, directionControl);
                }

                if (device instanceof DcMotor) {
                    DcMotorEx motorEx = (DcMotorEx) device;
                    // We map all motors to the DcMotorEx interface to access more advanced motor functions
                    // Includes controls for enabling/disabling the motor, setting zero power behavior, and resetting the encoder
                    TelemetryMenu.InteractiveToggle enabledControl = new TelemetryMenu.InteractiveToggle("Enabled", true, a -> {
                        if (a)
                            motorEx.setMotorEnable();
                        else
                            motorEx.setMotorDisable();
                        return motorEx.isMotorEnabled();
                    }).withColours("green", "red");
                    TelemetryMenu.InteractiveToggle zeroPowerBehaviorControl = new TelemetryMenu.InteractiveToggle("Zero Power Behavior", false, a -> {
                        motorEx.setZeroPowerBehavior(a ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
                        return motorEx.getZeroPowerBehavior();
                    }).withColours("white", "white");
                    TelemetryMenu.StaticClickableOption resetEncoder = new TelemetryMenu.StaticClickableOption("Reset Encoder") {
                        @Override
                        protected void onClick() {
                            motorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        }
                    };

                    // Then, we can report out all the motor statistics
                    TelemetryMenu.DynamicItem currentPosition = new TelemetryMenu.DynamicItem("Current Position (t/s)", motorEx::getCurrentPosition);
                    TelemetryMenu.DynamicItem currentVelocity = new TelemetryMenu.DynamicItem("Current Velocity (t/s)", motorEx::getVelocity);
                    TelemetryMenu.DynamicItem currentCurrent = new TelemetryMenu.DynamicItem("Current (A)", () -> motorEx.getCurrent(CurrentUnit.AMPS));
                    dev.addChildren(enabledControl, zeroPowerBehaviorControl, resetEncoder, currentPosition, currentVelocity, currentCurrent);

                    // We can also add the motor's current configuration type in a new menu
                    TelemetryMenu.MenuElement motorConfig = new TelemetryMenu.MenuElement("Motor Information", false);
                    // These configuration constants are unlikely to change, so we just use static text
                    MotorConfigurationType motorType = motorEx.getMotorType();
                    TelemetryMenu.StaticItem distributor = new TelemetryMenu.StaticItem("Distributor: " + motorType.getDistributorInfo().getDistributor() + " (" + motorType.getDistributorInfo().getModel() + ")");
                    TelemetryMenu.StaticItem tpr = new TelemetryMenu.StaticItem("Ticks Per Revolution: " + motorType.getTicksPerRev());
                    TelemetryMenu.StaticItem gearing = new TelemetryMenu.StaticItem("Gearing: " + motorType.getGearing());
                    TelemetryMenu.StaticItem rpm = new TelemetryMenu.StaticItem("Max RPM: " + motorType.getMaxRPM());
                    TelemetryMenu.StaticItem tps = new TelemetryMenu.StaticItem("Max TPS: " + motorType.getAchieveableMaxTicksPerSecond());
                    TelemetryMenu.StaticItem orientation = new TelemetryMenu.StaticItem("Orientation: " + motorType.getOrientation());
                    TelemetryMenu.StaticItem veloPid = new TelemetryMenu.StaticItem("Velocity PID: " + motorType.getHubVelocityParams());
                    TelemetryMenu.StaticItem posPid = new TelemetryMenu.StaticItem("Position PID: " + motorType.getHubPositionParams());

                    motorConfig.addChildren(distributor, tpr, gearing, rpm, tps, orientation, veloPid, posPid);
                    dev.addChild(motorConfig);
                }

                if (device instanceof Servo) {
                    // Servo controls are simpler and only require directionality and position controls
                    ServoImplEx servo = (ServoImplEx) device;
                    TelemetryMenu.InteractiveToggle enabledControl = new TelemetryMenu.InteractiveToggle("Enabled", false, a -> {
                        if (a)
                            servo.setPwmEnable();
                        else
                            servo.setPwmDisable();
                        return servo.isPwmEnabled();
                    }).withColours("green", "red").resetIf(() -> gamepad1.b);
                    TelemetryMenu.InteractiveToggle positionControl = new TelemetryMenu.InteractiveToggle("Position", false, a -> {
                        double curr = servo.getPosition();
                        servo.setPosition(a ? Mathf.scale(-gamepad1.lsy, -1, 1, 0, 1) : curr);
                        return curr;
                    });
                    TelemetryMenu.InteractiveToggle directionControl = new TelemetryMenu.InteractiveToggle("Direction", false, a -> {
                        servo.setDirection(a ? Servo.Direction.FORWARD : Servo.Direction.REVERSE);
                        return servo.getDirection();
                    }).withColours("white", "white");
                    dev.addChildren(enabledControl, positionControl, directionControl);
                }

                if (device instanceof TouchSensorMultiplexer) {
                    // TouchSensorMultiplexers are simple and only require a list of touch sensor states
                    TouchSensorMultiplexer mux = (TouchSensorMultiplexer) device;
                    for (int i = 0; i < mux.getSwitches(); i++) {
                        int finalI = i;
                        TelemetryMenu.DynamicItem touchSensor = new TelemetryMenu.DynamicItem("Channel " + i,
                                () -> mux.isTouchSensorPressed(finalI));
                        dev.addChild(touchSensor);
                    }
                }

                if (device instanceof AnalogInput) {
                    // AnalogInputs can be represented by their voltage readings
                    AnalogInput analog = (AnalogInput) device;
                    TelemetryMenu.DynamicItem voltage = new TelemetryMenu.DynamicItem("Voltage (V)", analog::getVoltage);
                    TelemetryMenu.DynamicItem maxVoltage = new TelemetryMenu.DynamicItem("Max Voltage (V)", analog::getMaxVoltage);
                    dev.addChildren(voltage, maxVoltage);
                }

                if (device instanceof DigitalChannel) {
                    // DigitalChannels are represented by their state and input/output mode
                    DigitalChannel digital = (DigitalChannel) device;
                    TelemetryMenu.InteractiveToggle state = new TelemetryMenu.InteractiveToggle("State", false, a -> {
                        digital.setState(a);
                        return digital.getState();
                    }).withColours("green", "red");
                    TelemetryMenu.InteractiveToggle mode = new TelemetryMenu.InteractiveToggle("Mode", false, a -> {
                        digital.setMode(a ? DigitalChannel.Mode.INPUT : DigitalChannel.Mode.OUTPUT);
                        return digital.getMode();
                    }).withColours("white", "white");
                    dev.addChildren(state, mode);
                }

                // OpticalDistanceSensors are simply LightSensors so we will use the same interface
                if (device instanceof LightSensor) {
                    LightSensor light = (LightSensor) device;
                    TelemetryMenu.InteractiveToggle ledControl = new TelemetryMenu.InteractiveToggle("LED", false, a -> {
                        light.enableLed(a);
                        return a;
                    }).withColours("green", "red");
                    TelemetryMenu.DynamicItem lightLevel = new TelemetryMenu.DynamicItem("Light Level", light::getLightDetected);
                    TelemetryMenu.DynamicItem rawLightLevel = new TelemetryMenu.DynamicItem("Raw Light Level", light::getRawLightDetected);
                    TelemetryMenu.DynamicItem rawLightLevelMax = new TelemetryMenu.DynamicItem("Raw Light Level Max", light::getRawLightDetectedMax);
                    dev.addChildren(lightLevel, rawLightLevel, rawLightLevelMax);
                }

                if (device instanceof TouchSensor) {
                    // Touch sensors are only one state and are usually the most used digital sensor
                    TouchSensor touch = (TouchSensor) device;
                    TelemetryMenu.DynamicItem state = new TelemetryMenu.DynamicItem("Pressed", touch::isPressed);
                    TelemetryMenu.DynamicItem value = new TelemetryMenu.DynamicItem("Value", touch::getValue);
                    dev.addChildren(state, value);
                }

                if (device instanceof PWMOutput) {
                    // Adjusting PWM is a bit sketchy, but we can display the current PWM value here
                    PWMOutput pwm = (PWMOutput) device;
                    TelemetryMenu.DynamicItem pwmValue = new TelemetryMenu.DynamicItem("PWM Output Time", pwm::getPulseWidthOutputTime);
                    TelemetryMenu.DynamicItem pwmPeriod = new TelemetryMenu.DynamicItem("PWM Period", pwm::getPulseWidthPeriod);
                    dev.addChildren(pwmValue, pwmPeriod);
                }

                // Raw I2C devices are partially useless to us in the HardwareTest (values don't mean much), so we can skip them.
                // We'll also try to initialise the IMU with default parameters for testing purposes
                if (device instanceof IMU) {
                    IMU imu = (IMU) device;
                    // Assuming the Control Hub is just sitting on the robot naturally forward
                    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
                    TelemetryMenu.StaticItem notice = new TelemetryMenu.StaticItem("IMU is initialised assuming LOGO UP and USB BACKWARD");
                    TelemetryMenu.DynamicItem orientation = new TelemetryMenu.DynamicItem("Yaw Pitch Roll (deg)", imu::getRobotYawPitchRollAngles);
                    TelemetryMenu.DynamicItem angVel = new TelemetryMenu.DynamicItem("Angular Velocity (deg/s)", () -> imu.getRobotAngularVelocity(AngleUnit.DEGREES));
                    dev.addChildren(notice, orientation, angVel);
                }

                if (device instanceof DistanceSensor) {
                    // Distance sensors are simple and only require a distance reading
                    DistanceSensor distance = (DistanceSensor) device;
                    TelemetryMenu.DynamicItem distanceValue = new TelemetryMenu.DynamicItem("Distance (cm)", () -> distance.getDistance(DistanceUnit.CM));
                    dev.addChild(distanceValue);
                }

                if (device instanceof ColorSensor) {
                    // Display all the color sensor values
                    ColorSensor color = (ColorSensor) device;
                    TelemetryMenu.InteractiveToggle ledControl = new TelemetryMenu.InteractiveToggle("LED", false, a -> {
                        color.enableLed(a);
                        return a;
                    }).withColours("green", "red");
                    TelemetryMenu.DynamicItem red = new TelemetryMenu.DynamicItem("Red", color::red);
                    TelemetryMenu.DynamicItem green = new TelemetryMenu.DynamicItem("Green", color::green);
                    TelemetryMenu.DynamicItem blue = new TelemetryMenu.DynamicItem("Blue", color::blue);
                    TelemetryMenu.DynamicItem alpha = new TelemetryMenu.DynamicItem("Alpha", color::alpha);
                    dev.addChildren(ledControl, red, green, blue, alpha);
                }

                if (device instanceof LED) {
                    // LEDs are simple and only require a state
                    LED led = (LED) device;
                    TelemetryMenu.InteractiveToggle state = new TelemetryMenu.InteractiveToggle("State", false, a -> {
                        led.enable(a);
                        return led.isLightOn();
                    }).withColours("green", "red");
                    dev.addChild(state);
                }

                if (device instanceof AccelerationSensor) {
                    // Other sensors we just add their data anyways since they may be useful...
                    AccelerationSensor accel = (AccelerationSensor) device;
                    TelemetryMenu.DynamicItem status = new TelemetryMenu.DynamicItem("Status", accel::status);
                    TelemetryMenu.DynamicItem acceleration = new TelemetryMenu.DynamicItem("Acceleration (g)", accel::getAcceleration);
                    dev.addChildren(acceleration);
                }

                if (device instanceof CompassSensor) {
                    CompassSensor compass = (CompassSensor) device;
                    TelemetryMenu.DynamicItem status = new TelemetryMenu.DynamicItem("Status", compass::status);
                    TelemetryMenu.DynamicItem direction = new TelemetryMenu.DynamicItem("Direction (deg)", compass::getDirection);
                    TelemetryMenu.StaticClickableOption measurement = new TelemetryMenu.StaticClickableOption("Measurement Mode") {
                        @Override
                        protected void onClick() {
                            compass.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);
                        }
                    };
                    TelemetryMenu.StaticClickableOption calibration = new TelemetryMenu.StaticClickableOption("Calibration Mode") {
                        @Override
                        protected void onClick() {
                            compass.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);
                        }
                    };
                    TelemetryMenu.DynamicItem calibrationStatus = new TelemetryMenu.DynamicItem("Calibration Failed?", compass::calibrationFailed);
                    dev.addChildren(direction);
                }

                if (device instanceof GyroSensor) {
                    GyroSensor gyro = (GyroSensor) device;
                    // GyroSensor has warnings of exceptions so we'll just ignore them and return null if they show up
                    Function<Supplier<?>, ?> ignoreSupport = (r) -> {
                        Object res = null;
                        try {
                            res = r.get();
                        } catch (UnsupportedOperationException ignored) {
                            // no-op
                        }
                        return res;
                    };
                    TelemetryMenu.DynamicItem status = new TelemetryMenu.DynamicItem("Status", () -> ignoreSupport.apply(gyro::status));
                    TelemetryMenu.StaticClickableOption calibrate = new TelemetryMenu.StaticClickableOption("Calibrate") {
                        @Override
                        protected void onClick() {
                            try {
                                gyro.calibrate();
                            } catch (UnsupportedOperationException ignored) {
                                // no-op
                            }
                        }
                    };
                    TelemetryMenu.StaticClickableOption resetZIntegrator = new TelemetryMenu.StaticClickableOption("Reset Z Integrator") {
                        @Override
                        protected void onClick() {
                            try {
                                gyro.resetZAxisIntegrator();
                            } catch (UnsupportedOperationException ignored) {
                                // no-op
                            }
                        }
                    };
                    TelemetryMenu.DynamicItem calibrating = new TelemetryMenu.DynamicItem("Is Calibrating?", () -> ignoreSupport.apply(gyro::isCalibrating));
                    TelemetryMenu.DynamicItem heading = new TelemetryMenu.DynamicItem("Heading (deg)", () -> ignoreSupport.apply(gyro::getHeading));
                    TelemetryMenu.DynamicItem rotationFraction = new TelemetryMenu.DynamicItem("Rotation Fraction", () -> ignoreSupport.apply(gyro::getRotationFraction));
                    TelemetryMenu.DynamicItem rawX = new TelemetryMenu.DynamicItem("Raw X", () -> ignoreSupport.apply(gyro::rawX));
                    TelemetryMenu.DynamicItem rawY = new TelemetryMenu.DynamicItem("Raw Y", () -> ignoreSupport.apply(gyro::rawY));
                    TelemetryMenu.DynamicItem rawZ = new TelemetryMenu.DynamicItem("Raw Z", () -> ignoreSupport.apply(gyro::rawZ));
                    dev.addChildren(status, calibrate, resetZIntegrator, calibrating, heading, rotationFraction, rawX, rawY, rawZ);
                }

                if (device instanceof IrSeekerSensor) {
                    IrSeekerSensor ir = (IrSeekerSensor) device;
                    TelemetryMenu.DynamicItem signalStrength = new TelemetryMenu.DynamicItem("Signal Strength", ir::getStrength);
                    TelemetryMenu.DynamicItem signalAngle = new TelemetryMenu.DynamicItem("Signal Angle", ir::getAngle);
                    TelemetryMenu.DynamicItem signalDetectedThreshold = new TelemetryMenu.DynamicItem("Signal Detected Threshold", ir::getSignalDetectedThreshold);
                    TelemetryMenu.DynamicItem signalDetected = new TelemetryMenu.DynamicItem("Signal Detected", ir::signalDetected);
                    TelemetryMenu.InteractiveToggle mode = new TelemetryMenu.InteractiveToggle("Mode", false, a -> {
                        ir.setMode(a ? IrSeekerSensor.Mode.MODE_600HZ : IrSeekerSensor.Mode.MODE_1200HZ);
                        return ir.getMode();
                    }).withColours("white", "white");
                    // Menu for individual sensors
                    TelemetryMenu.MenuElement sensorMenu = new TelemetryMenu.MenuElement("Individual Sensors", false);
                    IrSeekerSensor.IrSeekerIndividualSensor[] individualSensors = ir.getIndividualSensors();
                    for (int i = 0; i < individualSensors.length; i++) {
                        int finalI = i;
                        TelemetryMenu.DynamicItem item = new TelemetryMenu.DynamicItem("#" + i, () -> individualSensors[finalI].toString());
                        sensorMenu.addChild(item);
                    }
                    dev.addChildren(signalStrength, signalAngle, signalDetectedThreshold, signalDetected, mode, sensorMenu);
                }

                if (device instanceof UltrasonicSensor) {
                    UltrasonicSensor ultra = (UltrasonicSensor) device;
                    TelemetryMenu.DynamicItem status = new TelemetryMenu.DynamicItem("Status", ultra::status);
                    TelemetryMenu.DynamicItem ultrasonicLevel = new TelemetryMenu.DynamicItem("Ultrasonic Level", ultra::getUltrasonicLevel);
                    dev.addChildren(status, ultrasonicLevel);
                }

                if (device instanceof VoltageSensor) {
                    VoltageSensor voltage = (VoltageSensor) device;
                    TelemetryMenu.DynamicItem voltageValue = new TelemetryMenu.DynamicItem("Voltage (V)", voltage::getVoltage);
                    dev.addChild(voltageValue);
                }

                // Finally, we add the device to the category
                deviceMapping.addChild(dev);
            }
            root.addChild(deviceMapping);
        }
        menu = new TelemetryMenu(telemetry, root);
    }

    @Override
    protected void activeLoop() {
        if (gamepad1.back)
            emergencyStop();
        menu.loop(gamepad1);
    }
}
