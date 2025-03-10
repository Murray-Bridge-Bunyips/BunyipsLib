package au.edu.sa.mbhs.studentrobotics.bunyipslib.integrated;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OrientationSensor;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.TouchSensorMultiplexer;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.function.Function;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.DualTelemetry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.executables.MovingAverageTimer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.TelemetryMenu;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Exceptions;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text;

/**
 * Dynamic OpMode to test hardware functionality of most {@link HardwareDevice} objects.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public final class HardwareTester extends LinearOpMode {
    /**
     * Array of motor powers and servo positions that can be controlled via FtcDashboard.
     * Must be enabled for each device that is willing to delegate to the dashboard.
     */
    public static double[] actuators = new double[0];
    private final List<DashboardControls> dashboardControlled = new ArrayList<>();
    private DualTelemetry telemetry;

    @Override
    @SuppressWarnings({"unchecked", "ExtractMethodRecommender"})
    public void runOpMode() {
        dashboardControlled.clear();
        actuators = new double[0];
        MovingAverageTimer timer = new MovingAverageTimer();
        telemetry = new DualTelemetry(super.telemetry, timer);
        telemetry.overheadTag = "<b>HardwareTester</b>";
        telemetry.init();
        Map<String, List<HardwareDevice>> hardware;
        hardwareMap.logDevices();
        try {
            Field allDevicesMapField = HardwareMap.class.getDeclaredField("allDevicesMap");
            allDevicesMapField.setAccessible(true);
            hardware = (Map<String, List<HardwareDevice>>) allDevicesMapField.get(hardwareMap);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new Exceptions.EmergencyStop("Failed to access HardwareMap!");
        }
        if (hardware == null || hardware.isEmpty())
            throw new Exceptions.EmergencyStop("No hardware devices found!");
        OpModes.INSTANCE.__setLightsDirty(true); // HardwareTester does not qualify as a user-OpMode but we modify the lights
        hardwareMap.getAll(LynxModule.class).forEach(h -> h.setPattern(Arrays.asList(
                new Blinker.Step(Color.GREEN.color, 400, TimeUnit.MILLISECONDS),
                new Blinker.Step(Color.LTGRAY.color, 400, TimeUnit.MILLISECONDS)
        )));
        TelemetryMenu.MenuElement root = new TelemetryMenu.MenuElement("Hardware Tester", true);
        for (Map.Entry<String, List<HardwareDevice>> entry : hardware.entrySet()) {
            // Map each hardwareMap name to a category and add it to the root menu
            TelemetryMenu.MenuElement deviceMapping = new TelemetryMenu.MenuElement(entry.getKey(), false);
            // Each device in the category may have multiple mappings, so we add them to the category
            for (HardwareDevice device : entry.getValue()) {
                TelemetryMenu.MenuElement dev = new TelemetryMenu.MenuElement(entry.getKey() + " (" + device.getDeviceName() + ", v" + device.getVersion() + ")", false);
                // For each device, we add hardware-specific information
                TelemetryMenu.StaticItem deviceType = new TelemetryMenu.StaticItem("Manufacturer: " + device.getManufacturer());
                TelemetryMenu.StaticItem connInfo = new TelemetryMenu.StaticItem("Connection Info: " + device.getConnectionInfo());
                dev.addChildren(deviceType, connInfo);

                if (device instanceof DcMotorSimple motor) {
                    DashboardControls dashboardControls = new DashboardControls(entry.getKey(), motor);
                    // First, we map DcMotorSimple interfaces (including CRServos) to supply power and change direction
                    TelemetryMenu.InteractiveToggle powerControl = new TelemetryMenu.InteractiveToggle("Power", false, a -> {
                        if (!dashboardControls.active)
                            motor.setPower(a ? -gamepad1.left_stick_y : 0);
                        return Mathf.round(motor.getPower(), 3);
                    }).resetIf(() -> gamepad1.b || dashboardControls.active);
                    // We can also supply a controller for the dashboard
                    TelemetryMenu.InteractiveToggle dashboardControl = new TelemetryMenu.InteractiveToggle("Use Dashboard", false, a -> {
                        if (a && dashboardControls.index == -1) {
                            double[] newActuators = new double[actuators.length + 1];
                            System.arraycopy(actuators, 0, newActuators, 0, actuators.length);
                            newActuators[actuators.length] = 0;
                            actuators = newActuators;
                            dashboardControls.index = actuators.length - 1;
                            Dashboard.enableConfig(getClass());
                        }
                        dashboardControls.active = a;
                        return a ? "active" : "inactive";
                    }).withColours("green", "white");
                    TelemetryMenu.InteractiveToggle directionControl = new TelemetryMenu.InteractiveToggle("Direction", false, a -> {
                        motor.setDirection(a ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
                        return motor.getDirection();
                    }).withColours("white", "white");
                    dashboardControlled.add(dashboardControls);
                    dev.addChildren(powerControl, dashboardControl, directionControl);

                    if (device instanceof DcMotorEx motorEx) {
                        // We map all motors to the DcMotorEx interface to access more advanced motor functions
                        // Includes controls for enabling/disabling the motor, setting zero power behavior, and resetting the encoder
                        TelemetryMenu.InteractiveToggle enabledControl = new TelemetryMenu.InteractiveToggle("Enabled", true, a -> {
                            if (a)
                                motorEx.setMotorEnable();
                            else
                                motorEx.setMotorDisable();
                            return motorEx.isMotorEnabled();
                        }).withColours("green", "red").resetIf(() -> dashboardControls.active);
                        TelemetryMenu.InteractiveToggle zeroPowerBehaviorControl = new TelemetryMenu.InteractiveToggle("Zero Power Behavior", false, a -> {
                            motorEx.setZeroPowerBehavior(a ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
                            return motorEx.getZeroPowerBehavior();
                        }).withColours("white", "white");
                        TelemetryMenu.StaticClickableOption resetEncoder = new TelemetryMenu.StaticClickableOption("Reset Encoder") {
                            @Override
                            protected void onClick() {
                                motorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            }
                        };

                        // Then, we can report out all the motor statistics
                        TelemetryMenu.DynamicItem currentPosition = new TelemetryMenu.DynamicItem("Current Position (t)", motorEx::getCurrentPosition);
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
                }

                if (device instanceof ServoImplEx servo) {
                    DashboardControls dashboardControls = new DashboardControls(entry.getKey(), servo);
                    // Servo controls are simpler and only require directionality and position controls
                    TelemetryMenu.InteractiveToggle enabledControl = new TelemetryMenu.InteractiveToggle("Enabled", false, a -> {
                        if (a)
                            servo.setPwmEnable();
                        else
                            servo.setPwmDisable();
                        return servo.isPwmEnabled();
                    }).withColours("green", "red").resetIf(() -> gamepad1.b).latchIf(() -> dashboardControls.active);
                    TelemetryMenu.InteractiveToggle positionControl = new TelemetryMenu.InteractiveToggle("Position", false, a -> {
                        double pos = a ? Mathf.scale(-gamepad1.left_stick_y, -1, 1, 0, 1) : servo.getPosition();
                        servo.setPosition(pos);
                        return Mathf.round(pos, 3);
                    }).resetIf(() -> gamepad1.b || dashboardControls.active);
                    TelemetryMenu.StaticClickableOption setToZero = new TelemetryMenu.StaticClickableOption("Set to 0", () -> servo.setPosition(0));
                    TelemetryMenu.StaticClickableOption setToOne = new TelemetryMenu.StaticClickableOption("Set to 1", () -> servo.setPosition(1));
                    TelemetryMenu.InteractiveToggle dashboardControl = new TelemetryMenu.InteractiveToggle("Use Dashboard", false, a -> {
                        if (a && dashboardControls.index == -1) {
                            double[] newActuators = new double[actuators.length + 1];
                            System.arraycopy(actuators, 0, newActuators, 0, actuators.length);
                            newActuators[actuators.length] = servo.getPosition();
                            actuators = newActuators;
                            dashboardControls.index = actuators.length - 1;
                            Dashboard.enableConfig(getClass());
                        }
                        dashboardControls.active = a;
                        return a ? "active" : "inactive";
                    }).withColours("green", "white");
                    TelemetryMenu.InteractiveToggle directionControl = new TelemetryMenu.InteractiveToggle("Direction", false, a -> {
                        servo.setDirection(a ? Servo.Direction.FORWARD : Servo.Direction.REVERSE);
                        return servo.getDirection();
                    }).withColours("white", "white");
                    dashboardControlled.add(dashboardControls);
                    dev.addChildren(enabledControl, positionControl, setToZero, setToOne, dashboardControl, directionControl);
                }

                if (device instanceof TouchSensorMultiplexer mux) {
                    // TouchSensorMultiplexers are simple and only require a list of touch sensor states
                    for (int i = 0; i < mux.getSwitches(); i++) {
                        int finalI = i;
                        TelemetryMenu.DynamicItem touchSensor = new TelemetryMenu.DynamicItem("Channel " + i,
                                () -> mux.isTouchSensorPressed(finalI));
                        dev.addChild(touchSensor);
                    }
                }

                if (device instanceof AnalogInput analog) {
                    // AnalogInputs can be represented by their voltage readings
                    TelemetryMenu.DynamicItem voltage = new TelemetryMenu.DynamicItem("Voltage (V)", analog::getVoltage);
                    TelemetryMenu.DynamicItem maxVoltage = new TelemetryMenu.DynamicItem("Max Voltage (V)", analog::getMaxVoltage);
                    dev.addChildren(voltage, maxVoltage);
                }

                if (device instanceof DigitalChannel digital) {
                    // DigitalChannels are represented by their state and input/output mode
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
                if (device instanceof LightSensor light) {
                    TelemetryMenu.InteractiveToggle ledControl = new TelemetryMenu.InteractiveToggle("LED", false, a -> {
                        light.enableLed(a);
                        return a;
                    }).withColours("green", "red");
                    TelemetryMenu.DynamicItem lightLevel = new TelemetryMenu.DynamicItem("Light Level", light::getLightDetected);
                    TelemetryMenu.DynamicItem rawLightLevel = new TelemetryMenu.DynamicItem("Raw Light Level", light::getRawLightDetected);
                    TelemetryMenu.DynamicItem rawLightLevelMax = new TelemetryMenu.DynamicItem("Raw Light Level Max", light::getRawLightDetectedMax);
                    dev.addChildren(ledControl, lightLevel, rawLightLevel, rawLightLevelMax);
                }

                if (device instanceof TouchSensor touch) {
                    // Touch sensors are only one state and are usually the most used digital sensor
                    TelemetryMenu.DynamicItem state = new TelemetryMenu.DynamicItem("Pressed", touch::isPressed);
                    TelemetryMenu.DynamicItem value = new TelemetryMenu.DynamicItem("Value", touch::getValue);
                    dev.addChildren(state, value);
                }

                if (device instanceof PWMOutput pwm) {
                    // Adjusting PWM is a bit sketchy, but we can display the current PWM value here
                    TelemetryMenu.DynamicItem pwmValue = new TelemetryMenu.DynamicItem("PWM Output Time", pwm::getPulseWidthOutputTime);
                    TelemetryMenu.DynamicItem pwmPeriod = new TelemetryMenu.DynamicItem("PWM Period", pwm::getPulseWidthPeriod);
                    dev.addChildren(pwmValue, pwmPeriod);
                }

                // Raw I2C devices are partially useless to us in the HardwareTest (values don't mean much), so we can skip them.
                // We'll also try to initialise the IMU with default parameters for testing purposes
                if (device instanceof IMU imu) {
                    // Assuming the Control Hub is just sitting on the robot naturally forward
                    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
                    TelemetryMenu.StaticItem notice = new TelemetryMenu.StaticItem("IMU is initialised assuming LOGO UP and USB BACKWARD");
                    TelemetryMenu.DynamicItem orientation = new TelemetryMenu.DynamicItem("Yaw Pitch Roll (deg)", imu::getRobotYawPitchRollAngles);
                    TelemetryMenu.DynamicItem angVel = new TelemetryMenu.DynamicItem("Angular Velocity (deg/s)", () -> imu.getRobotAngularVelocity(AngleUnit.DEGREES));
                    dev.addChildren(notice, orientation, angVel);
                }

                // We can also initialise the BNO055IMU type since it has extra data compared to simply the universal IMU
                if (device instanceof BNO055IMU imu) {
                    // We can initialise the IMU with default parameters for testing purposes
                    BNO055IMU.Parameters params = new BNO055IMU.Parameters();
                    params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                    imu.initialize(params);
                    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

                    TelemetryMenu.StaticItem notice = new TelemetryMenu.StaticItem("IMU is initialised with default parameters (C, deg, ms/s/s)");
                    TelemetryMenu.DynamicItem systemStatus = new TelemetryMenu.DynamicItem("System Status", imu::getSystemStatus);
                    TelemetryMenu.DynamicItem calibStatus = new TelemetryMenu.DynamicItem("Calibration Status", imu::getCalibrationStatus);
                    TelemetryMenu.DynamicItem error = new TelemetryMenu.DynamicItem("System Error", imu::getSystemError);
                    TelemetryMenu.DynamicItem isSystemCalibrated = new TelemetryMenu.DynamicItem("System Calibrated?", imu::isSystemCalibrated);
                    TelemetryMenu.DynamicItem isGyroCalibrated = new TelemetryMenu.DynamicItem("Gyro Calibrated?", imu::isGyroCalibrated);
                    TelemetryMenu.DynamicItem isAccelerometerCalibrated = new TelemetryMenu.DynamicItem("Accelerometer Calibrated?", imu::isAccelerometerCalibrated);
                    TelemetryMenu.DynamicItem isMagnetometerCalibrated = new TelemetryMenu.DynamicItem("Magnetometer Calibrated?", imu::isMagnetometerCalibrated);
                    TelemetryMenu.DynamicItem angularOrientation = new TelemetryMenu.DynamicItem("Angular Orientation", imu::getAngularOrientation);
                    TelemetryMenu.DynamicItem position = new TelemetryMenu.DynamicItem("Position", imu::getPosition);
                    TelemetryMenu.DynamicItem velocity = new TelemetryMenu.DynamicItem("Velocity", imu::getVelocity);
                    TelemetryMenu.DynamicItem acceleration = new TelemetryMenu.DynamicItem("Acceleration", imu::getAcceleration);
                    TelemetryMenu.DynamicItem overallAcceleration = new TelemetryMenu.DynamicItem("Overall Acceleration", imu::getOverallAcceleration);
                    TelemetryMenu.DynamicItem linearAcceleration = new TelemetryMenu.DynamicItem("Linear Acceleration", imu::getLinearAcceleration);
                    TelemetryMenu.DynamicItem gravity = new TelemetryMenu.DynamicItem("Gravity", imu::getGravity);
                    TelemetryMenu.DynamicItem angVel = new TelemetryMenu.DynamicItem("Angular Velocity", imu::getAngularVelocity);
                    TelemetryMenu.DynamicItem temperature = new TelemetryMenu.DynamicItem("Temperature", () -> imu.getTemperature().temperature);
                    TelemetryMenu.DynamicItem magneticFlux = new TelemetryMenu.DynamicItem("Magnetic Flux", imu::getMagneticFieldStrength);
                    dev.addChildren(notice, systemStatus, calibStatus, error, isSystemCalibrated, isGyroCalibrated, isAccelerometerCalibrated, isMagnetometerCalibrated, angularOrientation, position, velocity, acceleration, overallAcceleration, linearAcceleration, gravity, angVel, temperature, magneticFlux);
                }

                if (device instanceof LynxModule lynx) {
                    // Control and Expansion Hubs
                    TelemetryMenu.DynamicItem firmwareVersion = new TelemetryMenu.DynamicItem("Firmware Version", lynx::getFirmwareVersionString);
                    TelemetryMenu.DynamicItem current = new TelemetryMenu.DynamicItem("Current (A)", () -> lynx.getCurrent(CurrentUnit.AMPS));
                    TelemetryMenu.DynamicItem gpioBusCurrent = new TelemetryMenu.DynamicItem("GPIO Bus Current (A)", () -> lynx.getGpioBusCurrent(CurrentUnit.AMPS));
                    TelemetryMenu.DynamicItem i2cBusCurrent = new TelemetryMenu.DynamicItem("I2C Bus Current (A)", () -> lynx.getI2cBusCurrent(CurrentUnit.AMPS));
                    TelemetryMenu.DynamicItem inputVoltage = new TelemetryMenu.DynamicItem("Input Voltage (V)", () -> lynx.getInputVoltage(VoltageUnit.VOLTS));
                    TelemetryMenu.DynamicItem auxVoltage = new TelemetryMenu.DynamicItem("Auxiliary Voltage (V)", () -> lynx.getAuxiliaryVoltage(VoltageUnit.VOLTS));
                    TelemetryMenu.DynamicItem temperature = new TelemetryMenu.DynamicItem("Temperature (C)", () -> lynx.getTemperature(TempUnit.CELSIUS));
                    TelemetryMenu.EnumOption ledPattern = new TelemetryMenu.EnumOption("LED Pattern", Color.values());
                    TelemetryMenu.StaticClickableOption applyPattern = new TelemetryMenu.StaticClickableOption("Apply Pattern", () -> {
                        Color color = (Color) ledPattern.getValue();
                        lynx.setConstant(color.color);
                    });
                    TelemetryMenu.StaticClickableOption reset = new TelemetryMenu.StaticClickableOption("Reset",
                            () -> lynx.setPattern(LynxModule.blinkerPolicy.getIdlePattern(lynx)));
                    dev.addChildren(firmwareVersion, current, gpioBusCurrent, i2cBusCurrent, inputVoltage, auxVoltage, temperature, ledPattern, applyPattern, reset);
                }

                if (device instanceof DistanceSensor distance) {
                    // Distance sensors are simple and only require a distance reading
                    TelemetryMenu.DynamicItem distanceValue = new TelemetryMenu.DynamicItem("Distance (cm)", () -> distance.getDistance(DistanceUnit.CM));
                    dev.addChild(distanceValue);
                }

                if (device instanceof ColorSensor color) {
                    // Display all the color sensor values
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

                if (device instanceof LED led) {
                    // LEDs are simple and only require a state
                    TelemetryMenu.InteractiveToggle state = new TelemetryMenu.InteractiveToggle("State", false, a -> {
                        led.enable(a);
                        return led.isLightOn();
                    }).withColours("green", "red");
                    dev.addChild(state);
                }

                if (device instanceof RevBlinkinLedDriver blinkin) {
                    // We can also control Blinkin devices
                    TelemetryMenu.StaticClickableOption off = new TelemetryMenu.StaticClickableOption("Turn Off", () -> blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK));
                    TelemetryMenu.StaticClickableOption white = new TelemetryMenu.StaticClickableOption("To White", () -> blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE));
                    TelemetryMenu.EnumOption pattern = new TelemetryMenu.EnumOption("Pattern Select", RevBlinkinLedDriver.BlinkinPattern.values());
                    TelemetryMenu.StaticClickableOption applicator = new TelemetryMenu.StaticClickableOption("Apply Pattern", () -> blinkin.setPattern((RevBlinkinLedDriver.BlinkinPattern) pattern.getValue()));
                    dev.addChildren(off, white, pattern, applicator);
                }

                if (device instanceof AccelerationSensor accel) {
                    // Other sensors we just add their data anyway since they may be useful...
                    TelemetryMenu.DynamicItem status = new TelemetryMenu.DynamicItem("Status", accel::status);
                    TelemetryMenu.DynamicItem acceleration = new TelemetryMenu.DynamicItem("Acceleration (g)", accel::getAcceleration);
                    dev.addChildren(status, acceleration);
                }

                if (device instanceof CompassSensor compass) {
                    TelemetryMenu.DynamicItem status = new TelemetryMenu.DynamicItem("Status", compass::status);
                    TelemetryMenu.DynamicItem direction = new TelemetryMenu.DynamicItem("Direction (deg)", compass::getDirection);
                    TelemetryMenu.StaticClickableOption measurement = new TelemetryMenu.StaticClickableOption("Measurement Mode", () -> compass.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE));
                    TelemetryMenu.StaticClickableOption calibration = new TelemetryMenu.StaticClickableOption("Calibration Mode", () -> compass.setMode(CompassSensor.CompassMode.CALIBRATION_MODE));
                    TelemetryMenu.DynamicItem calibrationStatus = new TelemetryMenu.DynamicItem("Calibration Failed?", compass::calibrationFailed);
                    dev.addChildren(status, direction, measurement, calibration, calibrationStatus);
                }

                if (device instanceof GyroSensor gyro) {
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
                    TelemetryMenu.StaticClickableOption calibrate = new TelemetryMenu.StaticClickableOption("Calibrate", () -> {
                        try {
                            gyro.calibrate();
                        } catch (UnsupportedOperationException ignored) {
                            // no-op
                        }
                    });
                    TelemetryMenu.StaticClickableOption resetZIntegrator = new TelemetryMenu.StaticClickableOption("Reset Z Integrator", () -> {
                        try {
                            gyro.resetZAxisIntegrator();
                        } catch (UnsupportedOperationException ignored) {
                            // no-op
                        }
                    });
                    TelemetryMenu.DynamicItem calibrating = new TelemetryMenu.DynamicItem("Is Calibrating?", () -> ignoreSupport.apply(gyro::isCalibrating));
                    TelemetryMenu.DynamicItem heading = new TelemetryMenu.DynamicItem("Heading (deg)", () -> ignoreSupport.apply(gyro::getHeading));
                    TelemetryMenu.DynamicItem rotationFraction = new TelemetryMenu.DynamicItem("Rotation Fraction", () -> ignoreSupport.apply(gyro::getRotationFraction));
                    TelemetryMenu.DynamicItem rawX = new TelemetryMenu.DynamicItem("Raw X", () -> ignoreSupport.apply(gyro::rawX));
                    TelemetryMenu.DynamicItem rawY = new TelemetryMenu.DynamicItem("Raw Y", () -> ignoreSupport.apply(gyro::rawY));
                    TelemetryMenu.DynamicItem rawZ = new TelemetryMenu.DynamicItem("Raw Z", () -> ignoreSupport.apply(gyro::rawZ));
                    dev.addChildren(status, calibrate, resetZIntegrator, calibrating, heading, rotationFraction, rawX, rawY, rawZ);
                }

                if (device instanceof IrSeekerSensor ir) {
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

                if (device instanceof UltrasonicSensor ultra) {
                    TelemetryMenu.DynamicItem status = new TelemetryMenu.DynamicItem("Status", ultra::status);
                    TelemetryMenu.DynamicItem ultrasonicLevel = new TelemetryMenu.DynamicItem("Ultrasonic Level", ultra::getUltrasonicLevel);
                    dev.addChildren(status, ultrasonicLevel);
                }

                if (device instanceof VoltageSensor voltage) {
                    TelemetryMenu.DynamicItem voltageValue = new TelemetryMenu.DynamicItem("Voltage (V)", voltage::getVoltage);
                    dev.addChild(voltageValue);
                }

                // We're also able to add some info based on the SparkFun OTOS, since we can actually parse the I2C
                // without doing some magic
                if (device instanceof SparkFunOTOS otos) {
                    TelemetryMenu.MenuElement warnings = new TelemetryMenu.MenuElement("Warnings", false);
                    TelemetryMenu.DynamicItem warnTilt = new TelemetryMenu.DynamicItem("Tilt Warning", () -> otos.getStatus().warnTiltAngle);
                    TelemetryMenu.DynamicItem warnTrack = new TelemetryMenu.DynamicItem("Tracking Warning", () -> otos.getStatus().warnOpticalTracking);
                    TelemetryMenu.DynamicItem errPaa = new TelemetryMenu.DynamicItem("PAA Error", () -> otos.getStatus().errorPaa);
                    TelemetryMenu.DynamicItem errLsm = new TelemetryMenu.DynamicItem("LSM Error", () -> otos.getStatus().errorLsm);
                    warnings.addChildren(warnTilt, warnTrack, errPaa, errLsm);
                    TelemetryMenu.DynamicItem linearScalar = new TelemetryMenu.DynamicItem("Linear Scalar", otos::getLinearScalar);
                    TelemetryMenu.DynamicItem angularScalar = new TelemetryMenu.DynamicItem("Angular Scalar", otos::getAngularScalar);
                    TelemetryMenu.StaticClickableOption selfTest = new TelemetryMenu.StaticClickableOption("Run Self Test") {
                        @Override
                        protected void onClick() {
                            telemetry.log("OTOS Self Test: %", otos.selfTest() ? "PASS" : "FAIL");
                        }
                    };
                    TelemetryMenu.StaticClickableOption calibrateIMU = new TelemetryMenu.StaticClickableOption("Calibrate IMU") {
                        @Override
                        protected void onClick() {
                            otos.calibrateImu(255, false);
                        }
                    };
                    TelemetryMenu.DynamicItem imuCalibrationStatus = new TelemetryMenu.DynamicItem("IMU Calibration Samples Remaining", otos::getImuCalibrationProgress);
                    TelemetryMenu.StaticClickableOption resetTracking = new TelemetryMenu.StaticClickableOption("Reset Tracking") {
                        @Override
                        protected void onClick() {
                            otos.resetTracking();
                        }
                    };
                    TelemetryMenu.DynamicItem posX = new TelemetryMenu.DynamicItem("Position X", () -> otos.getPosition().x);
                    TelemetryMenu.DynamicItem posY = new TelemetryMenu.DynamicItem("Position Y", () -> otos.getPosition().y);
                    TelemetryMenu.DynamicItem posTheta = new TelemetryMenu.DynamicItem("Position H", () -> otos.getPosition().h);
                    TelemetryMenu.DynamicItem velX = new TelemetryMenu.DynamicItem("Velocity X", () -> otos.getVelocity().x);
                    TelemetryMenu.DynamicItem velY = new TelemetryMenu.DynamicItem("Velocity Y", () -> otos.getVelocity().y);
                    TelemetryMenu.DynamicItem velTheta = new TelemetryMenu.DynamicItem("Velocity H", () -> otos.getVelocity().h);
                    TelemetryMenu.DynamicItem accX = new TelemetryMenu.DynamicItem("Acceleration X", () -> otos.getAcceleration().x);
                    TelemetryMenu.DynamicItem accY = new TelemetryMenu.DynamicItem("Acceleration Y", () -> otos.getAcceleration().y);
                    TelemetryMenu.DynamicItem accTheta = new TelemetryMenu.DynamicItem("Acceleration H", () -> otos.getAcceleration().h);
                    dev.addChildren(warnings, linearScalar, angularScalar, selfTest, calibrateIMU, imuCalibrationStatus, resetTracking, posX, posY, posTheta, velX, velY, velTheta, accX, accY, accTheta);
                }

                if (device instanceof OctoQuad octo) {
                    TelemetryMenu.DynamicItem position = new TelemetryMenu.DynamicItem("Positions", () -> Arrays.toString(octo.readAllPositions()));
                    TelemetryMenu.DynamicItem velocity = new TelemetryMenu.DynamicItem("Velocities", () -> Arrays.toString(octo.readAllVelocities()));
                    dev.addChildren(position, velocity);
                }

                if (device instanceof Gyroscope gyro) {
                    TelemetryMenu.DynamicItem angularVelocity = new TelemetryMenu.DynamicItem("Angular Velocity (deg/s)", () -> gyro.getAngularVelocity(AngleUnit.DEGREES));
                    dev.addChild(angularVelocity);
                }

                if (device instanceof OrientationSensor sensor) {
                    TelemetryMenu.DynamicItem ori = new TelemetryMenu.DynamicItem("Orientation (ext, XYZ, deg)", () -> sensor.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES));
                    dev.addChild(ori);
                }

                // Finally, we add the device to the category.
                // We skip vision devices as they are better tested independently
                // We also skip other devices (mostly I2C) since their testing requirement is fairly niche and has to be
                // manually implemented, manual tests can be written if required.
                deviceMapping.addChild(dev);
            }
            root.addChild(deviceMapping);
        }
        TelemetryMenu menu = new TelemetryMenu(telemetry, root);
        telemetry.clearAll();
        telemetry.opModeStatus = "<font color='green'>ready</font>";

        while (opModeInInit()) {
            timer.update();
            telemetry.update();
        }

        timer.reset();

        while (opModeIsActive()) {
            if (gamepad1.back)
                terminateOpModeNow();
            menu.loop(gamepad1);
            for (DashboardControls dc : dashboardControlled) {
                if (dc.index == -1)
                    continue;
                telemetry.addDashboard("<font face='monospace'>HardwareTester/actuators</font> index <b>" + dc.index + "</b>", dc.active ? Text.format("<b>%</b> (%)", dc.name, dc.device) : "inactive");
                if (!dc.active)
                    continue;
                if (dc.device instanceof DcMotorSimple dms) {
                    dms.setPower(actuators[dc.index]);
                } else if (dc.device instanceof ServoImplEx servo) {
                    servo.setPosition(actuators[dc.index]);
                }
            }
            timer.update();
            telemetry.update();
        }
    }

    private enum Color {
        BLACK(0xFF000000),
        DKGRAY(0xFF444444),
        GRAY(0xFF888888),
        LTGRAY(0xFFCCCCCC),
        WHITE(0xFFFFFFFF),
        RED(0xFFFF0000),
        GREEN(0xFF00FF00),
        BLUE(0xFF0000FF),
        YELLOW(0xFFFFFF00),
        CYAN(0xFF00FFFF),
        MAGENTA(0xFFFF00FF);

        public final int color;

        Color(int color) {
            this.color = color;
        }
    }

    private static class DashboardControls {
        public final HardwareDevice device;
        public final String name;
        public int index = -1;
        public boolean active = false;

        public DashboardControls(String name, HardwareDevice device) {
            this.name = name;
            this.device = device;
        }
    }
}
