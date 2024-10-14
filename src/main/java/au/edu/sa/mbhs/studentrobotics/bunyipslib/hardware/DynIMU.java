package au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliseconds;

import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.ImuInitMessage;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Dbg;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;

/**
 * Port of the {@link LazyImu} class to support the universal {@link IMU} interface by taking a reference to an
 * uninitialised {@link IMU} object and initialising it on the first call to a method.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public class DynIMU implements IMU {
    private final IMU imu;
    private final Parameters initParameters;
    private boolean triedInit;
    private double timeoutMs = 500;

    /**
     * Create a new object that will initialise the given {@link IMU} object when a read is required.
     *
     * @param uninitializedIMU the {@link IMU} instance that is <b>not yet initialised</b>.
     *                         Do not manually call {@link #initialize(Parameters)} on this IMU before passing it here.
     * @param initParameters   the parameters to use when initialising the IMU.
     */
    public DynIMU(IMU uninitializedIMU, Parameters initParameters) {
        this.initParameters = initParameters;
        imu = uninitializedIMU;
    }

    /**
     * Creates a DynIMU with no IMU attached. This is useful if you know for a fact that you will never need to use the
     * IMU, but still need to pass an IMU object to a method.
     *
     * @return a DynIMU object that will always return invalid zero data, do be cautious that accessing this object in
     * a method that would require the presence of an IMU will log warnings and potentially cause issues.
     */
    public static DynIMU none() {
        return new DynIMU(new IMU() {
            @Override
            public boolean initialize(Parameters parameters) {
                Dbg.warn(getClass(), "DynIMU.none() was accessed for initialisation.");
                return false;
            }

            @Override
            public void resetYaw() {
                Dbg.warn(getClass(), "DynIMU.none() was accessed for yaw reset.");
            }

            @Override
            public YawPitchRollAngles getRobotYawPitchRollAngles() {
                Dbg.warn(getClass(), "DynIMU.none() was accessed for yaw pitch roll angles.");
                return new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);
            }

            @Override
            public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
                Dbg.warn(getClass(), "DynIMU.none() was accessed for orientation.");
                return new Orientation();
            }

            @Override
            public Quaternion getRobotOrientationAsQuaternion() {
                Dbg.warn(getClass(), "DynIMU.none() was accessed for orientation as quaternion.");
                return new Quaternion();
            }

            @Override
            public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
                Dbg.warn(getClass(), "DynIMU.none() was accessed for angular velocity.");
                return new AngularVelocity();
            }

            @Override
            public Manufacturer getManufacturer() {
                Dbg.warn(getClass(), "DynIMU.none() was accessed for manufacturer.");
                return Manufacturer.Unknown;
            }

            @Override
            public String getDeviceName() {
                Dbg.warn(getClass(), "DynIMU.none() was accessed for device name.");
                return "DynIMU.none()";
            }

            @Override
            public String getConnectionInfo() {
                Dbg.warn(getClass(), "DynIMU.none() was accessed for connection info.");
                return "DynIMU.none()";
            }

            @Override
            public int getVersion() {
                Dbg.warn(getClass(), "DynIMU.none() was accessed for version.");
                return 0;
            }

            @Override
            public void resetDeviceConfigurationForOpMode() {
                Dbg.warn(getClass(), "DynIMU.none() was accessed for device configuration reset.");
            }

            @Override
            public void close() {
                Dbg.warn(getClass(), "DynIMU.none() was accessed for close.");
            }
        }, new Parameters(new RevHubOrientationOnRobot(Quaternion.identityQuaternion())));
    }

    /**
     * Set a custom timeout for the initialisation of the IMU that will fail the initialisation if it takes too long.
     *
     * @param timeout the timeout for the initialisation, default 500ms
     * @return this object for chaining.
     */
    public DynIMU withInitTimeout(Measure<Time> timeout) {
        timeoutMs = timeout.in(Milliseconds);
        return this;
    }

    private void tryInit() {
        if (triedInit) return;

        Dbg.log(getClass(), "Dynamic IMU initialisation in progress...");
        if (!imu.initialize(initParameters))
            RobotLog.addGlobalWarningMessage("DynIMU failed to initialise the IMU!");

        long start = System.nanoTime();
        ElapsedTime timer = new ElapsedTime();
        do {
            Quaternion q = imu.getRobotOrientationAsQuaternion();
            if (q.acquisitionTime != 0) {
                FlightRecorder.write("IMU_INIT", new ImuInitMessage(imu.getDeviceName(), start, System.nanoTime(), false));
                return;
            }
        } while (timer.milliseconds() < timeoutMs);

        FlightRecorder.write("IMU_INIT", new ImuInitMessage(imu.getDeviceName(), start, System.nanoTime(), true));
        RobotLog.addGlobalWarningMessage("DynIMU continues to return invalid data after " + timeoutMs + " ms!");

        triedInit = true;
    }

    @Override
    public boolean initialize(Parameters parameters) {
        // We can't really crash since this is still legal usage, but we should warn the user.
        Dbg.warn(getClass(), "Manual IMU initialisation was started on a DynIMU object. This is invalid usage!");
        return imu.initialize(parameters);
    }

    @Override
    public void resetYaw() {
        tryInit();
        imu.resetYaw();
    }

    @Override
    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        tryInit();
        return imu.getRobotYawPitchRollAngles();
    }

    @Override
    public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
        tryInit();
        return imu.getRobotOrientation(reference, order, angleUnit);
    }

    @Override
    public Quaternion getRobotOrientationAsQuaternion() {
        tryInit();
        return imu.getRobotOrientationAsQuaternion();
    }

    @Override
    public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
        tryInit();
        return imu.getRobotAngularVelocity(angleUnit);
    }

    @Override
    public Manufacturer getManufacturer() {
        return imu.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return imu.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return imu.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return imu.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        imu.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        imu.close();
    }
}
