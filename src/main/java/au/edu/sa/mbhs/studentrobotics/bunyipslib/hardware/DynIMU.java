package au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliseconds;

import androidx.annotation.NonNull;

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
    private final RevHubOrientationOnRobot orientation;
    private boolean triedInit;
    private double timeoutMs = 500;

    /**
     * Create a new object that will initialise the given {@link IMU} object when a read is required.
     *
     * @param uninitializedIMU the {@link IMU} instance that is <b>not yet initialised</b>.
     *                         Do not manually call {@link #initialize(Parameters)} on this IMU before passing it here.
     * @param orientation      the orientation of the IMU relative to the robot.
     */
    public DynIMU(@NonNull IMU uninitializedIMU, @NonNull RevHubOrientationOnRobot orientation) {
        this.orientation = orientation;
        imu = uninitializedIMU;
    }

    /**
     * Creates an IMU with no IMU attached. This is useful if you know for a fact that you will never need to use the
     * IMU, but still need to pass an IMU object to a method.
     *
     * @return an IMU object that will always return invalid zero data, do be cautious that accessing this object in
     * a method that would require the presence of an IMU will cause warnings and potentially cause issues.
     */
    @NonNull
    public static IMU none() {
        return new IMU() {
            private final boolean[] warned = new boolean[12];

            @Override
            public boolean initialize(Parameters parameters) {
                if (!warned[0]) Dbg.warn("DynIMU.none() was accessed for initialisation.");
                warned[0] = true;
                return false;
            }

            @Override
            public void resetYaw() {
                if (!warned[1]) Dbg.warn("DynIMU.none() was accessed for yaw reset.");
                warned[1] = true;
            }

            @Override
            public YawPitchRollAngles getRobotYawPitchRollAngles() {
                if (!warned[2]) Dbg.warn("DynIMU.none() was accessed for yaw pitch roll angles.");
                warned[2] = true;
                return new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);
            }

            @Override
            public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
                if (!warned[3]) Dbg.warn("DynIMU.none() was accessed for orientation.");
                warned[3] = true;
                return new Orientation();
            }

            @Override
            public Quaternion getRobotOrientationAsQuaternion() {
                if (!warned[4]) Dbg.warn("DynIMU.none() was accessed for orientation as quaternion.");
                warned[4] = true;
                return new Quaternion();
            }

            @Override
            public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
                if (!warned[5]) Dbg.warn("DynIMU.none() was accessed for angular velocity.");
                warned[5] = true;
                return new AngularVelocity();
            }

            @Override
            public Manufacturer getManufacturer() {
                if (!warned[6]) Dbg.warn("DynIMU.none() was accessed for manufacturer.");
                warned[6] = true;
                return Manufacturer.Unknown;
            }

            @Override
            public String getDeviceName() {
                if (!warned[7]) Dbg.warn("DynIMU.none() was accessed for device name.");
                warned[7] = true;
                return "DynIMU.none()";
            }

            @Override
            public String getConnectionInfo() {
                if (!warned[8]) Dbg.warn("DynIMU.none() was accessed for connection info.");
                warned[8] = true;
                return "DynIMU.none()";
            }

            @Override
            public int getVersion() {
                if (!warned[9]) Dbg.warn("DynIMU.none() was accessed for version.");
                warned[9] = true;
                return 0;
            }

            @Override
            public void resetDeviceConfigurationForOpMode() {
                if (!warned[10]) Dbg.warn("DynIMU.none() was accessed for device configuration reset.");
                warned[10] = true;
            }

            @Override
            public void close() {
                if (!warned[11]) Dbg.warn("DynIMU.none() was accessed for close.");
                warned[11] = true;
            }
        };
    }

    /**
     * Set a custom timeout for the initialisation of the IMU that will fail the initialisation if it takes too long.
     *
     * @param timeout the timeout for the initialisation, default 500ms
     * @return this object for chaining.
     */
    @NonNull
    public DynIMU withInitTimeout(@NonNull Measure<Time> timeout) {
        timeoutMs = timeout.in(Milliseconds);
        return this;
    }

    private void tryInit() {
        if (triedInit) return;
        triedInit = true;

        Dbg.logd("Dynamic IMU initialisation in progress...");
        if (!imu.initialize(new Parameters(orientation)))
            RobotLog.addGlobalWarningMessage("DynIMU failed to initialise the IMU!");

        long start = System.nanoTime();
        ElapsedTime timer = new ElapsedTime();
        do {
            Quaternion q = imu.getRobotOrientationAsQuaternion();
            if (q.acquisitionTime != 0) {
                FlightRecorder.write("IMU_INIT", new ImuInitMessage(imu.getDeviceName(), start, System.nanoTime(), false));
                Dbg.logv("IMU initialised.");
                return;
            }
        } while (timer.milliseconds() < timeoutMs);

        FlightRecorder.write("IMU_INIT", new ImuInitMessage(imu.getDeviceName(), start, System.nanoTime(), true));
        RobotLog.addGlobalWarningMessage("DynIMU continues to return invalid data after " + timeoutMs + " ms!");
    }

    @Override
    public boolean initialize(@NonNull Parameters parameters) {
        // We can't really crash since this is still legal usage, but we should warn the user.
        Dbg.warn("Manual IMU initialisation was started on a DynIMU object. This is invalid usage!");
        return imu.initialize(parameters);
    }

    @Override
    public void resetYaw() {
        tryInit();
        imu.resetYaw();
    }

    @NonNull
    @Override
    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        tryInit();
        return imu.getRobotYawPitchRollAngles();
    }

    @NonNull
    @Override
    public Orientation getRobotOrientation(@NonNull AxesReference reference, @NonNull AxesOrder order, @NonNull AngleUnit angleUnit) {
        tryInit();
        return imu.getRobotOrientation(reference, order, angleUnit);
    }

    @NonNull
    @Override
    public Quaternion getRobotOrientationAsQuaternion() {
        tryInit();
        return imu.getRobotOrientationAsQuaternion();
    }

    @NonNull
    @Override
    public AngularVelocity getRobotAngularVelocity(@NonNull AngleUnit angleUnit) {
        tryInit();
        return imu.getRobotAngularVelocity(angleUnit);
    }

    @NonNull
    @Override
    public Manufacturer getManufacturer() {
        return imu.getManufacturer();
    }

    @NonNull
    @Override
    public String getDeviceName() {
        return imu.getDeviceName();
    }

    @NonNull
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
