package au.edu.sa.mbhs.studentrobotics.bunyipslib.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.function.Consumer;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsOpMode;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.DualTelemetry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Hook;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.accumulators.Accumulator;

/**
 * Set of helper functions for drawing on the FtcDashboard canvas and operating various FtcDashboard functions.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
@Config
public final class Dashboard {
    private static final HashMap<String, Supplier<String>> observations = new HashMap<>();
    /**
     * The radius of the robot for drawing in inches.
     */
    public static double ROBOT_RADIUS = 9;
    /**
     * Enable to sync all packet operations to send a single packet at the time of {@link #sendAndClearSyncedPackets()}
     * being called. This is not required for usages of {@link DualTelemetry}, since synchronisation happens internally,
     * however, for custom implementations where various packets are sent, this serves as a useful option.
     * <p>
     * Packet syncing will only occur for usages of {@link #usePacket(Consumer)}. Do note that packets will not
     * be sent automatically if this is enabled, {@link #sendAndClearSyncedPackets()} must be called manually.
     */
    public static boolean USING_SYNCED_PACKETS = false;
    private static volatile TelemetryPacket accumulatedPacket = new TelemetryPacket();

    private Dashboard() {
        throw new AssertionError("This is a utility class");
    }

    @Hook(on = Hook.Target.POST_STOP)
    private static void reset() {
        USING_SYNCED_PACKETS = false;
        accumulatedPacket = new TelemetryPacket();
    }

    /**
     * Draw segments tracing out a pose history.
     *
     * @param canvas      dashboard canvas
     * @param poseHistory list of robot poses
     */
    public static void drawPoseHistory(@NonNull Canvas canvas, @NonNull List<Pose2d> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        canvas.strokePolyline(xPoints, yPoints);
    }

    /**
     * Draw a circle representing the robot and heading on the field.
     *
     * @param canvas dashboard canvas
     * @param pose   robot pose
     */
    public static void drawRobot(@NonNull Canvas canvas, @NonNull Pose2d pose) {
        canvas.setStrokeWidth(1);
        canvas.strokeCircle(pose.position.x, pose.position.y, ROBOT_RADIUS);

        Vector2d halfv = pose.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = pose.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        canvas.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }

    /**
     * Send a class to the dashboard for configuration, much as if the class was annotated with {@link Config}.
     *
     * @param clazz the class to send
     */
    public static void enableConfig(@NonNull Class<?> clazz) {
        FtcDashboard dash = FtcDashboard.getInstance();
        if (dash != null)
            dash.withConfigRoot(c ->
                    c.putVariable(clazz.getSimpleName(), ReflectionConfig.createVariableFromClass(clazz)));
    }

    /**
     * Merges two {@link TelemetryPacket} instances.
     *
     * @param a the first (leading) packet, this will be used as the parent
     * @param b the second packet, this packet will be deconstructed and merged into the leading packet
     * @return a merged packet leading with the first packet
     * @since 7.5.0
     */
    @SuppressWarnings("unchecked")
    public static TelemetryPacket mergePackets(TelemetryPacket a, TelemetryPacket b) {
        // Access all data fields from each packet, not including the field itself
        a.fieldOverlay().getOperations().addAll(b.fieldOverlay().getOperations());
        try {
            Field dataField = a.getClass().getDeclaredField("data");
            Field logField = a.getClass().getDeclaredField("log");
            dataField.setAccessible(true);
            logField.setAccessible(true);
            SortedMap<String, String> dataDst = (SortedMap<String, String>) dataField.get(a);
            List<String> logDst = (List<String>) logField.get(a);
            SortedMap<String, String> dataSrc = (SortedMap<String, String>) dataField.get(b);
            List<String> logSrc = (List<String>) logField.get(b);
            // Merge them all together
            assert dataDst != null && dataSrc != null;
            dataDst.putAll(dataSrc);
            dataField.set(a, dataDst);
            assert logDst != null && logSrc != null;
            logDst.addAll(logSrc);
            logField.set(a, logDst);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new RuntimeException("Failed to access internal fields, this shouldn't happen!", e);
        }
        return a;
    }

    /**
     * Extracts the log schema from {@link BunyipsSubsystem} instances to attach to an FtcDashboard packet, allowing live view
     * of data being saved to the log files. This is useful for live debugging and graphing of subsystems to observe internal
     * behaviour. An example of this is the {@link Accumulator}, which auto-logs pose estimates to the dashboard packet but can be
     * manually enabled for subsystems by this method.
     * <p>
     * Must be called periodically.
     *
     * @param subsystems the subsystems to extract {@link FlightRecorder} live log schemas from to display on a live FtcDashboard packet,
     *                   from {@link #usePacket(Consumer)}. These logs will not be displayed on DS telemetry.
     */
    public static void logLiveSchema(BunyipsSubsystem... subsystems) {
        usePacket(p -> {
            for (BunyipsSubsystem subsystem : subsystems) {
                for (Map.Entry<String, Object> field : traverseFields(new TreeMap<>(), subsystem.logger).entrySet()) {
                    p.put(field.getKey(), field.getValue().toString());
                }
            }
        });
    }

    private static TreeMap<String, Object> traverseFields(TreeMap<String, Object> fields, Object obj) {
        Field[] newFields = obj.getClass().getFields();
        for (Field newField : newFields) {
            try {
                if (newField.getType().isPrimitive() || newField.getType() == String.class || newField.getType().isEnum()) {
                    fields.put(newField.getName(), newField.get(obj));
                } else {
                    traverseFields(fields, Objects.requireNonNull(newField.get(obj)));
                }
            } catch (IllegalAccessException e) {
                throw new RuntimeException("Unable to access an internal field, this shouldn't happen!", e);
            }
        }
        return fields;
    }

    /**
     * Obtain a reference to a dashboard packet through an active {@link BunyipsOpMode}
     * or by creating a new packet to automatically send to FtcDashboard. Optionally allows packet synchronisation
     * through {@link #USING_SYNCED_PACKETS} and {@link #sendAndClearSyncedPackets()}.
     *
     * @param packetOperations the operations to perform on the packet, this packet will be managed
     *                         by this method or via the available {@link DualTelemetry} instance
     */
    public static void usePacket(@NonNull Consumer<TelemetryPacket> packetOperations) {
        BunyipsOpMode opMode = BunyipsOpMode.isRunning() ? BunyipsOpMode.getInstance() : null;
        TelemetryPacket packet;
        synchronized (Dashboard.class) {
            if (!USING_SYNCED_PACKETS) {
                packet = opMode == null ? new TelemetryPacket() : opMode.telemetry.getDashboardPacket();
            } else {
                packet = accumulatedPacket;
            }
        }
        // User operations, the packet may be sent manually here or automatically via BOM
        packetOperations.accept(packet);
        FtcDashboard dash = FtcDashboard.getInstance();
        if (opMode == null && !USING_SYNCED_PACKETS && dash != null)
            dash.sendTelemetryPacket(packet);
    }

    /**
     * Call to dispatch a synced packet update following the enabling of synced packets via {@link #USING_SYNCED_PACKETS}.
     * This ensures only one packet gets sent at a time avoiding overwriting.
     * <p>
     * No-ops if synced packets are not enabled.
     */
    public static void sendAndClearSyncedPackets() {
        synchronized (Dashboard.class) {
            if (!USING_SYNCED_PACKETS)
                return;
            FtcDashboard.getInstance().sendTelemetryPacket(accumulatedPacket);
            accumulatedPacket = new TelemetryPacket();
        }
    }
}
