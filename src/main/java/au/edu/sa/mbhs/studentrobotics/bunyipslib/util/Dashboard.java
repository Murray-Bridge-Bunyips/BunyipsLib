package au.edu.sa.mbhs.studentrobotics.bunyipslib.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.List;
import java.util.function.Consumer;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsOpMode;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.DualTelemetry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.purepursuit.path.Path;

/**
 * Set of helper functions for drawing on the FtcDashboard canvas.
 *
 * @since 6.0.0
 */
@Config
public final class Dashboard {
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
    }

    /**
     * Reset all static fields for an OpMode.
     */
    public static void resetForOpMode() {
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
     * Draw a path along sampled intervals.
     *
     * @param canvas     dashboard canvas
     * @param path       path to draw
     * @param resolution distance units; presumed inches
     */
    public static void drawSampledPath(@NonNull Canvas canvas, @NonNull Path path, double resolution) {
        int samples = (int) Math.ceil(path.length() / resolution);
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];
        double dx = path.length() / (samples - 1);
        for (int i = 0; i < samples; i++) {
            double displacement = i * dx;
            Pose2d pose = path.get(displacement);
            xPoints[i] = pose.position.x;
            yPoints[i] = pose.position.y;
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    /**
     * Draw a path along default sampled intervals.
     *
     * @param canvas dashboard canvas
     * @param path   path to draw
     */
    public static void drawSampledPath(@NonNull Canvas canvas, @NonNull Path path) {
        drawSampledPath(canvas, path, 2);
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
        if (opMode == null && !USING_SYNCED_PACKETS)
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
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
