package org.murraybridgebunyips.bunyipslib.roadrunner.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;

import java.util.List;
import java.util.function.Consumer;

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 *
 * @since 1.0.0-pre
 */
public final class DashboardUtil {
    private static final double DEFAULT_RESOLUTION = 2.0; // distance units; presumed inches
    private static final double ROBOT_RADIUS = 9; // in

    private DashboardUtil() {
    }

    /**
     * Draw a filled polygon on the dashboard canvas.
     *
     * @param canvas      dashboard canvas
     * @param poseHistory list of robot poses
     */
    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d pose = poseHistory.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    /**
     * Draw a filled polygon on the dashboard canvas.
     *
     * @param canvas     dashboard canvas
     * @param path       path to draw
     * @param resolution distance units; presumed inches
     */
    public static void drawSampledPath(Canvas canvas, Path path, double resolution) {
        int samples = (int) Math.ceil(path.length() / resolution);
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];
        double dx = path.length() / (samples - 1);
        for (int i = 0; i < samples; i++) {
            double displacement = i * dx;
            Pose2d pose = path.get(displacement);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    /**
     * Draw a filled polygon on the dashboard canvas.
     *
     * @param canvas dashboard canvas
     * @param path   path to draw
     */
    public static void drawSampledPath(Canvas canvas, Path path) {
        drawSampledPath(canvas, path, DEFAULT_RESOLUTION);
    }

    /**
     * Draw a filled polygon on the dashboard canvas.
     *
     * @param canvas dashboard canvas
     * @param pose   robot pose
     */
    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS);
        Vector2d v = pose.headingVec().times(ROBOT_RADIUS);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }

    /**
     * Obtain a reference to the dashboard field canvas through an active {@link BunyipsOpMode}
     * or by creating a new packet to automatically send to FtcDashboard.
     *
     * @param canvasOperations the operation to perform on the canvas before auto sending
     */
    public static void useCanvas(Consumer<Canvas> canvasOperations) {
        BunyipsOpMode opMode = BunyipsOpMode.isRunning() ? BunyipsOpMode.getInstance() : null;
        TelemetryPacket packet = opMode == null ? new TelemetryPacket() : null;
        Canvas canvas = opMode != null ? opMode.telemetry.dashboardFieldOverlay() : packet.fieldOverlay();
        // User operations, the packet may be sent manually here or automatically via BOM
        canvasOperations.accept(canvas);
        if (packet != null)
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
