package org.murraybridgebunyips.bunyipslib.transforms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Vector2d;

import org.murraybridgebunyips.bunyipslib.external.units.Distance;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.util.Text;

import java.util.Objects;

/**
 * A Rect is a set of two generic {@link Vector2d} instances that define a united upright rectangle.
 *
 * @author Lucas Bubner, 2024
 * @since 5.1.0
 */
public class Rect {
    /**
     * The distance unit these Rect dimensions were constructed with.
     */
    public final Distance unit;
    /**
     * The first point making up this Rect.
     */
    public Vector2d point1;
    /**
     * The second point making up this Rect.
     */
    public Vector2d point2;

    /**
     * Create a new Rect based on two points.
     *
     * @param p1   point 1
     * @param p2   point 2
     * @param unit the unit used for point 1 and 2
     */
    public Rect(Vector2d p1, Vector2d p2, Distance unit) {
        this.unit = unit;
        point1 = p1;
        point2 = p2;
    }

    /**
     * Normalise a Rect such that the first point is the bottom left and the second point is the top right.
     *
     * @param rect the Rect to normalise
     * @return the normalised Rect
     */
    public static Rect normalise(Rect rect) {
        Vector2d point1 = new Vector2d(Math.min(rect.point1.x, rect.point2.x), Math.min(rect.point1.y, rect.point2.y));
        Vector2d point2 = new Vector2d(Math.max(rect.point1.x, rect.point2.x), Math.max(rect.point1.y, rect.point2.y));
        return new Rect(point1, point2, rect.unit);
    }

    /**
     * Calculates the area of this Rect.
     *
     * @return the absolute area of this Rect.
     */
    public Measure<Distance> area() {
        return unit.of(Math.abs((point2.x - point1.x) * (point2.y - point1.y)));
    }

    /**
     * Check whether this Rect contains this {@link Vector2d}.
     *
     * @param point the point to check
     * @param unit  the distance unit of this point to convert to the {@link #unit}
     * @return whether the point is contained in the Rect
     */
    public boolean contains(Vector2d point, Distance unit) {
        point = new Vector2d(unit.of(point.x).in(this.unit), unit.of(point.y).in(this.unit));
        Rect normRect = normalise(this);
        return point.x >= normRect.point1.x && point.x <= normRect.point2.x
                && point.y >= normRect.point1.y && point.y <= normRect.point2.y;
    }

    /**
     * Check whether this Rect fully contains another Rect.
     *
     * @param other the other Rect to check
     * @return whether this Rect fully contains the other Rect
     */
    public boolean contains(Rect other) {
        Rect normRect = normalise(this);
        Rect normOther = normalise(other);
        return normRect.point1.x <= normOther.point1.x && normRect.point1.y <= normOther.point1.y
                && normRect.point2.x >= normOther.point2.x && normRect.point2.y >= normOther.point2.y;
    }

    /**
     * Check whether this Rect overlaps another Rect in some way.
     *
     * @param other the other Rect to check
     * @return whether this Rect overlaps the other Rect in some way
     */
    public boolean overlaps(Rect other) {
        Rect normRect = normalise(this);
        Rect normOther = normalise(other);
        return normRect.point1.x < normOther.point2.x && normRect.point2.x > normOther.point1.x
                && normRect.point1.y < normOther.point2.y && normRect.point2.y > normOther.point1.y;
    }

    /**
     * Return a new Rect such that the center of the new Rect is at the supplied point.
     *
     * @param center the center point of the new Rect
     * @return a new Rect with the same dimensions as this Rect and the center at the supplied point
     */
    public Rect centeredAt(Vector2d center) {
        Rect normRect = normalise(this);
        Vector2d halfSize = normRect.point2.minus(normRect.point1).times(0.5);
        return new Rect(center.minus(halfSize), center.plus(halfSize), unit);
    }

    /**
     * Draw this Rect on an FtcDashboard canvas.
     *
     * @param canvas the canvas to draw on
     */
    public void draw(Canvas canvas) {
        canvas.strokeRect(point1.x, point1.y, point2.x - point1.x, point2.y - point1.y);
    }

    /**
     * @return a copy of this Rect
     */
    public Rect copy() {
        return new Rect(point1, point2, unit);
    }

    @Override
    public int hashCode() {
        return Objects.hash(point1, point2, unit);
    }

    @Override
    public boolean equals(Object other) {
        if (this == other) return true;
        if (!(other instanceof Rect)) return false;
        Rect it = (Rect) other;
        return point1.equals(it.point1) && point2.equals(it.point2) && unit.equals(it.unit);
    }

    @NonNull
    @Override
    public String toString() {
        return Text.format("Rect{%, %, %}", point1, point2, unit);
    }
}
