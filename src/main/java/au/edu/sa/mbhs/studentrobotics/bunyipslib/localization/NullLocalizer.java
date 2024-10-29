package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;

/**
 * A localizer that does nothing.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public class NullLocalizer implements Localizer {
    @NonNull
    @Override
    public Twist2dDual<Time> update() {
        return new Twist2dDual<>(Vector2dDual.constant(Geometry.zeroVec(), 2), DualNum.constant(0, 2));
    }
}
