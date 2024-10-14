package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.DriveModel;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages.ThreeDeadWheelInputsMessage;

/**
 * Standard three dead wheel localizer that uses two parallel encoders and one perpendicular encoder to localize the robot.
 * <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/5f35f4c22c1ae7c0be5b35da0961c8f3a181ad31/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ThreeDeadWheelLocalizer.java">Source</a>
 *
 * @since 6.0.0
 */
public class ThreeWheelLocalizer implements Localizer {
    /**
     * The first parallel encoder.
     */
    public final Encoder par0;
    /**
     * The second parallel encoder.
     */
    public final Encoder par1;
    /**
     * The perpendicular encoder.
     */
    public final Encoder perp;

    private final DriveModel driveModel;
    private final Params params;
    private int lastPar0Pos, lastPar1Pos, lastPerpPos;
    private boolean initialized;

    /**
     * Create a new ThreeWheelLocalizer that will run on two parallel and one perpendicular encoders.
     *
     * @param driveModel the drive model to use for kinematics
     * @param params     the parameters for the dead wheel localizer
     * @param par0       the first parallel encoder
     * @param par1       the second parallel encoder
     * @param perp       the perpendicular encoder
     */
    public ThreeWheelLocalizer(DriveModel driveModel, Params params, RawEncoder par0, RawEncoder par1, RawEncoder perp) {
        this.par0 = new OverflowEncoder(par0);
        this.par1 = new OverflowEncoder(par1);
        this.perp = new OverflowEncoder(perp);
        this.driveModel = driveModel;
        this.params = params;

        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", params);
    }

    @NonNull
    public Twist2dDual<Time> update() {
        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        FlightRecorder.write("THREE_DEAD_WHEEL_INPUTS", new ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel));

        if (!initialized) {
            initialized = true;

            lastPar0Pos = par0PosVel.position;
            lastPar1Pos = par1PosVel.position;
            lastPerpPos = perpPosVel.position;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[]{
                                (params.par0YTicks * par1PosDelta - params.par1YTicks * par0PosDelta) / (params.par0YTicks - params.par1YTicks),
                                (params.par0YTicks * par1PosVel.velocity - params.par1YTicks * par0PosVel.velocity) / (params.par0YTicks - params.par1YTicks),
                        }).times(driveModel.inPerTick),
                        new DualNum<Time>(new double[]{
                                (params.perpXTicks / (params.par0YTicks - params.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (params.perpXTicks / (params.par0YTicks - params.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(driveModel.inPerTick)
                ),
                new DualNum<>(new double[]{
                        (par0PosDelta - par1PosDelta) / (params.par0YTicks - params.par1YTicks),
                        (par0PosVel.velocity - par1PosVel.velocity) / (params.par0YTicks - params.par1YTicks),
                })
        );

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

        return twist;
    }

    /**
     * Parameters for the three dead wheel localizer.
     */
    public static class Params {
        /**
         * The y position of the first parallel encoder (in tick units).
         */
        public double par0YTicks = 0.0;
        /**
         * The y position of the second parallel encoder (in tick units).
         */
        public double par1YTicks = 1.0;
        /**
         * The x position of the perpendicular encoder (in tick units).
         */
        public double perpXTicks = 0.0;

        /**
         * Utility builder for the three dead wheel localizer parameters.
         */
        public static class Builder {
            private final Params params = new Params();

            /**
             * Set the y position of the first parallel encoder (in tick units).
             *
             * @param par0YTicks the y position of the first parallel encoder as determined by tuning the deadwheel localizer
             * @return this builder
             */
            public Builder setPar0YTicks(double par0YTicks) {
                params.par0YTicks = par0YTicks;
                return this;
            }

            /**
             * Set the y position of the second parallel encoder (in tick units).
             *
             * @param par1YTicks the y position of the second parallel encoder as determined by tuning the deadwheel localizer
             * @return this builder
             */
            public Builder setPar1YTicks(double par1YTicks) {
                params.par1YTicks = par1YTicks;
                return this;
            }

            /**
             * Set the x position of the perpendicular encoder (in tick units).
             *
             * @param perpXTicks the x position of the perpendicular encoder as determined by tuning the deadwheel localizer
             * @return this builder
             */
            public Builder setPerpXTicks(double perpXTicks) {
                params.perpXTicks = perpXTicks;
                return this;
            }

            /**
             * Build the parameters.
             *
             * @return the parameters
             */
            public Params build() {
                return params;
            }
        }
    }
}
