package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages.TankLocalizerInputsMessage;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.DriveModel;

/**
 * Tank drivebase localizer that uses drive encoders on all parallel wheels to localize the robot.
 * <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/5f35f4c22c1ae7c0be5b35da0961c8f3a181ad31/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TankDrive.java#L130">Source</a>
 *
 * @since 6.0.0
 */
public class TankLocalizer implements Localizer {
    /**
     * Encoders on the left side of the robot.
     */
    public final List<Encoder> leftEncs;
    /**
     * Encoders on the right side of the robot.
     */
    public final List<Encoder> rightEncs;

    private final TankKinematics kinematics;
    private final DriveModel driveModel;
    private double lastLeftPos, lastRightPos;
    private boolean initialized;

    /**
     * Create a new TankLocalizer that will run on two parallel sets of drive motors.
     *
     * @param driveModel  the drive model to use for kinematics
     * @param leftMotors  the left motors
     * @param rightMotors the right motors
     */
    public TankLocalizer(@NonNull DriveModel driveModel, @NonNull List<DcMotor> leftMotors, @NonNull List<DcMotor> rightMotors) {
        kinematics = new TankKinematics(driveModel.inPerTick * driveModel.trackWidthTicks);
        this.driveModel = driveModel;

        List<Encoder> leftEncs = new ArrayList<>();
        for (DcMotor m : leftMotors) {
            Encoder e = new OverflowEncoder(new RawEncoder((DcMotorEx) m));
            leftEncs.add(e);
        }
        this.leftEncs = Collections.unmodifiableList(leftEncs);

        List<Encoder> rightEncs = new ArrayList<>();
        for (DcMotor m : rightMotors) {
            Encoder e = new OverflowEncoder(new RawEncoder((DcMotorEx) m));
            rightEncs.add(e);
        }
        this.rightEncs = Collections.unmodifiableList(rightEncs);
    }

    @NonNull
    @Override
    public Twist2dDual<Time> update() {
        List<PositionVelocityPair> leftReadings = new ArrayList<>(), rightReadings = new ArrayList<>();

        double meanLeftPos = 0.0, meanLeftVel = 0.0;
        for (Encoder e : leftEncs) {
            PositionVelocityPair p = e.getPositionAndVelocity();
            assert p.velocity != null;
            meanLeftPos += p.position;
            meanLeftVel += p.velocity;
            leftReadings.add(p);
        }
        meanLeftPos /= leftEncs.size();
        meanLeftVel /= leftEncs.size();

        double meanRightPos = 0.0, meanRightVel = 0.0;
        for (Encoder e : rightEncs) {
            PositionVelocityPair p = e.getPositionAndVelocity();
            meanRightPos += p.position;
            assert p.velocity != null;
            meanRightVel += p.velocity;
            rightReadings.add(p);
        }
        meanRightPos /= rightEncs.size();
        meanRightVel /= rightEncs.size();

        FlightRecorder.write("LOCALIZER_INPUTS_TANK",
                new TankLocalizerInputsMessage(leftReadings, rightReadings));

        if (!initialized) {
            initialized = true;

            lastLeftPos = meanLeftPos;
            lastRightPos = meanRightPos;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        TankKinematics.WheelIncrements<Time> twist = new TankKinematics.WheelIncrements<>(
                new DualNum<Time>(new double[]{
                        meanLeftPos - lastLeftPos,
                        meanLeftVel
                }).times(driveModel.inPerTick),
                new DualNum<Time>(new double[]{
                        meanRightPos - lastRightPos,
                        meanRightVel,
                }).times(driveModel.inPerTick)
        );

        lastLeftPos = meanLeftPos;
        lastRightPos = meanRightPos;

        return kinematics.forward(twist);
    }
}
