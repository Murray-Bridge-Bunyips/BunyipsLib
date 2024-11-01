package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Rotation2d;
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
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages.MecanumLocalizerInputsMessage;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.DriveModel;

/**
 * Mecanum drivebase localizer that uses four encoders and an IMU to localize the robot.
 * <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/5f35f4c22c1ae7c0be5b35da0961c8f3a181ad31/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumDrive.java#L125">Source</a>
 *
 * @since 6.0.0
 */
public class MecanumLocalizer implements Localizer {
    /**
     * The left front encoder.
     */
    public final Encoder leftFront;
    /**
     * The left back encoder.
     */
    public final Encoder leftBack;
    /**
     * The right back encoder.
     */
    public final Encoder rightBack;
    /**
     * The right front encoder.
     */
    public final Encoder rightFront;

    private final MecanumKinematics kinematics;
    private final DriveModel driveModel;
    private final IMU imu;

    private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
    private Rotation2d lastHeading;
    private boolean initialized;

    /**
     * Create a new MecanumLocalizer that will run on four Mecanum drive motors and an IMU.
     *
     * @param driveModel the drive model to use for kinematics
     * @param leftFront  the front left motor
     * @param leftBack   the back left motor
     * @param rightBack  the back right motor
     * @param rightFront the front right motor
     * @param imu        the IMU to use for heading
     */
    public MecanumLocalizer(@NonNull DriveModel driveModel, @Nullable DcMotor leftFront, @Nullable DcMotor leftBack, @Nullable DcMotor rightBack, @Nullable DcMotor rightFront, @Nullable IMU imu) {
        kinematics = new MecanumKinematics(driveModel.inPerTick * driveModel.trackWidthTicks, driveModel.inPerTick / driveModel.lateralInPerTick);
        this.driveModel = driveModel;
        this.leftFront = leftFront != null ? new OverflowEncoder(new RawEncoder((DcMotorEx) leftFront)) : null;
        this.leftBack = leftBack != null ? new OverflowEncoder(new RawEncoder((DcMotorEx) leftBack)) : null;
        this.rightBack = rightBack != null ? new OverflowEncoder(new RawEncoder((DcMotorEx) rightBack)) : null;
        this.rightFront = rightFront != null ? new OverflowEncoder(new RawEncoder((DcMotorEx) rightFront)) : null;
        this.imu = imu;
        // Wake up the IMU if it's a DynIMU
        if (imu != null)
            imu.getRobotOrientationAsQuaternion();
    }

    @NonNull
    @Override
    public Twist2dDual<Time> update() {
        if (leftFront == null || leftBack == null || rightBack == null || rightFront == null || imu == null)
            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );

        PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
        PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
        PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
        PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

        FlightRecorder.write("MECANUM_LOCALIZER_INPUTS", new MecanumLocalizerInputsMessage(
                leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles));

        Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

        if (!initialized) {
            initialized = true;

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        double headingDelta = heading.minus(lastHeading);
        Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                new DualNum<Time>(new double[]{
                        (leftFrontPosVel.position - lastLeftFrontPos),
                        leftFrontPosVel.velocity,
                }).times(driveModel.inPerTick),
                new DualNum<Time>(new double[]{
                        (leftBackPosVel.position - lastLeftBackPos),
                        leftBackPosVel.velocity,
                }).times(driveModel.inPerTick),
                new DualNum<Time>(new double[]{
                        (rightBackPosVel.position - lastRightBackPos),
                        rightBackPosVel.velocity,
                }).times(driveModel.inPerTick),
                new DualNum<Time>(new double[]{
                        (rightFrontPosVel.position - lastRightFrontPos),
                        rightFrontPosVel.velocity,
                }).times(driveModel.inPerTick)
        ));

        lastLeftFrontPos = leftFrontPosVel.position;
        lastLeftBackPos = leftBackPosVel.position;
        lastRightBackPos = rightBackPosVel.position;
        lastRightFrontPos = rightFrontPosVel.position;

        lastHeading = heading;

        return new Twist2dDual<>(
                twist.line,
                DualNum.cons(headingDelta, twist.angle.drop(1))
        );
    }
}