package com.pedropathing.follower;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class MecanumConstants {
    /** The Forward Velocity of the Robot - Different for each robot
     *  Default Value: 81.34056 */
    public static double xMovement = 81.34056;

    /** The Lateral Velocity of the Robot - Different for each robot
     *  Default Value: 65.43028 */
    public static double yMovement = 65.43028;

    private static double[] convertToPolar = Pose.cartesianToPolar(xMovement, -yMovement);

    /** The actual drive vector for the front left wheel, if the robot is facing a heading of 0 radians with the wheel centered at (0,0)
     *  Default Value: new Vector(convertToPolar[0], convertToPolar[1])
     * @implNote This vector should not be changed, but only accessed.
     */
    public static Vector frontLeftVector = MathFunctions.normalizeVector(new Vector(convertToPolar[0], convertToPolar[1]));
    public static double maxPower = 1;
    public static String leftFrontMotorName = "leftFront";
    public static String leftRearMotorName = "leftRear";
    public static String rightFrontMotorName = "rightFront";
    public static String rightRearMotorName = "rightRear";
    public static DcMotorSimple.Direction leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static double motorCachingThreshold = 0.01;
    public static boolean useBrakeModeInTeleOp = false;

    public MecanumConstants() {
        defaults();
    }

    public MecanumConstants xMovement(double xMovement) {
        MecanumConstants.xMovement = xMovement;
        return this;
    }

    public MecanumConstants yMovement(double yMovement) {
        MecanumConstants.yMovement = yMovement;
        return this;
    }

    public MecanumConstants maxPower(double maxPower) {
        MecanumConstants.maxPower = maxPower;
        return this;
    }

    public MecanumConstants leftFrontMotorName(String leftFrontMotorName) {
        MecanumConstants.leftFrontMotorName = leftFrontMotorName;
        return this;
    }

    public MecanumConstants leftRearMotorName(String leftRearMotorName) {
        MecanumConstants.leftRearMotorName = leftRearMotorName;
        return this;
    }

    public MecanumConstants rightFrontMotorName(String rightFrontMotorName) {
        MecanumConstants.rightFrontMotorName = rightFrontMotorName;
        return this;
    }

    public MecanumConstants rightRearMotorName(String rightRearMotorName) {
        MecanumConstants.rightRearMotorName = rightRearMotorName;
        return this;
    }

    public MecanumConstants leftFrontMotorDirection(DcMotorSimple.Direction leftFrontMotorDirection) {
        MecanumConstants.leftFrontMotorDirection = leftFrontMotorDirection;
        return this;
    }

    public MecanumConstants leftRearMotorDirection(DcMotorSimple.Direction leftRearMotorDirection) {
        MecanumConstants.leftRearMotorDirection = leftRearMotorDirection;
        return this;
    }

    public MecanumConstants rightFrontMotorDirection(DcMotorSimple.Direction rightFrontMotorDirection) {
        MecanumConstants.rightFrontMotorDirection = rightFrontMotorDirection;
        return this;
    }

    public MecanumConstants rightRearMotorDirection(DcMotorSimple.Direction rightRearMotorDirection) {
        MecanumConstants.rightRearMotorDirection = rightRearMotorDirection;
        return this;
    }

    public MecanumConstants motorCachingThreshold(double motorCachingThreshold) {
        MecanumConstants.motorCachingThreshold = motorCachingThreshold;
        return this;
    }

    public MecanumConstants useBrakeModeInTeleOp(boolean useBrakeModeInTeleOp) {
        MecanumConstants.useBrakeModeInTeleOp = useBrakeModeInTeleOp;
        return this;
    }

    /**
     * This method sets the default values for the MecanumConstants class.
     * It is called in the constructor of the MecanumConstants class.
     */
    public static void defaults() {
        xMovement = 81.34056;
        yMovement = 65.43028;
        convertToPolar = Pose.cartesianToPolar(xMovement, -yMovement);
        frontLeftVector = MathFunctions.normalizeVector(new Vector(convertToPolar[0], convertToPolar[1]));
        maxPower = 1;
        leftFrontMotorName = "leftFront";
        leftRearMotorName = "leftRear";
        rightFrontMotorName = "rightFront";
        rightRearMotorName = "rightRear";
        leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        motorCachingThreshold = 0.01;
        useBrakeModeInTeleOp = false;
    }
}
