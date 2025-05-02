package com.pedropathing.follower;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.math.Vector;
import com.pedropathing.control.FilteredPIDFController;
import com.pedropathing.control.PIDFController;

import java.util.ArrayList;

/** This is the VectorCalculator.
 * It is in charge of taking the errors produced by the ErrorCalculator and determining and returning drive + corrective vectors
 *
 * @author Baron Henderson - 20077 The Indubitables
 */
public class VectorCalculator {
    private Path currentPath;
    private PathChain currentPathChain;
    private Pose currentPose, closestPose;
    private double headingError, driveError;

    private ArrayList<Vector> velocities = new ArrayList<>();
    private ArrayList<Vector> accelerations = new ArrayList<>();
    private Vector velocity = new Vector();

    private Vector averageVelocity, averagePreviousVelocity, averageAcceleration;
    private Vector secondaryTranslationalIntegralVector, translationalIntegralVector;
    private Vector teleopDriveVector, teleopHeadingVector;

    public Vector driveVector, headingVector, translationalVector, centripetalVector, correctiveVector;

    private double previousSecondaryTranslationalIntegral;
    private double previousTranslationalIntegral;
    public static double drivePIDFFeedForward, secondaryDrivePIDFFeedForward, headingPIDFFeedForward, secondaryHeadingPIDFFeedForward, translationalPIDFFeedForward, secondaryTranslationalPIDFFeedForward, drivePIDFSwitch, headingPIDFSwitch, translationalPIDFSwitch;
    public static boolean useSecondaryDrivePID, useSecondaryHeadingPID, useSecondaryTranslationalPID;
    private double[] teleopDriveValues;

    private boolean useDrive = true, useHeading = true, useTranslational = true, useCentripetal = true, teleopDrive = false, followingPathChain = false;
    private double maxPowerScaling = 1.0, mass = 10.65;

    private int chainIndex;
    private double centripetalScaling;

    private PIDFController secondaryTranslationalPIDF;
    private PIDFController secondaryTranslationalIntegral;
    private PIDFController translationalPIDF;
    private PIDFController translationalIntegral;
    private PIDFController secondaryHeadingPIDF;

    private PIDFController headingPIDF;
    private FilteredPIDFController secondaryDrivePIDF, drivePIDF;

    public VectorCalculator() {
        drivePIDF = new FilteredPIDFController(FollowerConstants.drivePIDFCoefficients);
        secondaryDrivePIDF = new FilteredPIDFController(FollowerConstants.secondaryDrivePIDFCoefficients);
        headingPIDF = new PIDFController(FollowerConstants.headingPIDFCoefficients);
        secondaryHeadingPIDF = new PIDFController(FollowerConstants.secondaryHeadingPIDFCoefficients);
        translationalPIDF = new PIDFController(FollowerConstants.translationalPIDFCoefficients);
        secondaryTranslationalPIDF = new PIDFController(FollowerConstants.secondaryTranslationalPIDFCoefficients);
        translationalIntegral = new PIDFController(FollowerConstants.translationalIntegral);
        secondaryTranslationalIntegral = new PIDFController(FollowerConstants.secondaryTranslationalIntegral);
        updateConstants();
    }
    
    public void updateConstants() {
        drivePIDF.setCoefficients(FollowerConstants.drivePIDFCoefficients);
        secondaryDrivePIDF.setCoefficients(FollowerConstants.secondaryDrivePIDFCoefficients);
        headingPIDF.setCoefficients(FollowerConstants.headingPIDFCoefficients);
        secondaryHeadingPIDF.setCoefficients(FollowerConstants.secondaryHeadingPIDFCoefficients);
        translationalPIDF.setCoefficients(FollowerConstants.translationalPIDFCoefficients);
        secondaryTranslationalPIDF.setCoefficients(FollowerConstants.secondaryTranslationalPIDFCoefficients);
        translationalIntegral.setCoefficients(FollowerConstants.translationalIntegral);
        secondaryTranslationalIntegral.setCoefficients(FollowerConstants.secondaryTranslationalIntegral);
        drivePIDFSwitch = FollowerConstants.drivePIDFSwitch;
        headingPIDFSwitch = FollowerConstants.headingPIDFSwitch;
        translationalPIDFSwitch = FollowerConstants.translationalPIDFSwitch;
        drivePIDFFeedForward = FollowerConstants.drivePIDFFeedForward;
        secondaryDrivePIDFFeedForward = FollowerConstants.secondaryDrivePIDFFeedForward;
        headingPIDFFeedForward = FollowerConstants.headingPIDFFeedForward;
        secondaryHeadingPIDFFeedForward = FollowerConstants.secondaryHeadingPIDFFeedForward;
        translationalPIDFFeedForward = FollowerConstants.translationalPIDFFeedForward;
        secondaryTranslationalPIDFFeedForward = FollowerConstants.secondaryTranslationalPIDFFeedForward;
        useSecondaryDrivePID = FollowerConstants.useSecondaryDrivePIDF;
        useSecondaryHeadingPID = FollowerConstants.useSecondaryHeadingPIDF;
        useSecondaryTranslationalPID = FollowerConstants.useSecondaryTranslationalPIDF;
        mass = FollowerConstants.mass;
    }

    public void update(boolean useDrive, boolean useHeading, boolean useTranslational, boolean useCentripetal, boolean teleopDrive, int chainIndex, double maxPowerScaling, boolean followingPathChain, double centripetalScaling, Pose currentPose, Pose closestPose, Vector velocity, Path currentPath, PathChain currentPathChain, double driveError, double headingError) {
        updateConstants();

        this.useDrive = useDrive;
        this.useHeading = useHeading;
        this.useTranslational = useTranslational;
        this.useCentripetal = useCentripetal;
        this.teleopDrive = teleopDrive;
        this.maxPowerScaling = maxPowerScaling;
        this.chainIndex = chainIndex;
        this.followingPathChain = followingPathChain;
        this.centripetalScaling = centripetalScaling;
        this.currentPose = currentPose;
        this.closestPose = closestPose;
        this.velocity = velocity;
        this.currentPath = currentPath;
        this.currentPathChain = currentPathChain;
        this.driveError = driveError;
        this.headingError = headingError;

        if(teleopDrive)
            teleopUpdate();
    }
    
    public void breakFollowing() {
        secondaryDrivePIDF.reset();
        drivePIDF.reset();
        secondaryHeadingPIDF.reset();
        headingPIDF.reset();
        secondaryTranslationalPIDF.reset();
        secondaryTranslationalIntegral.reset();
        translationalPIDF.reset();
        translationalIntegral.reset();
        
        secondaryTranslationalIntegralVector = new Vector();
        translationalIntegralVector = new Vector();
        driveVector = new Vector();
        headingVector = new Vector();
        translationalVector = new Vector();
        centripetalVector = new Vector();
        correctiveVector = new Vector();

        int AVERAGED_VELOCITY_SAMPLE_NUMBER = 8;
        for (int i = 0; i < AVERAGED_VELOCITY_SAMPLE_NUMBER; i++) {
            velocities.add(new Vector());
        }
        for (int i = 0; i < AVERAGED_VELOCITY_SAMPLE_NUMBER / 2; i++) {
            accelerations.add(new Vector());
        }

        calculateAveragedVelocityAndAcceleration();
        teleopDriveVector = new Vector();
        teleopHeadingVector = new Vector();
        previousSecondaryTranslationalIntegral = 0;
        previousTranslationalIntegral = 0;
        teleopDriveValues = new double[3];
    }

    /**
     * Do the teleop calculations
     */
    public void teleopUpdate() {
        velocities.add(velocity);
        velocities.remove(velocities.get(velocities.size() - 1));

        calculateAveragedVelocityAndAcceleration();
    }

    /**
     * This returns a Vector in the direction the robot must go to move along the path. This Vector
     * takes into account the projected position of the robot to calculate how much power is needed.
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude.
     *
     * @return returns the drive vector.
     */
    public Vector getDriveVector() {
        if (!useDrive) return new Vector();

        if (followingPathChain && ((chainIndex < currentPathChain.size() - 1 && currentPathChain.getDecelerationType() == PathChain.DecelerationType.LAST_PATH) || currentPathChain.getDecelerationType() == PathChain.DecelerationType.NONE)) {
            return new Vector(maxPowerScaling, currentPath.getClosestPointTangentVector().getTheta());
        }

        if (driveError == -1)
            return new Vector(maxPowerScaling, currentPath.getClosestPointTangentVector().getTheta());

        if (Math.abs(driveError) < drivePIDFSwitch && useSecondaryDrivePID) {
            secondaryDrivePIDF.updateError(driveError);
            driveVector = new Vector(MathFunctions.clamp(secondaryDrivePIDF.runPIDF() + secondaryDrivePIDFFeedForward * Math.signum(driveError), -maxPowerScaling, maxPowerScaling), currentPath.getClosestPointTangentVector().getTheta());
            return MathFunctions.copyVector(driveVector);
        }

        drivePIDF.updateError(driveError);
        driveVector = new Vector(MathFunctions.clamp(drivePIDF.runPIDF() + drivePIDFFeedForward * Math.signum(driveError), -maxPowerScaling, maxPowerScaling), currentPath.getClosestPointTangentVector().getTheta());
        return MathFunctions.copyVector(driveVector);
    }

    /**
     * This returns a Vector in the direction of the robot that contains the heading correction
     * as its magnitude. Positive heading correction turns the robot counter-clockwise, and negative
     * heading correction values turn the robot clockwise. So basically, Pedro Pathing uses a right-
     * handed coordinate system.
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude.
     *
     * @return returns the heading vector.
     */
    public Vector getHeadingVector() {
        if (!useHeading) return new Vector();
        if (Math.abs(headingError) < headingPIDFSwitch && useSecondaryHeadingPID) {
            secondaryHeadingPIDF.updateError(headingError);
            headingVector = new Vector(MathFunctions.clamp(secondaryHeadingPIDF.runPIDF() + secondaryHeadingPIDFFeedForward * MathFunctions.getTurnDirection(currentPose.getHeading(), currentPath.getClosestPointHeadingGoal()), -maxPowerScaling, maxPowerScaling), currentPose.getHeading());
            return MathFunctions.copyVector(headingVector);
        }
        headingPIDF.updateError(headingError);
        headingVector = new Vector(MathFunctions.clamp(headingPIDF.runPIDF() + headingPIDFFeedForward * MathFunctions.getTurnDirection(currentPose.getHeading(), currentPath.getClosestPointHeadingGoal()), -maxPowerScaling, maxPowerScaling), currentPose.getHeading());
        return MathFunctions.copyVector(headingVector);
    }

    /**
     * This returns a combined Vector in the direction the robot must go to correct both translational
     * error as well as centripetal force.
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude.
     *
     * @return returns the corrective vector.
     */
    public Vector getCorrectiveVector() {
        Vector centripetal = getCentripetalForceCorrection();
        Vector translational = getTranslationalCorrection();
        Vector corrective = MathFunctions.addVectors(centripetal, translational);

        if (corrective.getMagnitude() > maxPowerScaling) {
            return MathFunctions.addVectors(centripetal, MathFunctions.scalarMultiplyVector(translational, MathFunctions.findNormalizingScaling(centripetal, translational, maxPowerScaling)));
        }

        correctiveVector = MathFunctions.copyVector(corrective);

        return corrective;
    }

    /**
     * This returns a Vector in the direction the robot must go to account for only translational
     * error.
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude.
     *
     * @return returns the translational correction vector.
     */
    public Vector getTranslationalCorrection() {
        if (!useTranslational) return new Vector();
        Vector translationalVector = new Vector();
        double x = closestPose.getX() - currentPose.getX();
        double y = closestPose.getY() - currentPose.getY();
        translationalVector.setOrthogonalComponents(x, y);

        if (!(currentPath.isAtParametricEnd() || currentPath.isAtParametricStart())) {
            translationalVector = MathFunctions.subtractVectors(translationalVector, new Vector(MathFunctions.dotProduct(translationalVector, MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta()));

            secondaryTranslationalIntegralVector = MathFunctions.subtractVectors(secondaryTranslationalIntegralVector, new Vector(MathFunctions.dotProduct(secondaryTranslationalIntegralVector, MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta()));
            translationalIntegralVector = MathFunctions.subtractVectors(translationalIntegralVector, new Vector(MathFunctions.dotProduct(translationalIntegralVector, MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta()));
        }

        if (MathFunctions.distance(currentPose, closestPose) < translationalPIDFSwitch && useSecondaryTranslationalPID) {
            secondaryTranslationalIntegral.updateError(translationalVector.getMagnitude());
            secondaryTranslationalIntegralVector = MathFunctions.addVectors(secondaryTranslationalIntegralVector, new Vector(secondaryTranslationalIntegral.runPIDF() - previousSecondaryTranslationalIntegral, translationalVector.getTheta()));
            previousSecondaryTranslationalIntegral = secondaryTranslationalIntegral.runPIDF();

            secondaryTranslationalPIDF.updateError(translationalVector.getMagnitude());
            translationalVector.setMagnitude(secondaryTranslationalPIDF.runPIDF() + secondaryTranslationalPIDFFeedForward);
            translationalVector = MathFunctions.addVectors(translationalVector, secondaryTranslationalIntegralVector);
        } else {
            translationalIntegral.updateError(translationalVector.getMagnitude());
            translationalIntegralVector = MathFunctions.addVectors(translationalIntegralVector, new Vector(translationalIntegral.runPIDF() - previousTranslationalIntegral, translationalVector.getTheta()));
            previousTranslationalIntegral = translationalIntegral.runPIDF();

            translationalPIDF.updateError(translationalVector.getMagnitude());
            translationalVector.setMagnitude(translationalPIDF.runPIDF() + translationalPIDFFeedForward);
            translationalVector = MathFunctions.addVectors(translationalVector, translationalIntegralVector);
        }

        translationalVector.setMagnitude(MathFunctions.clamp(translationalVector.getMagnitude(), 0, maxPowerScaling));

        this.translationalVector = MathFunctions.copyVector(translationalVector);

        return translationalVector;
    }

    /**
     * This returns a Vector in the direction the robot must go to account for only centripetal
     * force.
     * <p>
     * Note: This vector is clamped to be between [0, 1] in magnitude.
     *
     * @return returns the centripetal force correction vector.
     */
    public Vector getCentripetalForceCorrection() {
        if (!useCentripetal) return new Vector();
        double curvature;
        if (!teleopDrive) {
            curvature = currentPath.getClosestPointCurvature();
        } else {
            double yPrime = averageVelocity.getYComponent() / averageVelocity.getXComponent();
            double yDoublePrime = averageAcceleration.getYComponent() / averageVelocity.getXComponent();
            curvature = (yDoublePrime) / (Math.pow(Math.sqrt(1 + Math.pow(yPrime, 2)), 3));
        }
        if (Double.isNaN(curvature)) return new Vector();
        centripetalVector = new Vector(MathFunctions.clamp(centripetalScaling * mass * Math.pow(MathFunctions.dotProduct(velocity, MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), 2) * curvature, -maxPowerScaling, maxPowerScaling), currentPath.getClosestPointTangentVector().getTheta() + Math.PI / 2 * Math.signum(currentPath.getClosestPointNormalVector().getTheta()));
        return centripetalVector;
    }

    /**
     * This sets the teleop drive vectors.
     *
     * @param forwardDrive determines the forward drive vector for the robot in teleop. In field centric
     *                     movement, this is the x-axis.
     * @param lateralDrive determines the lateral drive vector for the robot in teleop. In field centric
     *                     movement, this is the y-axis.
     * @param heading determines the heading vector for the robot in teleop.
     * @param robotCentric sets if the movement will be field or robot centric
     */
    public void setTeleOpMovementVectors(double forwardDrive, double lateralDrive, double heading, boolean robotCentric) {
        teleopDriveValues[0] = MathFunctions.clamp(forwardDrive, -1, 1);
        teleopDriveValues[1] = MathFunctions.clamp(lateralDrive, -1, 1);
        teleopDriveValues[2] = MathFunctions.clamp(heading, -1, 1);
        teleopDriveVector.setOrthogonalComponents(teleopDriveValues[0], teleopDriveValues[1]);
        teleopDriveVector.setMagnitude(MathFunctions.clamp(teleopDriveVector.getMagnitude(), 0, 1));

        if (robotCentric) {
            teleopDriveVector.rotateVector(currentPose.getHeading());
        }

        teleopHeadingVector.setComponents(teleopDriveValues[2], currentPose.getHeading());
    }

    /**
     * This sets the teleop drive vectors. This defaults to robot centric.
     *
     * @param forwardDrive determines the forward drive vector for the robot in teleop. In field centric
     *                     movement, this is the x-axis.
     * @param lateralDrive determines the lateral drive vector for the robot in teleop. In field centric
     *                     movement, this is the y-axis.
     * @param heading determines the heading vector for the robot in teleop.
     */
    public void setTeleOpMovementVectors(double forwardDrive, double lateralDrive, double heading) {
        setTeleOpMovementVectors(forwardDrive, lateralDrive, heading, true);
    }

    /**
     * This calculates an averaged approximate velocity and acceleration. This is used for a
     * real-time correction of centripetal force, which is used in teleop.
     */
    public void calculateAveragedVelocityAndAcceleration() {
        averageVelocity = new Vector();
        averagePreviousVelocity = new Vector();

        for (int i = 0; i < velocities.size() / 2; i++) {
            averageVelocity = MathFunctions.addVectors(averageVelocity, velocities.get(i));
        }
        averageVelocity = MathFunctions.scalarMultiplyVector(averageVelocity, 1.0 / ((double) velocities.size() / 2));

        for (int i = velocities.size() / 2; i < velocities.size(); i++) {
            averagePreviousVelocity = MathFunctions.addVectors(averagePreviousVelocity, velocities.get(i));
        }
        averagePreviousVelocity = MathFunctions.scalarMultiplyVector(averagePreviousVelocity, 1.0 / ((double) velocities.size() / 2));

        accelerations.add(MathFunctions.subtractVectors(averageVelocity, averagePreviousVelocity));
        accelerations.remove(accelerations.size() - 1);

        averageAcceleration = new Vector();

        for (int i = 0; i < accelerations.size(); i++) {
            averageAcceleration = MathFunctions.addVectors(averageAcceleration, accelerations.get(i));
        }
        averageAcceleration = MathFunctions.scalarMultiplyVector(averageAcceleration, 1.0 / accelerations.size());
    }

    public boolean isTeleopDrive() {
        return teleopDrive;
    }

    public Vector getCentripetalVector() {
        return centripetalVector;
    }

    public Vector getTranslationalVector() {
        return translationalVector;
    }

    public Vector getTeleopHeadingVector() {
        return teleopHeadingVector;
    }

    public Vector getTeleopDriveVector() {
        return teleopDriveVector;
    }

    public Vector getTranslationalIntegralVector() {
        return translationalIntegralVector;
    }

    public Vector getAverageAcceleration() {
        return averageAcceleration;
    }

    public Vector getSecondaryTranslationalIntegralVector() {
        return secondaryTranslationalIntegralVector;
    }

    public Vector getAveragePreviousVelocity() {
        return averagePreviousVelocity;
    }

    public Vector getAverageVelocity() {
        return averageVelocity;
    }

    public void setDrivePIDFCoefficients(FilteredPIDFCoefficients drivePIDFCoefficients) {
        this.drivePIDF.setCoefficients(drivePIDFCoefficients);
    }

    public void setSecondaryDrivePIDFCoefficients(FilteredPIDFCoefficients secondaryDrivePIDFCoefficients) {
        this.secondaryDrivePIDF.setCoefficients(secondaryDrivePIDFCoefficients);
    }

    public void setHeadingPIDFCoefficients(PIDFCoefficients headingPIDFCoefficients) {
        this.headingPIDF.setCoefficients(headingPIDFCoefficients);
    }

    public void setSecondaryHeadingPIDFCoefficients(PIDFCoefficients secondaryHeadingPIDFCoefficients) {
        this.secondaryHeadingPIDF.setCoefficients(secondaryHeadingPIDFCoefficients);
    }

    public void setTranslationalPIDFCoefficients(PIDFCoefficients translationalPIDFCoefficients) {
        translationalPIDF.setCoefficients(translationalPIDFCoefficients);
    }

    public void setSecondaryTranslationalPIDFCoefficients(PIDFCoefficients secondaryTranslationalPIDFCoefficients) {
        this.secondaryTranslationalPIDF.setCoefficients(secondaryTranslationalPIDFCoefficients);
    }
    
}