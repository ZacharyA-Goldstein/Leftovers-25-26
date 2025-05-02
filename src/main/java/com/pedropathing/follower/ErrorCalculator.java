package com.pedropathing.follower;

import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.math.MathFunctions;

import java.util.Arrays;

/** This is the ErrorCalculator.
 * It is in charge of taking the Poses and Velocity produced by the PoseTracker and determining and returning the errors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 */
public class ErrorCalculator {
    private FollowerConstants constants;
    private final KalmanFilter driveKalmanFilter;
    private Pose closestPose, currentPose;
    private Path currentPath;
    private PathChain currentPathChain;
    private boolean followingPathChain;
    private double[] driveErrors;
    private int chainIndex;
    private double rawDriveError, previousRawDriveError, driveError, headingError, xMovement;
    private Vector velocityVector = new Vector();
    
    public ErrorCalculator(FollowerConstants constants) {
        this.constants = constants;
        
        KalmanFilterParameters driveKalmanFilterParameters = new KalmanFilterParameters(
                6,
                1);

        driveKalmanFilter = new KalmanFilter(driveKalmanFilterParameters);

    }

    public void update(Pose currentPose, Path currentPath, PathChain currentPathChain, boolean followingPathChain, Vector velocity, int chainIndex, double xMovement) {
        this.currentPose = currentPose;
        this.velocityVector = velocity;
        this.currentPath = currentPath;
        this.closestPose = this.currentPath.getClosestPoint(currentPose, 10);
        this.currentPathChain = currentPathChain;
        this.followingPathChain = followingPathChain;
        this.chainIndex = chainIndex;
        this.xMovement = xMovement;
    }

    /**
     * This returns the raw translational error, or how far off the closest point the robot is.
     *
     * @return This returns the raw translational error as a Vector.
     */
    public Vector getTranslationalError() {
        Vector error = new Vector();
        double x = closestPose.getX() - currentPose.getX();
        double y = closestPose.getY() - currentPose.getY();
        error.setOrthogonalComponents(x, y);
        return error;
    }

    /**
     * This returns the raw heading error, or how far off the closest point the robot is.
     *
     * @return This returns the raw heading error as a double.
     */
    public double getHeadingError() {
        headingError = MathFunctions.getTurnDirection(currentPose.getHeading(), currentPath.getClosestPointHeadingGoal()) * MathFunctions.getSmallestAngleDifference(currentPose.getHeading(), currentPath.getClosestPointHeadingGoal());
        return headingError;
    }

    /**
     * This returns the velocity the robot needs to be at to make it to the end of the Path
     * at some specified deceleration (well technically just some negative acceleration).
     *
     * @return returns the projected velocity.
     */
    public double getDriveVelocityError(double distanceToGoal) {
        Vector distanceToGoalVector = MathFunctions.scalarMultiplyVector(MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector()), distanceToGoal);
        Vector velocity = new Vector(MathFunctions.dotProduct(velocityVector, MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta());

        Vector forwardHeadingVector = new Vector(1.0, currentPose.getHeading());

        double forwardVelocity = MathFunctions.dotProduct(forwardHeadingVector, velocity);
        double forwardDistanceToGoal = MathFunctions.dotProduct(forwardHeadingVector, distanceToGoalVector);
        double forwardVelocityGoal = Math.signum(forwardDistanceToGoal) * Math.sqrt(Math.abs(-2 * currentPath.getZeroPowerAccelerationMultiplier() * constants.forwardZeroPowerAcceleration * (forwardDistanceToGoal <= 0 ? 1 : -1) * forwardDistanceToGoal));
        double forwardVelocityZeroPowerDecay = forwardVelocity - Math.signum(forwardDistanceToGoal) * Math.sqrt(Math.abs(Math.pow(forwardVelocity, 2) + 2 * constants.forwardZeroPowerAcceleration * Math.abs(forwardDistanceToGoal)));

        Vector lateralHeadingVector = new Vector(1.0, currentPose.getHeading() - Math.PI / 2);
        double lateralVelocity = MathFunctions.dotProduct(lateralHeadingVector, velocity);
        double lateralDistanceToGoal = MathFunctions.dotProduct(lateralHeadingVector, distanceToGoalVector);

        double lateralVelocityGoal = Math.signum(lateralDistanceToGoal) * Math.sqrt(Math.abs(-2 * currentPath.getZeroPowerAccelerationMultiplier() * constants.lateralZeroPowerAcceleration * (lateralDistanceToGoal <= 0 ? 1 : -1) * lateralDistanceToGoal));
        double lateralVelocityZeroPowerDecay = lateralVelocity - Math.signum(lateralDistanceToGoal) * Math.sqrt(Math.abs(Math.pow(lateralVelocity, 2) + 2 * constants.lateralZeroPowerAcceleration * Math.abs(lateralDistanceToGoal)));

        Vector forwardVelocityError = new Vector(forwardVelocityGoal - forwardVelocityZeroPowerDecay - forwardVelocity, forwardHeadingVector.getTheta());
        Vector lateralVelocityError = new Vector(lateralVelocityGoal - lateralVelocityZeroPowerDecay - lateralVelocity, lateralHeadingVector.getTheta());
        Vector velocityErrorVector = MathFunctions.addVectors(forwardVelocityError, lateralVelocityError);

        previousRawDriveError = rawDriveError;
        rawDriveError = velocityErrorVector.getMagnitude() * Math.signum(MathFunctions.dotProduct(velocityErrorVector, currentPath.getClosestPointTangentVector()));

        double projection = 2 * driveErrors[1] - driveErrors[0];

        driveKalmanFilter.update(rawDriveError - previousRawDriveError, projection);

        for (int i = 0; i < driveErrors.length - 1; i++) {
            driveErrors[i] = driveErrors[i + 1];
        }

        driveErrors[1] = driveKalmanFilter.getState();

        return driveKalmanFilter.getState();
    }

    public double getDriveError() {
        double distanceToGoal;
        if (!currentPath.isAtParametricEnd()) {
            if (followingPathChain && currentPathChain.getDecelerationType() == PathChain.DecelerationType.GLOBAL) {
                double remainingLength = 0;

                if (chainIndex < currentPathChain.size()) {
                    for (int i = chainIndex + 1; i<currentPathChain.size(); i++) {
                        remainingLength += currentPathChain.getPath(i).length();
                    }
                }

                distanceToGoal = remainingLength + currentPath.length() * (1 - currentPath.getClosestPointTValue());

                if (distanceToGoal >= Math.abs(currentPathChain.getDecelerationStartMultiplier() * 3/2 * Math.pow(xMovement, 2) / constants.forwardZeroPowerAcceleration)) {
                    return -1;
                }
            } else {
                distanceToGoal = currentPath.length() * (1 - currentPath.getClosestPointTValue());
            }
        } else {
            Vector offset = new Vector();
            offset.setOrthogonalComponents(currentPose.getX() - currentPath.getLastControlPoint().getX(), currentPose.getY() - currentPath.getLastControlPoint().getY());
            distanceToGoal = MathFunctions.dotProduct(currentPath.getEndTangent(), offset);
        }

        driveError = getDriveVelocityError(distanceToGoal);

        return driveError;
    }

    public double getRawDriveError() {
        return rawDriveError;
    }

    public double[] getDriveErrors() {
        return driveErrors;
    }

    public void breakFollowing() {
        driveError = 0;
        headingError = 0;
        rawDriveError = 0;
        previousRawDriveError = 0;
        driveErrors = new double[2];
        Arrays.fill(driveErrors, 0);
        driveKalmanFilter.reset();
    }
}
