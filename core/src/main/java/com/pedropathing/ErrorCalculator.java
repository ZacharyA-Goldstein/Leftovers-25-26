package com.pedropathing;

import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.math.Kinematics;
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
    private KalmanFilter driveKalmanFilter;
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

    public void update(Pose currentPose, Path currentPath, PathChain currentPathChain, boolean followingPathChain, Pose closestPose, Vector velocity, int chainIndex, double xMovement) {
        this.currentPose = currentPose;
        this.velocityVector = velocity;
        this.currentPath = currentPath;
        this.closestPose = closestPose;
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
        if (currentPath == null) {
            return 0;
        }

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
        if (currentPath == null) {
            return 0;
        }

        Vector distanceToGoalVector = currentPath.getClosestPointTangentVector().normalize().times(distanceToGoal);
        Vector velocity = new Vector(velocityVector.dot(currentPath.getClosestPointTangentVector().normalize()), currentPath.getClosestPointTangentVector().getTheta());

        Vector forwardHeadingVector = new Vector(1.0, currentPose.getHeading());

        double forwardVelocity = forwardHeadingVector.dot(velocity);
        double forwardDistanceToGoal = forwardHeadingVector.dot(distanceToGoalVector);
        double forwardVelocityGoal = Kinematics.getVelocityToStopWithDeceleration(
            forwardDistanceToGoal,
            constants.forwardZeroPowerAcceleration
                * ( currentPath.getDecelerationStrength() * 4)
        );
        double forwardVelocityZeroPowerDecay = forwardVelocity -
            Kinematics.getFinalVelocityAtDistance(
                forwardVelocity,
                constants.forwardZeroPowerAcceleration,
                forwardVelocityGoal
            );

        Vector lateralHeadingVector = new Vector(1.0, currentPose.getHeading() - Math.PI / 2);
        double lateralVelocity = lateralHeadingVector.dot(velocity);
        double lateralDistanceToGoal = lateralHeadingVector.dot(distanceToGoalVector);

        double lateralVelocityGoal = Kinematics.getVelocityToStopWithDeceleration(
            lateralDistanceToGoal,
            constants.lateralZeroPowerAcceleration
                * (currentPath.getDecelerationStrength() * 4)
        );
        double lateralVelocityZeroPowerDecay = lateralVelocity -
            Kinematics.getFinalVelocityAtDistance(
                lateralVelocity,
                constants.lateralZeroPowerAcceleration,
                lateralVelocityGoal
            );

        Vector forwardVelocityError = new Vector(forwardVelocityGoal - forwardVelocityZeroPowerDecay - forwardVelocity, forwardHeadingVector.getTheta());
        Vector lateralVelocityError = new Vector(lateralVelocityGoal - lateralVelocityZeroPowerDecay - lateralVelocity, lateralHeadingVector.getTheta());
        Vector velocityErrorVector = forwardVelocityError.plus(lateralVelocityError);

        previousRawDriveError = rawDriveError;
        rawDriveError = velocityErrorVector.getMagnitude() * Math.signum(velocityErrorVector.dot(currentPath.getClosestPointTangentVector()));

        double projection = Kinematics.predictNextLoopVelocity(driveErrors[1], driveErrors[0]);

        driveKalmanFilter.update(rawDriveError - previousRawDriveError, projection);

        for (int i = 0; i < driveErrors.length - 1; i++) {
            driveErrors[i] = driveErrors[i + 1];
        }

        driveErrors[1] = driveKalmanFilter.getState();

        return driveKalmanFilter.getState();
    }

    public double getDriveError() {
        double distanceToGoal;

        if (currentPath == null) {
            return 0;
        }

        if (!currentPath.isAtParametricEnd()) {
            if (followingPathChain && currentPathChain.getDecelerationType() == PathChain.DecelerationType.GLOBAL) {
                double remainingLength = 0;

                if (chainIndex < currentPathChain.size()) {
                    for (int i = chainIndex + 1; i<currentPathChain.size(); i++) {
                        remainingLength += currentPathChain.getPath(i).length();
                    }
                }

                distanceToGoal = remainingLength + currentPath.length() - currentPath.getDistanceTraveled();

                double stoppingDistance = Kinematics.getStoppingDistance(
                    xMovement, constants.forwardZeroPowerAcceleration
                );
                if (distanceToGoal >= Math.abs(stoppingDistance
                    * currentPathChain.getDecelerationStart() * 3)) {
                    return -1;
                }
            } else {
                distanceToGoal = currentPath.length() - currentPath.getDistanceTraveled();
            }
        } else {
            Vector offset = new Vector();
            offset.setOrthogonalComponents(currentPose.getX() - currentPath.getLastControlPoint().getX(), currentPose.getY() - currentPath.getLastControlPoint().getY());
            distanceToGoal = currentPath.getEndTangent().dot(offset);
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

    public void setConstants(FollowerConstants constants) {
        this.constants = constants;
    }

    public String debugString() {
        return "Current Pose: " + currentPose.toString() + "\n" +
               "Closest Pose: " + closestPose.toString() + "\n" +
               "Current Path: " + (currentPath != null ? currentPath.toString() : "null") + "\n" + "Following Path Chain: " + followingPathChain + "\n" +
               "Chain Index: " + chainIndex + "\n" +
               "Drive Error: " + getDriveError() + "\n" +
               "Heading Error: " + getHeadingError() + "\n" +
               "Raw Drive Error: " + getRawDriveError();
    }
}
