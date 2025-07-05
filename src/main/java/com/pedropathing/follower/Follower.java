package com.pedropathing.follower;

import com.pedropathing.ErrorCalculator;
import com.pedropathing.VectorCalculator;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.Drivetrain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.paths.PathPoint;
import com.pedropathing.util.DashboardPoseTracker;

import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathCallback;
import com.pedropathing.paths.PathChain;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;

/**
 * This is the Follower class. It handles the actual following of the paths and all the on-the-fly
 * calculations that are relevant for movement.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.1.0, 5/1/2025
 */
public class Follower {
    public FollowerConstants constants;
    public PathConstraints pathConstraints;
    public PoseTracker poseTracker;
    public ErrorCalculator errorCalculator;
    public VectorCalculator vectorCalculator;
    public Drivetrain drivetrain;
    private final DashboardPoseTracker dashboardPoseTracker;

    private Pose currentPose = new Pose();
    private PathPoint closestPose = new PathPoint();
    private PathPoint previousClosestPose = new PathPoint();
    private Path currentPath = null;
    private PathChain currentPathChain = null;

    private int BEZIER_CURVE_SEARCH_LIMIT;
    private int chainIndex;
    private boolean followingPathChain, holdingPosition, isBusy, isTurning, reachedParametricPathEnd, holdPositionAtEnd, manualDrive;
    private boolean automaticHoldEnd;
    private double globalMaxPower = 1, centripetalScaling;
    private double holdPointTranslationalScaling;
    private double holdPointHeadingScaling;
    private double turnHeadingErrorThreshold;
    private long reachedParametricPathEndTime;
    public boolean useTranslational = true;
    public boolean useCentripetal = true;
    public boolean useHeading = true;
    public boolean useDrive = true;
    private Timer zeroVelocityDetectedTimer = null;
    private Runnable resetFollowing = null;

    /**
     * This creates a new Follower given a HardwareMap.
     * @param constants FollowerConstants to use
     * @param localizer Localizer to use
     * @param drivetrain Drivetrain to use
     * @param pathConstraints PathConstraints to use
     */
    public Follower(FollowerConstants constants, Localizer localizer, Drivetrain drivetrain, PathConstraints pathConstraints) {
        this.constants = constants;
        this.pathConstraints = pathConstraints;

        poseTracker = new PoseTracker(localizer);
        errorCalculator = new ErrorCalculator(constants);
        vectorCalculator = new VectorCalculator(constants);
        this.drivetrain = drivetrain;

        dashboardPoseTracker = new DashboardPoseTracker(poseTracker);

        BEZIER_CURVE_SEARCH_LIMIT = constants.BEZIER_CURVE_SEARCH_LIMIT;
        holdPointTranslationalScaling = constants.holdPointTranslationalScaling;
        holdPointHeadingScaling = constants.holdPointHeadingScaling;
        centripetalScaling = constants.centripetalScaling;
        turnHeadingErrorThreshold = constants.turnHeadingErrorThreshold;
        automaticHoldEnd = constants.automaticHoldEnd;

        breakFollowing();
    }

    public void updateConstants() {
        this.BEZIER_CURVE_SEARCH_LIMIT = constants.BEZIER_CURVE_SEARCH_LIMIT;
        this.holdPointTranslationalScaling = constants.holdPointTranslationalScaling;
        this.holdPointHeadingScaling = constants.holdPointHeadingScaling;
        this.centripetalScaling = constants.centripetalScaling;
        this.turnHeadingErrorThreshold = constants.turnHeadingErrorThreshold;
        this.automaticHoldEnd = constants.automaticHoldEnd;
    }

    /**
     * This creates a new Follower given a HardwareMap.
     * @param constants FollowerConstants to use
     * @param localizer Localizer to use
     * @param drivetrain Drivetrain to use
     */
    public Follower(FollowerConstants constants, Localizer localizer, Drivetrain drivetrain) {
        this(constants, localizer, drivetrain, PathConstraints.defaultConstraints);
    }

    public void setCentripetalScaling(double set) {
        centripetalScaling = set;
    }

    /**
     * This sets the maximum power the motors are allowed to use.
     *
     * @param set This caps the motor power from [0, 1].
     */
    public void setMaxPower(double set) {
        globalMaxPower = set;
        drivetrain.setMaxPowerScaling(set);
    }

    /**
     * This gets a Point from the current Path from a specified t-value.
     *
     * @return returns the Point.
     */
    public Pose getPointFromPath(double t) {
        if (currentPath != null) {
            return currentPath.getPoint(t);
        } else {
            return null;
        }
    }

    /**
     * This sets the current pose in the PoseTracker without using offsets.
     *
     * @param pose The pose to set the current pose to.
     */
    public void setPose(Pose pose) {
        poseTracker.setPose(pose);
    }

    /**
     * This returns the current pose from the PoseTracker.
     *
     * @return returns the pose
     */
    public Pose getPose() {
        return poseTracker.getPose();
    }

    /**
     * This returns the current velocity of the robot as a Vector.
     *
     * @return returns the current velocity as a Vector.
     */
    public Vector getVelocity() {
        return poseTracker.getVelocity();
    }

    /**
     * This sets the starting pose. Do not run this after moving at all.
     *
     * @param pose the pose to set the starting pose to.
     */
    public void setStartingPose(Pose pose) {
        poseTracker.setStartingPose(pose);
    }

    /**
     * This holds a Point.
     *
     * @param point   the Point to stay at.
     * @param heading the heading to face.
     */
    public void holdPoint(BezierPoint point, double heading) {
        breakFollowing();
        holdingPosition = true;
        isBusy = false;
        followingPathChain = false;
        currentPath = new Path(point);
        currentPath.setConstantHeadingInterpolation(heading);
        previousClosestPose = closestPose;
        closestPose = currentPath.updateClosestPose(poseTracker.getPose(), 1);
    }

    /**
     * This holds a Point.
     *
     * @param pose the Point (as a Pose) to stay at.
     */
    public void holdPoint(Pose pose) {
        holdPoint(new BezierPoint(pose), pose.getHeading());
    }

    /**
     * This follows a Path.
     * This also makes the Follower hold the last Point on the Path.
     *
     * @param path the Path to follow.
     * @param holdEnd this makes the Follower hold the last Point on the Path.
     */
    public void followPath(Path path, boolean holdEnd) {
        drivetrain.setMaxPowerScaling(globalMaxPower);
        breakFollowing();
        holdPositionAtEnd = holdEnd;
        isBusy = true;
        followingPathChain = false;
        currentPath = path;
        previousClosestPose = closestPose;
        closestPose = currentPath.updateClosestPose(poseTracker.getPose(), BEZIER_CURVE_SEARCH_LIMIT);
    }

    /**
     * This follows a Path.
     *
     * @param path the Path to follow.
     */
    public void followPath(Path path) {
        followPath(path, automaticHoldEnd);
    }

    /**
     * This follows a PathChain. Drive vector projection is only done on the last Path.
     * This also makes the Follower hold the last Point on the PathChain.
     *
     * @param pathChain the PathChain to follow.
     * @param holdEnd this makes the Follower hold the last Point on the PathChain.
     */
    public void followPath(PathChain pathChain, boolean holdEnd) {
        followPath(pathChain, globalMaxPower, holdEnd);
    }

    /**
     * This follows a PathChain. Drive vector projection is only done on the last Path.
     *
     * @param pathChain the PathChain to follow.
     */
    public void followPath(PathChain pathChain) {
        followPath(pathChain, automaticHoldEnd);
    }

    /**
     * This follows a PathChain. Drive vector projection is only done on the last Path.
     * This also makes the Follower hold the last Point on the PathChain.
     *
     * @param pathChain the PathChain to follow.
     * @param maxPower the max power of the Follower for this path
     * @param holdEnd this makes the Follower hold the last Point on the PathChain.
     */
    public void followPath(PathChain pathChain, double maxPower, boolean holdEnd) {
        drivetrain.setMaxPowerScaling(maxPower);
        breakFollowing();
        holdPositionAtEnd = holdEnd;
        isBusy = true;
        followingPathChain = true;
        chainIndex = 0;
        currentPathChain = pathChain;
        currentPath = pathChain.getPath(chainIndex);
        previousClosestPose = closestPose;
        closestPose = currentPath.updateClosestPose(poseTracker.getPose(), BEZIER_CURVE_SEARCH_LIMIT);
        currentPathChain.resetCallbacks();

        for (PathCallback callback : currentPathChain.getCallbacks()) {
            if (callback.getPathIndex() == chainIndex) {
                callback.initialize();
            }
        }
    }

    /**
     * Resumes pathing, can only be called after pausePathFollowing()
     */
    public void resumePathFollowing() {
        if (resetFollowing != null) {
            resetFollowing.run();
            resetFollowing = null;
            isBusy = true;
            previousClosestPose = closestPose;
            closestPose = currentPath.updateClosestPose(poseTracker.getPose(), BEZIER_CURVE_SEARCH_LIMIT);
        }
    }

    /**
     * Pauses pathing, can only be restarted with resumePathFollowing
     */
    public void pausePathFollowing() {
        isBusy = false;

        boolean prevHoldEnd = holdPositionAtEnd;

        if (followingPathChain && currentPathChain != null) {
            PathChain lastChain = currentPathChain;
            int lastIndex = chainIndex;

            resetFollowing = () -> {
                followingPathChain = true;
                chainIndex = lastIndex;
                currentPathChain = lastChain;
                holdPositionAtEnd = prevHoldEnd;
                currentPath = currentPathChain.getPath(lastIndex);
            };
        } else if (currentPath != null) {
            Path lastPath = currentPath;

            resetFollowing = () -> {
                holdPositionAtEnd = prevHoldEnd;
                currentPath = lastPath;
            };
        }

        holdPoint(getPose());
    }

    /**
     * This starts teleop drive control.
     */
    public void startTeleopDrive() {
        breakFollowing();
        manualDrive = true;
        drivetrain.startTeleopDrive();
    }

    /**
     * This starts teleop drive control.
     */
    public void startTeleopDrive(boolean useBrakeMode) {
        breakFollowing();
        manualDrive = true;
        drivetrain.startTeleopDrive(useBrakeMode);
    }

    /**
     * This sets the Teleop drive movement vectors
     */
    public void setTeleOpDrive(double forward, double strafe, double turn, boolean isRobotCentric) {
        vectorCalculator.setTeleOpMovementVectors(forward, strafe, turn, isRobotCentric);
    }

    /** Updates the Mecanum constants */
    public void updateDrivetrain() {
        drivetrain.updateConstants();
    }

    /** Calls an update to the PoseTracker, which updates the robot's current position estimate. */
    public void updatePose() {
        poseTracker.update();
        currentPose = poseTracker.getPose();
        dashboardPoseTracker.update();
    }

    /** Calls an update to the ErrorCalculator, which updates the robot's current error. */
    public void updateErrors() {
        errorCalculator.update(currentPose, currentPath, currentPathChain, followingPathChain, closestPose.getPose(), poseTracker.getVelocity(), chainIndex, drivetrain.xMovement());
    }

    /** Calls an update to the VectorCalculator, which updates the robot's current vectors to correct. */
    public void updateVectors() {
        vectorCalculator.update(useDrive, useHeading, useTranslational, useCentripetal, manualDrive, chainIndex, drivetrain.getMaxPowerScaling(), followingPathChain, centripetalScaling, currentPose, closestPose.getPose(), poseTracker.getVelocity(), currentPath, currentPathChain, getDriveError(), getTranslationalError(), getHeadingError());
    }

    public void updateErrorAndVectors() {updateErrors(); updateVectors();}


    /**
     * This calls an update to the PoseTracker, which updates the robot's current position estimate.
     * This also updates all the Follower's PIDFs, which updates the motor powers.
     */
    public void update() {
        dashboardPoseTracker.update();
        updateConstants();
        updatePose();
        updateDrivetrain();

        if (manualDrive) {
            previousClosestPose = closestPose;
            closestPose = new PathPoint();
            updateErrorAndVectors();
            drivetrain.getAndRunDrivePowers(getCentripetalForceCorrection(), getTeleopHeadingVector(), getTeleopDriveVector(), poseTracker.getPose().getHeading());
            return;
        }

        if (currentPath == null) {
            return;
        }

        if (holdingPosition) {
            previousClosestPose = closestPose;
            closestPose = currentPath.updateClosestPose(poseTracker.getPose(), 1);
            currentPath.updateDistance(previousClosestPose,closestPose);
            updateErrorAndVectors();
            drivetrain.getAndRunDrivePowers(getTranslationalCorrection().times(holdPointTranslationalScaling), getHeadingVector().times(holdPointHeadingScaling), new Vector(), poseTracker.getPose().getHeading());

            if(getHeadingError() < turnHeadingErrorThreshold && isTurning) {
                isTurning = false;
                isBusy = false;
            }
            return;
        }

        if (isBusy) {
            previousClosestPose = closestPose;
            closestPose = currentPath.updateClosestPose(poseTracker.getPose(), BEZIER_CURVE_SEARCH_LIMIT);
            currentPath.updateDistance(previousClosestPose, closestPose);
            if (followingPathChain) updateCallbacks();

            updateErrorAndVectors();
            drivetrain.getAndRunDrivePowers(getCorrectiveVector(), getHeadingVector(), getDriveVector(), poseTracker.getPose().getHeading());
        }

        if (poseTracker.getVelocity().getMagnitude() < 1.0 && currentPath.getClosestPointTValue() > 0.8
                && zeroVelocityDetectedTimer == null && isBusy) {
            zeroVelocityDetectedTimer = new Timer();
        }

        if (!(currentPath.isAtParametricEnd() || ( zeroVelocityDetectedTimer != null && zeroVelocityDetectedTimer.getElapsedTime() > 500.0))) {
            return;
        }

        if (followingPathChain && chainIndex < currentPathChain.size() - 1) {
            breakFollowing();
            isBusy = true;
            followingPathChain = true;
            chainIndex++;
            currentPath = currentPathChain.getPath(chainIndex);
            previousClosestPose = closestPose;
            closestPose = currentPath.updateClosestPose(poseTracker.getPose(), BEZIER_CURVE_SEARCH_LIMIT);
            updateErrorAndVectors();

            for (PathCallback callback : currentPathChain.getCallbacks()) {
                if (callback.getPathIndex() == chainIndex) {
                    callback.initialize();
                }
            }

            return;
        }

        if (!reachedParametricPathEnd) {
            reachedParametricPathEnd = true;
            reachedParametricPathEndTime = System.currentTimeMillis();
        }

        updateErrorAndVectors();
        if (!(
            (
                System.currentTimeMillis() - reachedParametricPathEndTime
                > currentPath.getPathEndTimeoutConstraint()
            )
            || (
                poseTracker.getVelocity().getMagnitude()
                < currentPath.getPathEndVelocityConstraint()
            )
            && (
                poseTracker.getPose().distanceFrom(closestPose.getPose())
                < currentPath.getPathEndTranslationalConstraint()
            )
            && (
                MathFunctions.getSmallestAngleDifference(poseTracker.getPose().getHeading(), currentPath.getClosestPointHeadingGoal())
                < currentPath.getPathEndHeadingConstraint()
            )
        )) {
            return;
        }
        if (holdPositionAtEnd) {
            holdPositionAtEnd = false;
            holdPoint(new BezierPoint(currentPath.getLastControlPoint()), currentPath.getHeadingGoal(1));
        } else {
            breakFollowing();
        }
    }

    /** This checks if any PathCallbacks should be run right now, and runs them if applicable. */
    public void updateCallbacks() {
        for (PathCallback callback : currentPathChain.getCallbacks()) {
            if (!callback.isCompleted() && callback.isReady()) {
                callback.run();
            }
        }
    }

    /** This resets the PIDFs and stops following the current Path. */
    public void breakFollowing() {
        errorCalculator.breakFollowing();
        vectorCalculator.breakFollowing();
        drivetrain.breakFollowing();
        manualDrive = false;
        holdingPosition = false;
        isBusy = false;
        reachedParametricPathEnd = false;
        zeroVelocityDetectedTimer = null;
    }

    /**
     * This returns if the Follower is currently following a Path or a PathChain.
     * @return returns if the Follower is busy.
     */
    public boolean isBusy() {
        return isBusy;
    }

    /**
     * This returns the closest pose to the robot on the Path the Follower is currently following.
     * This closest pose is calculated through a binary search method with some specified number of
     * steps to search. By default, 10 steps are used, which should be more than enough.
     * @return returns the closest pose.
     */
    public PathPoint getClosestPose() {
        return closestPose;
    }

    /**
     * This returns whether the follower is at the parametric end of its current Path.
     * The parametric end is determined by if the closest Point t-value is greater than some specified
     * end t-value.
     * If running a PathChain, this returns true only if at parametric end of last Path in the PathChain.
     * @return returns whether the Follower is at the parametric end of its Path.
     */
    public boolean atParametricEnd() {
        if (followingPathChain) {
            if (chainIndex == currentPathChain.size() - 1) return currentPath.isAtParametricEnd();
            return false;
        }
        return currentPath.isAtParametricEnd();
    }

    /**
     * This returns the t value of the closest point on the current Path to the robot
     * In the absence of a current Path, it returns 1.0.
     * @return returns the current t value.
     */
    public double getCurrentTValue() {
        if (isBusy) return currentPath.getClosestPointTValue();
        return 1.0;
    }

    /**
     * This returns the current path number. For following Paths, this will return 0. For PathChains,
     * this will return the current path number. For holding Points, this will also return 0.
     * @return returns the current path number.
     */
    public double getCurrentPathNumber() {
        if (!followingPathChain) return 0;
        return chainIndex;
    }

    /**
     * This returns a new PathBuilder object for easily building PathChains.
     * @return returns a new PathBuilder object.
     */
    public static PathBuilder pathBuilder(PathConstraints constraints) {
        return new PathBuilder(constraints);
    }

    /**
     * This returns a new PathBuilder object for easily building PathChains.
     * @return returns a new PathBuilder object.
     */
    public static PathBuilder pathBuilder() {
        return new PathBuilder();
    }

    /**
     * This returns the total number of radians the robot has turned.
     * @return the total heading.
     */
    public double getTotalHeading() {
        return poseTracker.getTotalHeading();
    }

    /**
     * This returns the current Path the Follower is following. This can be null.
     * @return returns the current Path.
     */
    public Path getCurrentPath() {
        return currentPath;
    }

    //Thanks to team 21229 Quality Control for creating this algorithm to detect if the robot is stuck.
    /** @return true if the robot is stuck and false otherwise */
    public boolean isRobotStuck() {
        return zeroVelocityDetectedTimer != null;
    }

    public boolean isLocalizationNAN() {
        return poseTracker.getLocalizer().isNAN();
    }

    /** Turns a certain amount of degrees left
     * @param radians the amount of radians to turn
     * @param isLeft true if turning left, false if turning right
     */
    public void turn(double radians, boolean isLeft) {
        Pose temp = new Pose(getPose().getX(), getPose().getY(), getPose().getHeading() + (isLeft ? radians : -radians));
        holdPoint(temp);
        isTurning = true;
        isBusy = true;
    }

    /** Turns to a specific heading
     * @param radians the heading in radians to turn to
     */
    public void turnTo(double radians) {
        holdPoint(new Pose(getPose().getX(), getPose().getY(), radians));
        isTurning = true;
        isBusy = true;
    }

    /** Turns to a specific heading in degrees
     * @param degrees the heading in degrees to turn to
     */
    public void turnToDegrees(double degrees) {
        turnTo(Math.toRadians(degrees));
    }

    /** Turns a certain amount of degrees left
     * @param degrees the amount of degrees to turn
     * @param isLeft true if turning left, false if turning right
     */
    public void turnDegrees(double degrees, boolean isLeft) {
        turn(Math.toRadians(degrees), isLeft);
    }

    public boolean isTurning() {
        return isTurning;
    }

    /**
     * Checks if the robot is at a certain pose within certain tolerances
     * @param pose Pose to compare with the current pose
     * @param xTolerance Tolerance for the x position
     * @param yTolerance Tolerance for the y position
     * @param headingTolerance Tolerance for the heading
     */
    public boolean atPose(Pose pose, double xTolerance, double yTolerance, double headingTolerance) {
        return Math.abs(pose.getX() - getPose().getX()) < xTolerance && Math.abs(pose.getY() - getPose().getY()) < yTolerance && Math.abs(pose.getHeading() - getPose().getHeading()) < headingTolerance;
    }

    /**
     * Checks if the robot is at a certain pose within certain tolerances
     * @param pose Pose to compare with the current pose
     * @param xTolerance Tolerance for the x position
     * @param yTolerance Tolerance for the y position
     */
    public boolean atPose(Pose pose, double xTolerance, double yTolerance) {
        return Math.abs(pose.getX() - getPose().getX()) < xTolerance && Math.abs(pose.getY() - getPose().getY()) < yTolerance;
    }

    /**
     * Sets the maximum power that can be used by the Drivetrain.
     * @param maxPowerScaling setting the max power scaling
     */
    public void setMaxPowerScaling(double maxPowerScaling) {
        drivetrain.setMaxPowerScaling(maxPowerScaling);
    }

    /**
     * Gets the maximum power that can be used by the drive vector scaler. Ranges between 0 and 1.
     * @return returns the max power scaling
     */
    public double getMaxPowerScaling() {
        return drivetrain.getMaxPowerScaling();
    }

    /** Returns the useDrive boolean */
    public boolean getUseDrive() { return useDrive; }

    /** Returns the useHeading boolean */
    public boolean getUseHeading() { return useHeading; }

    /** Returns the useTranslational boolean */
    public boolean getUseTranslational() { return useTranslational; }

    /** Returns the useCentripetal boolean */
    public boolean getUseCentripetal() { return useCentripetal; }

    /** Return the teleopDrive boolean */
    public boolean getTeleopDrive() { return manualDrive; }

    /** Returns the chainIndex of the current PathChain */
    public int getChainIndex() { return chainIndex; }

    /** Returns the current PathChain */
    public PathChain getCurrentPathChain() { return currentPathChain; }

    /** Returns if following a path chain */
    public boolean getFollowingPathChain() { return followingPathChain; }

    /** Return the centripetal scaling */
    public double getCentripetalScaling() { return centripetalScaling; }

    public boolean isTeleopDrive() { return manualDrive; }
    public Vector getTeleopHeadingVector() { return vectorCalculator.getTeleopHeadingVector(); }
    public Vector getTeleopDriveVector() { return vectorCalculator.getTeleopDriveVector(); }
    public double getHeadingError() { return errorCalculator.getHeadingError(); }
    public Vector getTranslationalError() { return errorCalculator.getTranslationalError(); }
    public double getDriveError() { return errorCalculator.getDriveError(); }
    public Vector getDriveVector() { return vectorCalculator.getDriveVector(); }
    public Vector getCorrectiveVector() { return vectorCalculator.getCorrectiveVector(); }
    public Vector getHeadingVector() { return vectorCalculator.getHeadingVector(); }
    public Vector getTranslationalCorrection() { return vectorCalculator.getTranslationalCorrection(); }
    public Vector getCentripetalForceCorrection() { return vectorCalculator.getCentripetalForceCorrection(); }
    public PathConstraints getConstraints() { return pathConstraints; }
    public FollowerConstants getConstants() { return constants; }
    public void setConstraints(PathConstraints pathConstraints) { this.pathConstraints = pathConstraints; }
    public Drivetrain getDrivetrain() { return drivetrain; }
    public PoseTracker getPoseTracker() { return poseTracker; }
    public ErrorCalculator getErrorCalculator() { return errorCalculator; }
    public VectorCalculator getVectorCalculator() { return vectorCalculator; }
    public DashboardPoseTracker getDashboardPoseTracker() { return dashboardPoseTracker; }
    public void setXMovement(double xMovement) { drivetrain.setXMovement(xMovement); }
    public void setYMovement(double yMovement) { drivetrain.setYMovement(yMovement); }

    public void setDrivePIDFCoefficients(FilteredPIDFCoefficients drivePIDFCoefficients) { vectorCalculator.setDrivePIDFCoefficients(drivePIDFCoefficients); }
    public void setSecondaryDrivePIDFCoefficients(FilteredPIDFCoefficients secondaryDrivePIDFCoefficients) { vectorCalculator.setSecondaryDrivePIDFCoefficients(secondaryDrivePIDFCoefficients); }
    public void setHeadingPIDFCoefficients(PIDFCoefficients headingPIDFCoefficients) { vectorCalculator.setHeadingPIDFCoefficients(headingPIDFCoefficients); }
    public void setSecondaryHeadingPIDFCoefficients(PIDFCoefficients secondaryHeadingPIDFCoefficients) { vectorCalculator.setSecondaryHeadingPIDFCoefficients(secondaryHeadingPIDFCoefficients); }
    public void setTranslationalPIDFCoefficients(PIDFCoefficients translationalPIDFCoefficients) { vectorCalculator.setTranslationalPIDFCoefficients(translationalPIDFCoefficients); }
    public void setSecondaryTranslationalPIDFCoefficients(PIDFCoefficients secondaryTranslationalPIDFCoefficients) { vectorCalculator.setSecondaryTranslationalPIDFCoefficients(secondaryTranslationalPIDFCoefficients); }
    public void setConstants(FollowerConstants constants) {
        this.constants = constants;
        updateConstants();
        errorCalculator.setConstants(constants);
        vectorCalculator.setConstants(constants);
        drivetrain.updateConstants();
    }
    private double getHeadingGoal(double t) {
        if (currentPathChain != null) {
            return currentPathChain.getHeadingGoal(new PathChain.PathT(chainIndex, t));
        }

        return currentPath.getHeadingGoal(t);
    }

    private double getHeadingGoal(PathPoint point) {
        if (currentPathChain != null) {
            return currentPathChain.getHeadingGoal(new PathChain.PathT(chainIndex, point.tValue));
        }

        return currentPath.getHeadingGoal(point);
    }

    public Vector getClosestPointTangentVector() {
        return getClosestPose().getTangentVector();
    }

    public void activateAllPIDFs() {
        useDrive = true;
        useHeading = true;
        useTranslational = true;
        useCentripetal = true;
    }

    public void deactivateAllPIDFs() {
        useDrive = false;
        useHeading = false;
        useTranslational = false;
        useCentripetal = false;
    }

    public void activateDrive() { useDrive = true; }
    public void activateHeading() { useHeading = true; }
    public void activateTranslational() { useTranslational = true; }
    public void activateCentripetal() { useCentripetal = true; }
    public double getDistanceTraveledOnPath() {
        return currentPath.getDistanceTraveled();
    }
    public double getPathCompletion() {
        return currentPath.getPathCompletion();
    }

    /**
     * This is a debugging method that returns a String array of debug information.
     */
    public String[] debug() {
        String[] info = new String[4];
        info[0] = poseTracker.debugString();
        info[1] = errorCalculator.debugString();
        info[2] = vectorCalculator.debugString();
        info[3] = drivetrain.debugString();
        return info;
    }
}