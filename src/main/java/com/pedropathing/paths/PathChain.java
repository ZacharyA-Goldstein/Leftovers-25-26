package com.pedropathing.paths;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * This is the PathChain class. This class handles chaining together multiple Paths into a larger
 * collection of Paths that can be run continuously. Additionally, this allows for the addition of
 * PathCallbacks to specific Paths in the PathChain, allowing for non-blocking code to be run in
 * the middle of a PathChain.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/11/2024
 */
public class PathChain {
    private PathConstraints constraints;
    private ArrayList<Path> pathChain = new ArrayList<>();
    private double length = 0;

    public enum DecelerationType {
        NONE,
        GLOBAL,
        LAST_PATH
    }
    private DecelerationType decelerationType = DecelerationType.LAST_PATH;
    private double decelerationStartMultiplier;
    private ArrayList<PathCallback> callbacks = new ArrayList<>();
    public HeadingInterpolator headingInterpolator = null;

    /**
     * This creates a new PathChain from some specified Paths.
     *
     * IMPORTANT NOTE: Order matters here. The order in which the Paths are input is the order in
     * which they will be run.
     *
     * @param paths the specified Paths.
     */
    public PathChain(Path... paths) {
        this(PathConstraints.defaultConstraints, paths);
    }

    /**
     * This creates a new PathChain from some specified Paths and a PathConstraints.
     *
     * IMPORTANT NOTE: Order matters here. The order in which the Paths are input is the order in
     * which they will be run.
     *
     * @param paths the specified Paths.
     * @param constraints the PathConstraints for the PathChain.
     */
    public PathChain(PathConstraints constraints, Path... paths) {
        this.constraints = constraints;
        decelerationStartMultiplier = constraints.getDecelerationStartMultiplier();

        for (Path path : paths) {
            path.setConstraints(constraints);
            pathChain.add(path);
            length += path.length();
        }
    }

    /**
     * This creates a new PathChain from an ArrayList of Paths.
     *
     * IMPORTANT NOTE: Order matters here. The order in which the Paths are input is the order in
     * which they will be run.
     *
     * @param paths the ArrayList of Paths.
     */
    public PathChain(ArrayList<Path> paths) {
        this(PathConstraints.defaultConstraints, paths);
    }

    /**
     * This creates a new PathChain from an ArrayList of Paths.
     *
     * IMPORTANT NOTE: Order matters here. The order in which the Paths are input is the order in
     * which they will be run.
     *
     * @param paths the ArrayList of Paths.
     * @param constraints the PathConstraints for the PathChain.
     */
    public PathChain(PathConstraints constraints, ArrayList<Path> paths) {
        this.constraints = constraints;
        decelerationStartMultiplier = constraints.getDecelerationStartMultiplier();

        for (Path path : paths) {
            path.setConstraints(constraints);
        }

        pathChain = paths;
    }

    /**
     * This returns the Path on the PathChain at a specified index.
     *
     * @param index the index.
     * @return returns the Path at the index.
     */
    public Path getPath(int index) {
        return pathChain.get(index);
    }

    /**
     * This returns the size of the PathChain.
     *
     * @return returns the size of the PathChain.
     */
    public int size() {
        return pathChain.size();
    }

    /**
     * This sets the PathCallbacks of the PathChain with some specified PathCallbacks.
     *
     * @param callbacks the specified PathCallbacks.
     */
    public void setCallbacks(PathCallback... callbacks) {
        this.callbacks.addAll(Arrays.asList(callbacks));
    }

    /**
     * This sets the PathCallbacks of the PathChain with an ArrayList of PathCallbacks.
     *
     * @param callbacks the ArrayList of PathCallbacks.
     */
    public void setCallbacks(ArrayList<PathCallback> callbacks) {
        this.callbacks = callbacks;
    }

    /**
     * This returns the PathCallbacks of this PathChain in an ArrayList.
     *
     * @return returns the PathCallbacks.
     */
    public ArrayList<PathCallback> getCallbacks() {
        return callbacks;
    }

    public void resetCallbacks() {
        for (PathCallback callback : callbacks) {
            callback.reset();
        }
    }

    /**
     * Sets the deceleration type of the PathChain. This determines how the robot will decelerate
     * @param decelerationType the type of deceleration to use
     */
    public void setDecelerationType(DecelerationType decelerationType) {
        this.decelerationType = decelerationType;
    }

    /**
     * This returns the deceleration type of the PathChain. This determines how the robot will decelerate
     *
     * @return returns the deceleration type.
     */
    public DecelerationType getDecelerationType() {
        return decelerationType;
    }

    /**
     * This returns the length of the PathChain.
     *
     * @return returns the length of the PathChain.
     */
    public double length() {
        return length;
    }

    /**
     * This sets the deceleration start multiplier of the PathChain. This determines how far from the end
     * of the PathChain the robot will start decelerating.
     *
     * @param decelerationStartMultiplier the multiplier for the deceleration start point.
     */
    public void setDecelerationStartMultiplier(double decelerationStartMultiplier) {
        this.decelerationStartMultiplier = decelerationStartMultiplier;
    }

    /**
     * This returns the deceleration start multiplier of the PathChain. This determines how far from the end
     * of the PathChain the robot will start decelerating.
     *
     * @return returns the deceleration start multiplier.
     */
    public double getDecelerationStartMultiplier() {
        return decelerationStartMultiplier;
    }

    /**
     * This returns the end Pose of the PathChain.
     *
     * @return returns the end Pose of the PathChain.
     */
    public Pose endPose() {
        Path last = pathChain.get(pathChain.size() - 1);
        return last.endPose();
    }

    /**
     * This returns the end Point of the PathChain.
     *
     * @return returns the end Point of the PathChain.
     */
    public Pose endPoint() {
        Path last = pathChain.get(pathChain.size() - 1);
        return last.getLastControlPoint();
    }

    /**
     * This sets the PathConstraints for all Paths in the PathChain.
     * @param constraints the PathConstraints to set for all Paths in the PathChain
     */
    public void setConstraintsForAll(PathConstraints constraints) {
        this.constraints = constraints;
        for (Path path : pathChain) {
            path.setConstraints(constraints);
        }
    }

    /**
     * Represents a specific point within a {@link PathChain}, defined by a path index and a t-value (parametric position).
     * Provides utility methods to access the corresponding path, pose, point, tangent vector, and heading goal.
     */
    public static class PathT {
        /** The index of the path in the chain. */
        final int pathIndex;
        /** The t-value (parametric position) within the path. */
        final double t;

        /**
         * Constructs a new PathT with the specified path index and t-value.
         * @param pathIndex the index of the path in the chain
         * @param t the t-value (parametric position) within the path
         */
        public PathT(int pathIndex, double t) {
            this.pathIndex = pathIndex;
            this.t = t;
        }

        /**
         * Returns the index of the path in the chain.
         * @return the path index
         */
        public int getPathIndex() {
            return pathIndex;
        }

        /**
         * Returns the t-value (parametric position) within the path.
         * @return the t-value
         */
        public double getT() {
            return t;
        }

        /**
         * Returns the {@link Path} object at the specified path index in the given {@link PathChain}.
         * @param pathChain the PathChain containing the path
         * @return the Path at the specified index
         */
        public Path getPath(PathChain pathChain) {
            return pathChain.getPath(pathIndex);
        }

        /**
         * Returns the pose at this t-value within the specified path in the given {@link PathChain}.
         * @param pathChain the PathChain containing the path
         * @return the pose at the specified t-value
         */
        public Pose getPose(PathChain pathChain) {
            return pathChain.getPath(pathIndex).getPose(t);
        }

        /**
         * Returns the point at this t-value within the specified path in the given {@link PathChain}.
         * @param pathChain the PathChain containing the path
         * @return the point at the specified t-value
         */
        public Pose getPoint(PathChain pathChain) {
            return pathChain.getPath(pathIndex).getPoint(t);
        }

        /**
         * Returns the tangent vector at this t-value within the specified path in the given {@link PathChain}.
         * @param pathChain the PathChain containing the path
         * @return the tangent vector at the specified t-value
         */
        public Vector getTangentVector(PathChain pathChain) {
            return pathChain.getPath(pathIndex).getTangentVector(t);
        }

        /**
         * Returns the heading goal at this t-value within the specified path in the given {@link PathChain}.
         * @param pathChain the PathChain containing the path
         * @return the heading goal at the specified t-value
         */
        public double getHeadingGoal(PathChain pathChain) {
            return pathChain.getPath(pathIndex).getHeadingGoal(t);
        }
    }

    /**
     * This gets the path that corresponds to the given completion amount of the chain
     * @param t completion of the PathChain from [0,1] based on distance traveled
     */
    private PathT chainCompletionToPath(double t) {
        double lengthSum = 0;
        double currentT = 1;
        for (int i = 0; i < pathChain.size(); i++) {
            lengthSum += pathChain.get(i).length();

            if (lengthSum > length) {
                return new PathT(i, currentT);
            }

            currentT = t - lengthSum / length;
        }

        return new PathT(pathChain.size()-1, currentT);
    }


    /**
     * Sets the HeadingInterpolator for this PathChain.
     * The HeadingInterpolator determines how the heading is interpolated along the PathChain.
     *
     * @param headingInterpolator the HeadingInterpolator to use for heading interpolation.
     */
    public void setHeadingInterpolator(HeadingInterpolator headingInterpolator) {
        this.headingInterpolator = headingInterpolator;
    }

    /**
     * Returns the heading goal at the specified {@link PathT} value.
     * If a {@link HeadingInterpolator} is set, it interpolates the heading along the chain using the interpolator.
     * Otherwise, it returns the heading goal from the path at the given t-value.
     *
     * @param pathTValue the {@link PathT} value specifying the path index and t-value within the path
     * @return the heading goal (in radians) at the specified point in the path chain
     */
    public double getHeadingGoal(PathT pathTValue) {
        if (headingInterpolator != null) {
            double sumLength = 0;
            for (int i = 0; i < pathTValue.pathIndex; i++) {
                sumLength += pathChain.get(i).length();
            }

            double pathInitialTValue = sumLength / length;
            double chainT = pathInitialTValue + pathTValue.t * pathChain.get(pathTValue.pathIndex).length() / length;
            return headingInterpolator.interpolate(new PathPoint(chainT, pathTValue.getPoint(this), pathTValue.getTangentVector(this)));
        }

        return pathTValue.getHeadingGoal(this);
    }

    /**
     * Returns the pose at the specified {@link PathT} value in the PathChain.
     *
     * @param pathTValue the {@link PathT} value specifying the path index and t-value within the path
     * @return the pose at the specified point in the path chain
     */
    public Pose getPose(PathT pathTValue) {
        return pathTValue.getPose(this);
    }

    /**
     * Returns the point at the specified {@link PathT} value in the PathChain.
     *
     * @param pathTValue the {@link PathT} value specifying the path index and t-value within the path
     * @return the point at the specified location in the path chain
     */
    public Pose getPoint(PathT pathTValue) {
        return pathTValue.getPoint(this);
    }

    /**
     * Returns a {@link PathPoint} containing detailed pose information at the specified {@link PathT} value.
     * The returned PathPoint includes the t-value, pose, and tangent vector at the given point in the PathChain.
     *
     * @param pathTValue the {@link PathT} value specifying the path index and t-value within the path
     * @return a {@link PathPoint} with t-value, pose, and tangent vector at the specified location
     */
    public PathPoint getPoseInformation(PathT pathTValue) {
        return new PathPoint(pathTValue.getT(), pathTValue.getPose(this), pathTValue.getTangentVector(this));
    }
}
