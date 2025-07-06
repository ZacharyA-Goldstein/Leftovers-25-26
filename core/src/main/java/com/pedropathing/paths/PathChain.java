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
 * @author Havish Sripada - 12808 RevAmped Robotics
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
    private double decelerationStart;
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
        decelerationStart = constraints.getDecelerationStart();

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
        decelerationStart = constraints.getDecelerationStart();

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

    /**
     * This resets the PathCallbacks of this PathChain.
     */

    public void resetCallbacks() {
        for (PathCallback callback : callbacks) {
            callback.reset();
        }
    }

    /**
     * This sets the deceleration type of the PathChain.
     * @param decelerationType the deceleration type to set
     */
    public void setDecelerationType(DecelerationType decelerationType) {
        this.decelerationType = decelerationType;
    }

    /**
     * This returns the deceleration type of the PathChain.
     * @return the deceleration type of the PathChain
     */
    public DecelerationType getDecelerationType() {
        return decelerationType;
    }

    /**
     * This returns the length of the PathChain.
     * @return the length of the PathChain
     */
    public double length() {
        return length;
    }

    /**
     * This sets the deceleration start multiplier of the PathChain.
     * @param decelerationStart the deceleration start multiplier to set
     */
    public void setDecelerationStart(double decelerationStart) {
        this.decelerationStart = decelerationStart;
    }

    /**
     * This returns the deceleration start multiplier of the PathChain.
     * @return the deceleration start multiplier of the PathChain
     */
    public double getDecelerationStart() {
        return decelerationStart;
    }

    /**
     * This returns the end pose of the PathChain.
     * @return the end pose of the PathChain
     */
    public Pose endPose() {
        Path last = pathChain.get(pathChain.size() - 1);
        return last.endPose();
    }

    /**
     * This returns the end point of the PathChain.
     * @return the end point of the PathChain
     */
    public Pose endPoint() {
        Path last = pathChain.get(pathChain.size() - 1);
        return last.getLastControlPoint();
    }

    /**
     * This sets the constraints for all Paths in the PathChain.
     * @param constraints the constraints to set
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
         * This creates a new Path and T-Value pair from a path index and a t value.
         * @param pathIndex this specifies the index of the path in the chain
         * @param t this is the t-value of the point in the path
         */
        public PathT(int pathIndex, double t) {
            this.pathIndex = pathIndex;
            this.t = t;
        }

        /**
         * This returns the index of the path in the chain.
         * @return the index of the path in the chain
         */
        public int getPathIndex() {
            return pathIndex;
        }

        /**
         * This returns the t-value of the point in the path.
         * @return the t-value of the point in the path
         */
        public double getT() {
            return t;
        }

        /**
         * This returns the path in the chain.
         * @return the path in the chain
         */
        public Path getPath(PathChain pathChain) {
            return pathChain.getPath(pathIndex);
        }

        /**
         * This returns the pose of the point in the path.
         * @param pathChain this is the path chain
         * @return the pose of the point in the path
         */
        public Pose getPose(PathChain pathChain) {
            return pathChain.getPath(pathIndex).getPose(t);
        }

        /**
         * This returns the point in the path.
         * @param pathChain this is the path chain
         * @return the point in the path
         */
        public Pose getPoint(PathChain pathChain) {
            return pathChain.getPath(pathIndex).getPoint(t);
        }

        /**
         * This returns the tangent vector of the point in the path.
         * @param pathChain this is the path chain
         * @return the tangent vector of the point in the path
         */
        public Vector getTangentVector(PathChain pathChain) {
            return pathChain.getPath(pathIndex).getTangentVector(t);
        }

        /**
         * This returns the heading goal of the path.
         * @param pathChain this is the path chain
         * @return the heading goal of the path
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
     * Sets the path's heading interpolation
     * @param headingInterpolator the heading interpolation to set
     */
    public void setHeadingInterpolator(HeadingInterpolator headingInterpolator) {
        this.headingInterpolator = headingInterpolator;
    }

    /**
     * This returns the heading goal of the path.
     * @param pathTValue this is the path and t-value
     * @return the heading goal of the path
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
     * This returns the tangent vector of the point in the path.
     * @param pathTValue this is the path and t-value
     * @return the tangent vector of the point in the path
     */
    public Vector getTangentVector(PathT pathTValue) {
        return pathTValue.getTangentVector(this);
    }

    /**
     * This returns the pose of the point in the path.
     * @param pathTValue this is the path and t-value
     * @return the pose of the point in the path
     */
    public Pose getPose(PathT pathTValue) {
        return pathTValue.getPose(this);
    }

    /**
     * This returns the point in the path, without a heading goal.
     * @param pathTValue this is the path and t-value
     * @return the point in the path
     */
    public Pose getPoint(PathT pathTValue) {
        return pathTValue.getPoint(this);
    }

    /**
     * This returns the pose information of the point in the path (with the heading goal).
     * @param pathTValue this is the path and t-value
     * @return the pose information of the point in the path
     */
    public PathPoint getPoseInformation(PathT pathTValue) {
        return new PathPoint(pathTValue.getT(), pathTValue.getPose(this), pathTValue.getTangentVector(this));
    }
}
