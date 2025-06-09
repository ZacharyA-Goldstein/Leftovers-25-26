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
        decelerationStartMultiplier = constraints.decelerationStartMultiplier;

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
        decelerationStartMultiplier = constraints.decelerationStartMultiplier;

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

    public void setDecelerationType(DecelerationType decelerationType) {
        this.decelerationType = decelerationType;
    }

    public DecelerationType getDecelerationType() {
        return decelerationType;
    }

    public double length() {
        return length;
    }

    public void setDecelerationStartMultiplier(double decelerationStartMultiplier) {
        this.decelerationStartMultiplier = decelerationStartMultiplier;
    }

    public double getDecelerationStartMultiplier() {
        return decelerationStartMultiplier;
    }

    public Pose endPose() {
        Path last = pathChain.get(pathChain.size() - 1);
        return last.endPose();
    }

    public Pose endPoint() {
        Path last = pathChain.get(pathChain.size() - 1);
        return last.getLastControlPoint();
    }

    public void setConstraintsForAll(PathConstraints constraints) {
        this.constraints = constraints;
        for (Path path : pathChain) {
            path.setConstraints(constraints);
        }
    }

    public static class PathT {
        final int pathIndex;
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

        public int getPathIndex() {
            return pathIndex;
        }

        public double getT() {
            return t;
        }

        public Path getPath(PathChain pathChain) {
            return pathChain.getPath(pathIndex);
        }

        public Pose getPose(PathChain pathChain) {
            return pathChain.getPath(pathIndex).getPoint(t);
        }

        public Vector getTangentVector(PathChain pathChain) {
            return pathChain.getPath(pathIndex).getTangentVector(t);
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

    public void setHeadingInterpolator(HeadingInterpolator headingInterpolator) {
        this.headingInterpolator = headingInterpolator;
    }

    public double getHeadingGoal(PathT pathTValue) {
        if (headingInterpolator != null) {
            double sumLength = 0;
            for (int i = 0; i < pathTValue.pathIndex; i++) {
                sumLength += pathChain.get(i).length();
            }

            double pathInitialTValue = sumLength / length;
            double chainT = pathInitialTValue + pathTValue.t * pathChain.get(pathTValue.pathIndex).length() / length;
            return headingInterpolator.interpolate(new PathPoint(chainT, pathTValue.getPose(this), pathTValue.getTangentVector(this)));
        }

        return pathTValue.getPath(this).getHeadingGoal(pathTValue.t);
    }
}
