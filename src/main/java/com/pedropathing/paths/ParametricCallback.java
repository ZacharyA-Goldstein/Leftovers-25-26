package com.pedropathing.paths;

import com.pedropathing.follower.Follower;

/**
 * This is the ParametricCallback class. This class handles callbacks that occur a given amount of time after a Path in a PathChain has begun running.
 *
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
public class ParametricCallback implements PathCallback {
    private Follower follower;
    private double startCondition;
    private Runnable runnable;
    private int index;

    /**
     * Constructor for the ParametricCallback.
     * @param index The index of the path that this callback is for
     * @param startCondition The time at which the callback should occur
     * @param follower The follower that is running the path
     * @param runnable The runnable that will be run when the callback is triggered
     */
    public ParametricCallback(int index, double startCondition, Follower follower, Runnable runnable) {
        this.index = index;
        this.follower = follower;
        this.startCondition = startCondition;
        this.runnable = runnable;
    }

    /**
     * This method runs the callback.
     * @return true if the action was successful
     */
    @Override
    public boolean run() {
        runnable.run();
        return true;
    }

    /**
     * This method checks if the callback is ready to run.
     * @return if the callback is ready to run
     */
    @Override
    public boolean isReady() {
        return follower.getCurrentTValue() >= startCondition;
    }

    /**
     * This method returns the index of the path that this callback is for.
     * @return the index of the path that this callback is for
     */
    @Override
    public int getPathIndex() {
        return index;
    }
}
