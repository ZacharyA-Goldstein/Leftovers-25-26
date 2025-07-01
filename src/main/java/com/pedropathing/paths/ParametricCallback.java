package com.pedropathing.paths;

import com.pedropathing.follower.Follower;

/**
 * This is the ParametricCallback class. This class handles callbacks that occur after a certain t-value of the Path in a PathChain has begun achieved.
 *
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
public class ParametricCallback implements PathCallback {
    private final Follower follower;
    private final double startCondition;
    private final Runnable runnable;
    private final int index;

    public ParametricCallback(int index, double startCondition, Follower follower, Runnable runnable) {
        this.index = index;
        this.follower = follower;
        this.startCondition = startCondition;
        this.runnable = runnable;
    }

    @Override
    public boolean run() {
        runnable.run();
        return true;
    }

    @Override
    public boolean isReady() {
        return follower.getCurrentTValue() >= startCondition;
    }

    @Override
    public int getPathIndex() {
        return index;
    }
}
