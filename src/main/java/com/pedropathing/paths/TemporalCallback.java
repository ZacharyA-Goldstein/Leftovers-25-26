package com.pedropathing.paths;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

/**
 * This is the TemporalCallback class. This class handles callbacks that occur a given amount of time after a Path in a PathChain has begun running.
 *
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
public class TemporalCallback implements PathCallback {
    private double startCondition;
    private Runnable runnable;
    private double startTime = 0;
    private int index;

    /**
     * This creates a new TemporalCallback from an index, start condition, and runnable.
     * @param index the index of the path in the path chain
     * @param startCondition the time since the start of the path to run the callback
     * @param runnable the runnable to run
     */
    public TemporalCallback(int index, double startCondition, Runnable runnable) {
        this.index = index;
        this.startCondition = startCondition;
        this.runnable = runnable;
    }

    /**
     * This runs the callback.
     * @return true if the action was successful
     */
    @Override
    public boolean run() {
        runnable.run();
        return true;
    }

    /**
     * This checks if the callback is ready to run.
     * @return true if the callback is ready to run
     */
    @Override
    public boolean isReady() {
        return TimeUnit.NANOSECONDS.convert(System.nanoTime(), TimeUnit.MILLISECONDS) - startTime >= startCondition;
    }

    /**
     * This initializes the callback.
     */
    @Override
    public void initialize() {
        startTime = TimeUnit.NANOSECONDS.convert(System.nanoTime(), TimeUnit.MILLISECONDS);
    }

    @Override
    public int getPathIndex() {
        return index;
    }
}
