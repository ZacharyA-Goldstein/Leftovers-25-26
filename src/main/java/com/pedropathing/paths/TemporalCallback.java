package com.pedropathing.paths;

import java.util.concurrent.TimeUnit;

/**
 * This is the TemporalCallback class. This class handles callbacks that occur a given amount of time after a Path in a PathChain has begun running.
 *
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
public class TemporalCallback implements PathCallback {
    /**
     * The time (in milliseconds) after which the callback should be triggered since initialization.
     */
    private final double startCondition;

    /**
     * The action to execute when the callback is triggered.
     */
    private final Runnable runnable;

    /**
     * The start time (in milliseconds) when the callback was initialized.
     */
    private double startTime = 0;

    /**
     * The index of the path this callback is associated with.
     */
    private final int index;

    /**
     * Constructs a TemporalCallback.
     *
     * @param index the index of the path this callback is associated with
     * @param startCondition the time (in milliseconds) after which the callback should be triggered
     * @param runnable the action to execute when the callback is triggered
     */
    public TemporalCallback(int index, double startCondition, Runnable runnable) {
        this.index = index;
        this.startCondition = startCondition;
        this.runnable = runnable;
    }

    /**
     * Executes the callback action.
     *
     * @return true after running the action
     */
    @Override
    public boolean run() {
        runnable.run();
        return true;
    }

    /**
     * Checks if the specified time has elapsed since initialization.
     *
     * @return true if the elapsed time is greater than or equal to the start condition
     */
    @Override
    public boolean isReady() {
        return TimeUnit.NANOSECONDS.convert(System.nanoTime(), TimeUnit.MILLISECONDS) - startTime >= startCondition;
    }

    /**
     * Initializes the callback by recording the current time.
     */
    @Override
    public void initialize() {
        startTime = TimeUnit.NANOSECONDS.convert(System.nanoTime(), TimeUnit.MILLISECONDS);
    }

    /**
     * Returns the index of the path this callback is associated with.
     *
     * @return the path index
     */
    @Override
    public int getPathIndex() {
        return index;
    }
}
