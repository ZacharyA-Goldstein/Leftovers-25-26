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

    public TemporalCallback(int index, double startCondition, Runnable runnable) {
        this.index = index;
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
        return TimeUnit.NANOSECONDS.convert(System.nanoTime(), TimeUnit.MILLISECONDS) - startTime >= startCondition;
    }

    @Override
    public void initialize() {
        startTime = TimeUnit.NANOSECONDS.convert(System.nanoTime(), TimeUnit.MILLISECONDS);
    }

    @Override
    public int getPathIndex() {
        return index;
    }
}
