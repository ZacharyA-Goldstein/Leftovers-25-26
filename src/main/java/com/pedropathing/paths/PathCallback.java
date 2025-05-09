package com.pedropathing.paths;

/**
 * This is the PathCallback class. This class handles callbacks of Runnables in PathChains.
 * Basically, this allows you to run non-blocking code in the middle of PathChains.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Havish Sripada - 12808 RevAmped Robotics
 * @version 2.0, 5/6/2025
 */
public interface PathCallback {
    boolean run();
    boolean isReady();
    default void initialize() {}
    default void reset() {}
    default boolean isCompleted() {return false;}
    int getPathIndex();
}
