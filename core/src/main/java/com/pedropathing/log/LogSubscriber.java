package com.pedropathing.log;

/**
 * A functional interface for subscribers to log events.
 * Implementations of this interface can be used to send log messages from a key and value.
 *
 * @author BeepBot99
 */
@FunctionalInterface
public interface LogSubscriber {
    /**
     * Called when a log event occurs.
     *
     * @param key   The key associated with the log event.
     * @param value The value associated with the log event.
     */
    void onLog(String key, Object value);

    /**
     * Update / Push the log subscriber.
     */
    default void update() { }
}
