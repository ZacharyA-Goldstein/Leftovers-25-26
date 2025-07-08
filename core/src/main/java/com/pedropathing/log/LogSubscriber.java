package com.pedropathing.log;

/**
 * A functional interface for subscribers to log events.
 * Implementations of this interface can be used to send log messages from a key and value.
 *
 * @author BeepBot99
 */
@FunctionalInterface
public interface LogSubscriber {
    void onLog(String key, Object value);
    default void update() { }
}
