package com.pedropathing.log;

import java.util.ArrayList;
import java.util.List;

/**
 * Logger class for logging key-value pairs and notifying subscribers.
 * Subscribers can implement the LogSubscriber interface to receive log updates.'
 *
 * @author BeepBot99
 * @author Baron Henderson - 20077 The Indubitables
 */
public final class Logger {
    private static final List<LogSubscriber> subscribers = new ArrayList<>();

    public static void addSubscriber(LogSubscriber subscriber) {
        subscribers.add(subscriber);
    }

    public static void log(String key, Object value) {
        for (LogSubscriber subscriber : subscribers) {
            subscriber.onLog(key, value);
        }
    }

    public static void update() {
        for (LogSubscriber subscriber : subscribers) {
            subscriber.update();
        }
    }

    public static void clearSubscribers() {
        subscribers.clear();
    }
}