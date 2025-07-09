package com.pedropathing.log;

import java.util.HashMap;

public class PedroLogger implements LogSubscriber {
    private HashMap<String, Object> data = new HashMap<>();
    @Override
    public void onLog(String key, Object value) {
        data.put(key, value);
    }

    @Override
    public void update() {
        // Generate Logs
        for (String key : data.keySet()) {
            // Add to log
        } System.out.println("Log: " + data);
    }
}
