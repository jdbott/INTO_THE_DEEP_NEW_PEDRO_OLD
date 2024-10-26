package org.firstinspires.ftc.teamcode;

import java.util.function.BooleanSupplier;

public class WaitHelper {
    private long waitStartTime = 0;
    private boolean waiting = false;

    // Non-blocking wait for a specific duration
    public boolean waitFor(long duration) {
        if (!waiting) {
            waiting = true;
            waitStartTime = System.currentTimeMillis();
        }

        if (System.currentTimeMillis() - waitStartTime >= duration) {
            waiting = false; // Reset waiting state
            return true; // Wait completed
        }
        return false; // Still waiting
    }

    // Non-blocking wait for a condition to be met
    public boolean waitForCondition(BooleanSupplier condition) {
        if (!waiting) {
            waiting = true;
        }

        if (condition.getAsBoolean()) {
            waiting = false; // Reset waiting state
            return true; // Condition met
        }
        return false; // Still waiting for condition
    }
}