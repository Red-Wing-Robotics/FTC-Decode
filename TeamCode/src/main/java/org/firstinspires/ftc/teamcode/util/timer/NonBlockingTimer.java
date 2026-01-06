package org.firstinspires.ftc.teamcode.util.timer;

/**
 * A reusable, non-blocking timer for state machines and autonomous paths.
 */
public class NonBlockingTimer {

    private long durationMs;
    private long startTime = 0;
    private boolean isRunning = false;

    /**
     * Creates a timer with a specified duration.
     *
     * @param durationMs The duration in milliseconds.
     */
    public NonBlockingTimer(long durationMs) {
        this.durationMs = durationMs;
    }

    /**
     * Starts the timer. If the timer is already running, it will be reset and started again.
     */
    public void start() {
        this.startTime = System.currentTimeMillis();
        this.isRunning = true;
    }

    /**
     * Checks if the timer has finished.
     *
     * @return True if the timer is running and the specified duration has passed, false otherwise.
     */
    public boolean isFinished() {
        if (!isRunning) {
            return false;
        }
        return System.currentTimeMillis() - startTime >= durationMs;
    }

    /**
     * Stops and resets the timer, making it ready to be started again.
     */
    public void reset() {
        this.isRunning = false;
        this.startTime = 0;
    }

    /**
     * Gets the elapsed time since the timer started.
     *
     * @return The elapsed time in milliseconds, or 0 if the timer is not running.
     */
    public long getElapsedTime() {
        if (!isRunning) {
            return 0;
        }
        return System.currentTimeMillis() - startTime;
    }

    /**
     * Changes the duration of the timer. This does not affect a currently running timer
     * unless it is reset and started again.
     *
     * @param durationMs The new duration in milliseconds.
     */
    public void setDuration(long durationMs) {
        this.durationMs = durationMs;
    }
}
