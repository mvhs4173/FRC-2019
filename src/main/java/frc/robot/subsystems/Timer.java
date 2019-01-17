package org.usfirst.frc.team4173.robot.subsystems;

/**
 * Created by ROBOT18 on 11/7/2017.
 * Use for when you need to keep track of time
 */

public class Timer {
    private double startTime;
    private double seconds;
    private boolean initialized;

    Timer(){
        initialized = false;
    }

    /**
     * @param seconds how long the timer should run
     */
    public void init(double seconds){
        this.seconds = seconds;
        startTime = (double) System.nanoTime() / 1.0E9;
        initialized = true;
    }

    public boolean isInitialized() {
        return initialized;
    }

    /**
     * @return true when timer is done
     */
    public boolean isTimerUp(){
        double currentTime = System.nanoTime() / 1.0E9;
        if(!initialized){
            return false;
        } else{
            return (currentTime - startTime >= seconds);
        }
    }

    public void disable(){
        initialized = false;
    }
}