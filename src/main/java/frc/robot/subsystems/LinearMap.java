package org.usfirst.frc.team4173.robot.subsystems;

/**
 * Created by ROBOT12 on 1/12/2017.
 */


public class LinearMap {
    private double slope;
    private double intercept;
    public LinearMap(double from1, double to1, double from2, double to2){
        slope = (to2 - to1) / (from2 - from1);
        intercept = to1 - from1 * slope;
    }
    public double forward(double from){
        return from * slope + intercept;
    }
    public double backward(double to){
        return (to - intercept) / slope;
    }
}
