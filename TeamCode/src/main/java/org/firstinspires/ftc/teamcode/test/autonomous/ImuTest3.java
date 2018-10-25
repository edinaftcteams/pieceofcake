package org.firstinspires.ftc.teamcode.test.autonomous;

import com.qualcomm.robotcore.util.Range;

public class ImuTest3 {
    private double KP;

    /**
     * The gain for the integral part of the controller.
     */
    private double KI;

    /**
     * The gain for the derivative part of the controller.
     */
    private double KD;

    /** The output from the proportional part of the controller. */
    private double error = 0;

    /**
     * The position the robot is trying to get to.
     */
    private double target = 0;

    /**
     * The ouput from the integral part of the controller.
     */
    private double integral = 0;

    /**
     * The output from the derivative part of the controller.
     */
    private double derivative = 0;

    /**
     * The time, in nanoseconds, of the last time of controller ran through the loop. Used when caluclating integral and derivative.
     */
    private long timeAtUpdate;

    private boolean integralSet = false;
    private boolean derivativeSet = false;
    private double derivativeAveraging = 0.95;
    private boolean processManualDerivative = false;

    private double maxErrorForIntegral = Double.POSITIVE_INFINITY;
    private double maxDerivative = Double.POSITIVE_INFINITY;

    private double nanoToUnit(long nano) {  //Used to convert nanoseconds to seconds
        return nano/1E9;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    /**
     * Gets the target value the system is currently trying to reach or maintain.
     * @return The current target value
     */
    public double getTarget() {
        return target;
    }

    public double output() {
        return KP*error+KI*integral+KD*derivative;
    }

    /**
     * Does all of the fancy math, requiring only an input into the proportional part of the controller.
     * @param input The proportional input into the fancy math, such as the heading from a gyro
     */
    public void input(double input) {
        long newTime = System.nanoTime();
        error = target-input;
        if (!integralSet) integral += Range.clip(error, -maxErrorForIntegral, maxErrorForIntegral)*nanoToUnit(newTime-timeAtUpdate);
        if (!derivativeSet) derivative = derivative*derivativeAveraging+(error/nanoToUnit(newTime-timeAtUpdate))*(1-derivativeAveraging);
        timeAtUpdate = newTime;
        integralSet = false;
        derivativeSet = false;
    }
}
