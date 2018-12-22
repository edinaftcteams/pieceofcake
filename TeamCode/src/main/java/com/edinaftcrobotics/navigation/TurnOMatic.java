package com.edinaftcrobotics.navigation;

import com.edinaftcrobotics.drivetrain.Mecanum;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Timer;
import java.util.TimerTask;

public class TurnOMatic {
    BNO055IMU imu = null;
    Mecanum mecanum = null;
    Telemetry telemetry = null;
    private double error = 0;
    private double startAngle = 0;
    private double endAngle = 0;
    private double derivative = 0;
    private double integral = 0;
    private double previousError = 0;
    private double currentAngle = 0;
    private double timerLength = 200;
    private double Kp = 0.2, Ki = 0.01, Kd = 1; // PID constant multipliers
    private double output = 0;
    private double previousOutput = 0;
    private long previousTime;
    private long difference = 0;
    private double firstValue = 0;
    private boolean firstRun = true;
    private Timer timer = null;
    private TimerTask timerTask = null;

    public TurnOMatic(BNO055IMU imu, Mecanum mecanum, Telemetry telemetry, double toWnatAngle) {
        this.imu = imu;
        this.mecanum = mecanum;
        this.telemetry = telemetry;
        startAngle = GetImuAngle();
        endAngle = toWnatAngle + startAngle;
        StartTimer();
    }

    public void Turn(double closeEnough, long ticksToWait) {
        double currentRatio = 1;

        while (Math.abs(currentRatio) > closeEnough) {
            currentRatio = (previousOutput - output) / firstValue;

            telemetry.addData("Angle", "%f", currentAngle);
            telemetry.addData("PreviousOutput: ", previousOutput);
            telemetry.addData("Output: ", output);
            telemetry.addData("CurrentRatio", currentRatio);
            telemetry.addData("Proportional", error);
            telemetry.addData("Integral", integral);
            telemetry.addData("Derivative", derivative);
            telemetry.addData("Time", difference);
            telemetry.addData("Output Difference: ", previousOutput - output);
            double answer = Range.clip(currentRatio, -1.0, 1.0) * .8;
            telemetry.addData("Left Power: ", "%f", answer);
            telemetry.addData("Right Power: ", "%f", -answer);

            //mecanum.Move(-answer , -answer);
            telemetry.update();
        }

        mecanum.Stop();

        StopTimer();
    }

    private double GetImuAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return angles.firstAngle;
    }

    private void SetupTimerTask() {
        timerTask = new TimerTask() {
            @Override
            public void run() {
                if (firstRun) {
                    if (Double.isNaN(integral)){
                        integral = 0;
                    }
                }

                currentAngle = GetImuAngle() - startAngle;
                long currentTime = System.currentTimeMillis();
                difference = currentTime - previousTime;

                error = endAngle - currentAngle;
                integral = integral + (error * difference);
                derivative = (error - previousError) / difference;

                previousOutput = output;
                output = (Kp * error) + (Ki * integral) + (Kd * derivative);

                if (firstRun) {
                    firstValue = Math.abs(previousOutput - output);
                    firstRun = false;
                }

                previousError = error;
                previousTime = currentTime;
            }
        };
    }

    private void StartTimer() {
        error = 0;
        derivative = 0;
        integral = 0;
        previousError = 0;
        currentAngle = 0;
        output = 0;
        previousOutput = 0;
        previousTime = System.currentTimeMillis();
        firstRun = true;
        firstValue = 0;

        timer = new Timer();
        timer.scheduleAtFixedRate(timerTask, 200, 200);
    }

    private void StopTimer() {

        timer.cancel();
    }
}
