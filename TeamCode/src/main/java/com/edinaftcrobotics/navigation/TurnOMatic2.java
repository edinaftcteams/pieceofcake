package com.edinaftcrobotics.navigation;

import com.edinaftcrobotics.drivetrain.Mecanum;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Timer;
import java.util.TimerTask;

public class TurnOMatic2 {
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
    private double Kp = 0.2, Ki = 0.01, Kd =  1; // PID constant multipliers
    private double output = 0;
    private double previousTime;
    private double firstValue = 0;
    private boolean firstRun = true;
    private LinearOpMode opMode = null;

    public TurnOMatic2(BNO055IMU imu, Mecanum mecanum, Telemetry telemetry, double toWnatAngle, LinearOpMode opMode) {
        this.imu = imu;
        this.mecanum = mecanum;
        this.telemetry = telemetry;
        startAngle = currentAngle = GetImuAngle();
        endAngle = toWnatAngle;
        this.opMode = opMode;
        this.mecanum.StopAndResetMotors2();
    }

    public void Turn(double closeEnough, long ticksToWait) {
        previousTime = System.currentTimeMillis();

        opMode.sleep(100);

        while (opMode.opModeIsActive()){

            ComputeOuptut();
            telemetry.addData("S, C, E Angle", "%f, %f, %f", startAngle, currentAngle, endAngle);
            telemetry.addData("Proportional", "%f %f",  error, Kp * error);
            telemetry.addData("Integral", "%f %f", integral, Ki * integral);
            telemetry.addData("Derivative", "%f %f", derivative, Kd * derivative);
            double left = output;
            double right = -output;
            telemetry.addData("Left Power: ", "%f", output);
            telemetry.addData("Right Power: ", "%f", -output);

            opMode.sleep(200);
            //mecanum.Move(left , right);
            telemetry.update();
        }

        mecanum.Stop();
    }

    private double GetImuAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return angles.firstAngle;
    }

    private void ComputeOuptut() {
        double currentTime = System.currentTimeMillis();
        double difference = (currentTime - previousTime);
        // basic pid code for getting the angle and computing the output
        // for motor speed
        currentAngle = GetImuAngle();

        error = endAngle - currentAngle;
        integral = integral + (error * difference);
        if (integral > 1.0) {
            integral = 1.0;
        }

        if (integral < -1.0) {
            integral = -1.0;
        }

        derivative = (error - previousError) / difference;

        output = (Kp * error) + (Ki * integral) + (Kd * derivative);

        previousError = error;
        previousTime = currentTime;

        telemetry.addData("Difference", difference);
    }
}
