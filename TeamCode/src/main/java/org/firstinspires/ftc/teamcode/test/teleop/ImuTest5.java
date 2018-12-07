package org.firstinspires.ftc.teamcode.test.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.Timer;
import java.util.TimerTask;

@TeleOp(name = "Test: IMU4", group = "Teleop Test")
//@Disabled
public class ImuTest5 extends LinearOpMode {
    BNO055IMU imu = null;
    private double error = 0;
    private double endAngle = 135;
    private double derivative = 0;
    private double integral = 0;
    private double previousError = 0;
    private double currentAngle = 0;
    private double timerLength = 200;
    private double Kp = 0.2, Ki = 0.01, Kd = 1; // PID constant multipliers
    private double output = 0;
    private double previousOutput = 0;
    private long previousTime = 0;
    private long difference = 0;
    private double firstValue = 0;
    private boolean firstRun = true;
    private Timer timer = new Timer();

    public void runOpMode() {

        SetupIMU();

        SetupTimer();

        waitForStart();

        while (opModeIsActive()) {
            currentAngle = GetImpuAngle();
            double currentRatio = (previousOutput - output) / firstValue *1000000000 * 10;

            telemetry.addData("Angle: ", currentAngle);
            telemetry.addData("Output: ", output);
            telemetry.addData("Output Difference: ", previousOutput - output);
            telemetry.addData("Left Power: ",  currentRatio);
            telemetry.addData("Right Power: ", -currentRatio);

            telemetry.update();
        }
    }

    private void SetupIMU() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
    }

    private double GetImpuAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("IMU Angles:", angles);

        return angles.firstAngle;
    }

    private double GetJoyStickAngle() {
        double angles = Math.toDegrees(Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y));

        telemetry.addData("Joystick Angles:", angles);

        return angles;
    }

    private void SetupTimer() {
        timer.scheduleAtFixedRate(new TimerTask() {


            public void run() {
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
        }, 200, 200);
    }
}
