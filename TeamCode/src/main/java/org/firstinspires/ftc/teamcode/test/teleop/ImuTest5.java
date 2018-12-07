package org.firstinspires.ftc.teamcode.test.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Test: IMU4", group = "Teleop Test")
//@Disabled
public class ImuTest5 extends LinearOpMode {
    BNO055IMU imu = null;

    public void runOpMode() {
        double integral = 0;
        double preError = 0;
        double Kp = 1.5;
        double Ki = .5;
        double Kd = .3;
        double Dt = .02;
        double previousOutput = 0;
        double output = 0;
        double acceptedError = 0.5;

        //SetupIMU();

        waitForStart();

        while (opModeIsActive()) {
            double error = 0;

            //error = GetImpuAngle();

            error = 90 - GetJoyStickAngle();
            // track error over time, scaled to the timer interval
            integral = integral + (error * Dt);
            // determine the amount of change from the last time checked
            double derivative = (error - preError) / Dt;
            // calculate how much to drive the output in order to get to the
            // desired setpoint.
            previousOutput = output;
            output = (Kp * error) + (Ki * integral) + (Kd * derivative);
            // remember the error for the next time around.
            preError = error;

            double difference = previousOutput - output;
            telemetry.addData("Output", "%f", output);
            telemetry.addData("Difference", "%f", previousOutput - output);
            if (Math.abs(difference) < acceptedError) {
                telemetry.addData("Turn", "Stop");
            } else if (difference > 0) {
                telemetry.addData("Turn", "Left");
            } else if (difference < 0) {
                telemetry.addData("Turn", "Right");
            }
            telemetry.update();

            sleep(200);
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
}
