package org.firstinspires.ftc.teamcode.autonomous;

import com.edinaftcrobotics.drivetrain.Mecanum;
import com.edinaftcrobotics.vision.camera.BackPhoneCamera;
import com.edinaftcrobotics.vision.camera.Camera;
import com.edinaftcrobotics.vision.tracker.roverruckus.GoldMineralTracker;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.enums.AutonomousStates;
import org.firstinspires.ftc.teamcode.enums.MineralLocation;
import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

abstract class BaseAutoOpMode extends LinearOpMode {
    protected MineralLocation mineralLocation = MineralLocation.RIGHT;
    protected Camera camera = null;
    protected GoldMineralTracker mineralTracker = null;
    protected PieceOfCake robot = new PieceOfCake();
    protected Mecanum mecanum = null;
    protected int latchHeight = 0;
    protected BNO055IMUImpl imu = null;

    protected void InitRobot() {
        robot.init(hardwareMap);

        mecanum = new Mecanum(robot.getFrontL(), robot.getFrontR(), robot.getBackL(), robot.getBackR());
    }

    protected void InitGyro() {
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //Add calibration file?
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();  //Figure out why the naive one doesn't have a public constructor
        parameters.loggingEnabled = true;   //For debugging
        parameters.loggingTag = "IMU";      //For debugging

        imu.initialize(parameters);
        while (!imu.isGyroCalibrated());
    }

    protected void ActivateCamera() throws InterruptedException {
        camera = new BackPhoneCamera();

        camera.activate();
        mineralTracker = new GoldMineralTracker(camera);
    }

    protected void DeactivateCamera() {
        camera.deactivate();
        camera = null;
    }

    public AutonomousStates Latch (double powerLevel) {
        robot.getLift().setPower(powerLevel);
        latchHeight = robot.getLift().getCurrentPosition();

        return AutonomousStates.LATCHED;
    }

    public AutonomousStates Drop() {
        // do something to drop
        robot.getLift().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getLift().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getLift().setPower(-.5);
        robot.getLift().setTargetPosition(latchHeight);

        int error = Math.abs((int)(latchHeight * 0.95));
        int currentPosition =  Math.abs(robot.getLift().getCurrentPosition());

        while (robot.getLift().isBusy() && currentPosition < error) {
            currentPosition =  Math.abs(robot.getLift().getCurrentPosition());
            idle();
        }

        robot.getLift().setPower(0);

        return AutonomousStates.DROPPED;
    }

    public AutonomousStates LocateMineral() throws InterruptedException {
        if (mineralTracker.getGoldMineralLocation()) {
            if (mineralTracker.getYPosition() < 100) {
                mineralLocation = MineralLocation.LEFT;
            } else if ((mineralTracker.getYPosition() > 100) && (mineralTracker.getYPosition() < 500)) {
                mineralLocation = MineralLocation.MIDDLE;
            } else if (mineralTracker.getYPosition() > 500) {
                mineralLocation = MineralLocation.RIGHT;
            }
        } else {
            mineralLocation = MineralLocation.RIGHT;
        }

        telemetry.update();

        return AutonomousStates.MINERAL_LOCATED;
    }

    public AutonomousStates DriveToMineral (int forwardDistance, int slideLeftDistance, int slideRightDistance) {
        mecanum.MoveForward(.5, forwardDistance, this);

        if (mineralLocation == MineralLocation.LEFT) {
            mecanum.SlideLeft(.5, slideLeftDistance, this);
        } else if (mineralLocation == MineralLocation.RIGHT) {
            mecanum.SlideRight(.5, slideRightDistance, this);
        }

        return AutonomousStates.AT_MINERAL;
    }

    public AutonomousStates PushMineral (int pushDistance) {
        mecanum.MoveForward(.5, pushDistance, this);
        mecanum.MoveBackwards(-.5, pushDistance, this);
        return AutonomousStates.MINERAL_PUSHED;
    }

    public AutonomousStates MoveToMiddle(int slideLeftDistance, int slideRightDistanc) {
        if (mineralLocation == MineralLocation.LEFT) {
            mecanum.SlideRight(.5, slideLeftDistance, this);
        } else if (mineralLocation == MineralLocation.RIGHT) {
            mecanum.SlideLeft(.5, slideRightDistanc, this);
        }

        return AutonomousStates.AT_MIDDLE;
    }

    public AutonomousStates DriveToDepot () {
        return AutonomousStates.AT_DEPOT;
    }

    public AutonomousStates DropMarker () {
        robot.getSlide().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getSlide().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getSlide().setPower(.5);
        robot.getSlide().setTargetPosition(1000);

        robot.getFrontFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getFrontFlip().setPower(.5);
        robot.getFrontFlip().setTargetPosition(1000);

        robot.getIntake().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getIntake().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getIntake().setPower(.5);
        robot.getIntake().setTargetPosition(1000);

        return AutonomousStates.DROPPED_MARKER;
    }

    public AutonomousStates DriveToWall () {
        return AutonomousStates.AT_WALL;
    }

    public AutonomousStates StraightenOnWall () {
        return AutonomousStates.STRAIGHTENED_ON_WALL;
    }
    public AutonomousStates DriveToCrater () {
        return AutonomousStates.AT_CRATER;
    }
    public AutonomousStates ParkInCrater () {
        return AutonomousStates.PARKED_IN_CRATER;
    }
}
