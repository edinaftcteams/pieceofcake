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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.enums.AutonomousStates;
import org.firstinspires.ftc.teamcode.enums.MineralLocation;
import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

import java.util.List;

abstract class BaseAutoOpMode extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    protected int DrivePerInch = (int)(1120 / 12.56);
    private int ArmDistancePerSecond = 19;
    private int DropArmPerDegree = (int)(8640 / 360);
    private int BackFlip = 0;
    private int VerticalFlip = (int)(DropArmPerDegree * 22);
    private int FlatFlip = (int)(DropArmPerDegree * 102.5);
    private int BottomFlip = (int)(DropArmPerDegree * 143.5);

    protected MineralLocation mineralLocation = MineralLocation.RIGHT;
    protected Camera camera = null;
    protected GoldMineralTracker mineralTracker = null;
    protected PieceOfCake robot = new PieceOfCake();
    protected Mecanum mecanum = null;
    protected int latchHeight = (int)(4.5 * 288 / 1.75);
    protected BNO055IMUImpl imu = null;
    protected TFObjectDetector tfod;

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

    protected void ActivateTFCamera() {
        camera = new BackPhoneCamera();

        camera.activate();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, camera.getPOCVuforia());
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }


        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }
    }

    protected void DeactivateCamera() {
        camera.deactivate();
        camera = null;
    }

    protected void DeactivateTFCamera() {
        if (tfod != null) {
            tfod.shutdown();
        }

        camera.deactivate();;
        camera = null;
    }

    public AutonomousStates Latch (double powerLevel) {
        //robot.getLift().setPower(powerLevel);
        robot.getLockServo().setPower(1);

        return AutonomousStates.LATCHED;
    }

    public AutonomousStates Drop() {
        // do something to drop
        robot.getFrontFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getFrontFlip().setTargetPosition(VerticalFlip);
        robot.getFrontFlip().setPower(.2);
        robot.getLift().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getLift().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.getFrontFlip().isBusy()) {
            idle();
        }

        robot.getFrontFlip().setPower(0);

        robot.getLift().setTargetPosition(latchHeight);
        robot.getLockServo().setPower(0);
        robot.getLift().setPower(-.5);

        int error = Math.abs((int)(latchHeight * 0.95));
        int currentPosition =  Math.abs(robot.getLift().getCurrentPosition());

        while (robot.getLift().isBusy() && currentPosition < error) {
            currentPosition =  Math.abs(robot.getLift().getCurrentPosition());
            idle();
        }

        robot.getLift().setPower(0);

        return AutonomousStates.DROPPED;
    }

    public AutonomousStates LocateTFMineral() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                int goldMineralX = 300;
                if (updatedRecognitions.size() > 0) {
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        }

                        telemetry.addData("Gold Mineral Position", "%d", goldMineralX);
                        telemetry.update();
                    }
                }

                if (goldMineralX < 100) {
                    mineralLocation = MineralLocation.LEFT;
                } else if ((goldMineralX >= 100) && (goldMineralX < 150)) {
                    mineralLocation = MineralLocation.MIDDLE;
                } else {
                    mineralLocation = MineralLocation.RIGHT;
                }
            }
        }

        return AutonomousStates.MINERAL_LOCATED;
    }

    public AutonomousStates LocateMineral() throws InterruptedException {
        if (mineralTracker.getGoldMineralLocation()) {
            if (mineralTracker.getYPosition() > 150) {
                mineralLocation = MineralLocation.LEFT;
            } else if (mineralTracker.getYPosition() < 150) {
                mineralLocation = MineralLocation.MIDDLE;
            } else {
                mineralLocation = MineralLocation.RIGHT;
            }
        } else {
            mineralLocation = MineralLocation.RIGHT;
        }

        telemetry.update();

        return AutonomousStates.MINERAL_LOCATED;
    }

    public AutonomousStates MoveForward(int forwardDistance) {
        mecanum.MoveForward(.5, forwardDistance, this);

        return AutonomousStates.MOVED_FORWARD;
    }

    public AutonomousStates DriveToMineral (int slideLeftDistance, int slideRightDistance) {
        if (mineralLocation == MineralLocation.LEFT) {
            mecanum.SlideLeft(.5, slideLeftDistance, this);
        } else if (mineralLocation == MineralLocation.RIGHT) {
            mecanum.SlideRight(.5, slideRightDistance, this);
        }

        return AutonomousStates.AT_MINERAL;
    }

    public AutonomousStates PushMineral (int pushDistance) {
        mecanum.MoveForward(.5, pushDistance, this);
        return AutonomousStates.MINERAL_PUSHED;
    }

    public AutonomousStates MoveToMiddle(int slideLeftDistance, int slideRightDistanc) {
        if (mineralLocation == MineralLocation.LEFT) {
            mecanum.SlideRight(.5, slideLeftDistance, this);
            mecanum.TurnLeft(.5 , 100, this);
        } else if (mineralLocation == MineralLocation.RIGHT) {
            mecanum.SlideLeft(.5, slideRightDistanc, this);
            mecanum.TurnRight(.5 , 100, this);
        }

        return AutonomousStates.AT_MIDDLE;
    }

    public AutonomousStates DriveToDepot () {
        return AutonomousStates.AT_DEPOT;
    }

    public AutonomousStates DropMarker () {
        ElapsedTime watch = new ElapsedTime();
        watch.reset();

        robot.getSlide().setPower(.5);
        // run for 1500 milliseconds
        while (watch.milliseconds() < 1500) {
            idle();
        }

        robot.getSlide().setPower(0);

        robot.getFrontFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getFrontFlip().setPower(.5);
        robot.getFrontFlip().setTargetPosition(FlatFlip);
        while (robot.getFrontFlip().isBusy()) {
            idle();;
        }

        robot.getIntake().setPower(.5);
        robot.getIntake().setTargetPosition(1000);
        while (robot.getIntake().isBusy()) {
            idle();
        }

        robot.getFrontFlip().setPower(.5);
        robot.getFrontFlip().setTargetPosition(BackFlip);
        while (robot.getFrontFlip().isBusy()) {
            idle();;
        }

        watch.reset();
        robot.getSlide().setPower(-.5);
        // run for 1500 milliseconds
        while (watch.milliseconds() < 1500) {
            idle();
        }

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
