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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private int BackFlip = 0;
    private int VerticalFlip = 880;
    private int FlatFlip = 1860;

    protected MineralLocation mineralLocation = MineralLocation.RIGHT;
    protected Camera camera = null;
    protected GoldMineralTracker mineralTracker = null;
    protected PieceOfCake robot = new PieceOfCake();
    protected Mecanum mecanum = null;
    protected int latchHeight = 100;
    protected BNO055IMUImpl imu = null;
    protected TFObjectDetector tfod;

    protected int slideRightPosition = DrivePerInch * 8;
    protected int slideLeftPosition = DrivePerInch * 8;
    protected int knockForwardPosition = DrivePerInch * 8;
    protected int driveForwardPosition = DrivePerInch * 12;
    protected boolean yPressed = false;
    protected boolean aPressed = false;
    protected boolean dPadLeftPressed = false;
    protected boolean dPadRightPressed = false;
    protected boolean dPadUpPressed = false;
    protected boolean dPadDownPressed = false;
    protected boolean bumperLeftPressed = false;
    protected boolean bumperRightPressed = false;

    protected void InitRobot() {
        robot.init(hardwareMap);

        mecanum = new Mecanum(robot.getFrontL(), robot.getFrontR(), robot.getBackL(), robot.getBackR(), false);
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

    protected void InitSetup() {
        while (!gamepad1.x) {
            if (gamepad1.y && !yPressed) {
                driveForwardPosition += (int) (DrivePerInch * .5);
                yPressed = true;
            } else if (!gamepad1.y) {
                yPressed = false;
            }

            if (gamepad1.a && !aPressed) {
                driveForwardPosition -= (int) (DrivePerInch * .5);
                aPressed = true;
            } else if (!gamepad1.a) {
                aPressed = false;
            }

            if (gamepad1.dpad_left && !dPadLeftPressed) {
                slideRightPosition -= (int) (DrivePerInch * .5);
                dPadLeftPressed = true;
            } else if (!gamepad1.dpad_left) {
                dPadLeftPressed = false;
            }

            if (gamepad1.dpad_right && !dPadRightPressed) {
                slideRightPosition += (int) (DrivePerInch * .5);
                dPadRightPressed = true;
            } else if (!gamepad1.dpad_right) {
                dPadRightPressed = false;
            }

            if (gamepad1.dpad_up && !dPadUpPressed) {
                knockForwardPosition += (int) (DrivePerInch * .5);
                dPadUpPressed = true;
            } else if (!gamepad1.dpad_up) {
                dPadUpPressed = false;
            }

            if (gamepad1.dpad_down && !dPadDownPressed) {
                knockForwardPosition -= (int) (DrivePerInch * .5);
                dPadDownPressed = true;
            } else if (!gamepad1.dpad_down) {
                dPadDownPressed = false;
            }

            if (gamepad1.left_bumper && !bumperLeftPressed) {
                slideLeftPosition -= (int) (DrivePerInch * .5);
                bumperLeftPressed = true;
            } else if (!gamepad1.left_bumper) {
                bumperLeftPressed = false;
            }

            if (gamepad1.right_bumper && !bumperRightPressed) {
                slideLeftPosition += (int) (DrivePerInch * .5);
                bumperRightPressed = true;
            } else if (!gamepad1.right_bumper) {
                bumperRightPressed = false;
            }

            telemetry.addData("DPad L/R controls slide right position, currently", "%d", slideRightPosition);
            telemetry.addData("DPad U/D controls knock forward position, currently", "%d", knockForwardPosition);
            telemetry.addData("Bumper L/R controls slide left position, currently", "%d", slideLeftPosition);
            telemetry.addData("Buttons Y/A controls drive forward position, currently", "%d", driveForwardPosition);
            telemetry.addData("Press X", "to lock in settings");
            telemetry.update();
        }
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

    public void SearchForTFMineral() {
        ActivateTFCamera();

        while (!isStarted()) {
            synchronized (this) {
                try {
                    this.wait();
                    LocateTFMineral();
                    if (mineralLocation == MineralLocation.LEFT) {
                        telemetry.addData("Mineral Location", "Left");
                    } else if (mineralLocation == MineralLocation.MIDDLE) {
                        telemetry.addData("Mineral Location", "Middle");
                    } else if (mineralLocation == MineralLocation.RIGHT) {
                        telemetry.addData("Mineral Location", "Right");
                    } else {
                        telemetry.addData("Mineral Location", "Lost");
                    }
                    telemetry.update();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }

        LocateTFMineral();
        if (mineralLocation == MineralLocation.LEFT) {
            telemetry.addData("Mineral Location", "Left");
        } else if (mineralLocation == MineralLocation.MIDDLE) {
            telemetry.addData("Mineral Location", "Middle");
        } else if (mineralLocation == MineralLocation.RIGHT) {
            telemetry.addData("Mineral Location", "Right");
        } else {
            telemetry.addData("Mineral Location", "Lost, Default to Right");
        }
        telemetry.update();

        DeactivateTFCamera();
    }

    private void LocateTFMineral() {
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
    }

    public AutonomousStates Latch () {
        robot.getLift().setPower(-.3);
        robot.getLockServo().setPower(1);

        return AutonomousStates.LATCHED;
    }

    public AutonomousStates Drop() {
        ElapsedTime watch = new ElapsedTime();
        robot.getSlide().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // do something to drop
        robot.getTopFlip().setPosition(1);
        robot.getFrontFlip().setTargetPosition(VerticalFlip);
        robot.getFrontFlip().setPower(.7);

        while (robot.getFrontFlip().isBusy()) {
            idle();
        }

        robot.getFrontFlip().setPower(0);

        robot.getLockServo().setPower(0);
        robot.getLift().setPower(0);

        watch.reset();
        while (watch.milliseconds() < 5000) {
            idle();
        }

        robot.getLift().setPower(.3);
        watch.reset();
        while (watch.milliseconds() < 1500) {
            idle();
        }

        robot.getLift().setPower(0);

        return AutonomousStates.DROPPED;
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
        mecanum.MoveForward(.2, forwardDistance, this);

        return AutonomousStates.MOVED_FORWARD;
    }

    public AutonomousStates DriveToMineral (int slideLeftDistance, int slideRightDistance) {
        if (mineralLocation == MineralLocation.LEFT) {
            mecanum.SlideLeft(.2, slideLeftDistance, this);
        } else if (mineralLocation == MineralLocation.RIGHT) {
            mecanum.SlideRight(.2, slideRightDistance, this);
        }

        return AutonomousStates.AT_MINERAL;
    }

    public AutonomousStates PushMineral (int pushDistance) {
        mecanum.MoveForward(.2, pushDistance, this);
        return AutonomousStates.MINERAL_PUSHED;
    }

    public AutonomousStates PushMineralAndDriveToDepot(int knockForwardPosition) {
        if (mineralLocation == MineralLocation.LEFT) {
            mecanum.TurnRight(.5 , 1000, this);
        } else if (mineralLocation == MineralLocation.RIGHT) {
            mecanum.TurnLeft(.5 , 1000, this);
        }

        mecanum.MoveForward(.2, knockForwardPosition, this);
        return AutonomousStates.AT_DEPOT;
    }

    public AutonomousStates ExtendArm() {
        ElapsedTime watch = new ElapsedTime();
        watch.reset();

        // slide arm out
        robot.getSlide().setPower(.5);
        // run for 1500 milliseconds
        while (watch.milliseconds() < 1500) {
            idle();
        }

        robot.getSlide().setPower(0);

        return AutonomousStates.ARM_EXTENDED;
    }

    public AutonomousStates FlipToBack() {
        robot.getFrontFlip().setPower(.5);
        robot.getFrontFlip().setTargetPosition(BackFlip);
        while (robot.getFrontFlip().isBusy()) {
            idle();;
        }

        robot.getFrontFlip().setPower(0);

        return AutonomousStates.FLIP_AT_BACK;
    }

    public AutonomousStates DropMarker () {
        ElapsedTime watch = new ElapsedTime();
        watch.reset();

        // slide arm out
        robot.getSlide().setPower(-.5);
        // run for 1500 milliseconds
        while (watch.milliseconds() < 1500) {
            idle();
        }

        robot.getSlide().setPower(0);

        // flip intake to flat
        robot.getFrontFlip().setPower(.5);
        robot.getFrontFlip().setTargetPosition(FlatFlip);
        while (robot.getFrontFlip().isBusy()) {
            idle();;
        }

        robot.getFrontFlip().setPower(0);
        // spin the intake to dump marker
        robot.getIntake().setPower(.5);
        watch.reset();
        while (watch.milliseconds() < 1500) {
            idle();
        }

        // flip intake back to 0
        robot.getIntake().setPower(0);
        robot.getFrontFlip().setPower(.5);
        robot.getFrontFlip().setTargetPosition(BackFlip);
        while (robot.getFrontFlip().isBusy()) {
            idle();;
        }

        robot.getFrontFlip().setPower(0);
        // move slide back in
        watch.reset();
        robot.getSlide().setPower(.5);
        // run for 1500 milliseconds
        while (watch.milliseconds() < 1700) {
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
