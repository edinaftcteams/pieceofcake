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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
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
    protected int DrivePerInch = (int)(1120 / 18.85);
    private int ArmDistancePerSecond = 19;
    private int BackFlip = 0;
    private int VerticalFlip = 880;
    private int FlatFlip = 1860;
    private static final String VUFORIA_KEY = "ASA9XvT/////AAABmUnq30r9sU3Nmf/+RS+Xx0CHgJj/JtD5ycahnuM/0B2SFvbMRPIZCbLi4LeOkfse9Dymor5W7vNMYI+vmqVx9kpEaKE8VM7cFMUb/T1LLwlCPdX9QKOruzTcRdlYswR7ULh4K11GuFZDO/45pSks+Nf25kT5cnV+IN3TsscA0o7I6XPIeUoAJJPsjw+AycsmRk2uffr3Bnupexr93iRfHylniqP+ss4cRcT1lOqS5Zhh7FQaoelR58qL/RUorGpknjy9ufCn9ervc6Mz01u3ZkM/EOa5wUPT8bDzPZ6nMDaadqumorT5Py+GtJSUosUgz4Gd3iR++fdEk6faFZq3L9xfBSagNykwhiyYx+oqwVqe";

    protected MineralLocation mineralLocation = MineralLocation.RIGHT;
    private VuforiaLocalizer vuforia;
    protected GoldMineralTracker mineralTracker = null;
    protected PieceOfCake robot = new PieceOfCake();
    protected Mecanum mecanum = null;
    protected int latchHeight = 100;
    protected BNO055IMUImpl imu = null;
    protected TFObjectDetector tfod;

    protected int slideRightPosition = DrivePerInch * 23;
    protected int slideLeftPosition = DrivePerInch * 23;
    protected int knockForwardPosition = DrivePerInch * 32;
    protected int driveForwardPosition = (int)(DrivePerInch * 20.5);
    protected int anglePushDistance = DrivePerInch * 8;
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

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }
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

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    public void LocateTFMineral() {
        for (int x = 0; x< 1; x++) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    int goldMineralX = 0;
                    if (updatedRecognitions.size() > 0) {
                        for (Recognition recognition : updatedRecognitions) {
                            if ((recognition.getLabel().equals(LABEL_GOLD_MINERAL)) && recognition.getTop() < 500){
                                goldMineralX = (int) recognition.getLeft();
                            }

                            telemetry.addData("Gold Mineral Position", "%d", goldMineralX);
                        }
                    }

                    if (goldMineralX > 300) {
                        mineralLocation = MineralLocation.LEFT;
                        telemetry.addData("Mineral Location", "Left");
                    } else if ((goldMineralX >= 50) && (goldMineralX <= 300)) {
                        mineralLocation = MineralLocation.MIDDLE;
                        telemetry.addData("Mineral Location", "Middle");
                    } else {
                        mineralLocation = MineralLocation.RIGHT;
                        telemetry.addData("Mineral Location", "Right");
                    }
                } else {
                    telemetry.addData("Nothing", "Detected");
                    mineralLocation = MineralLocation.RIGHT;
                }
            } else {
                telemetry.addData("No", "TFOD");
            }

            telemetry.update();
            sleep(100);
        }
    }

    public void ShutdownTFOD() {
        if (tfod != null) {
            tfod.shutdown();
        }

    }
    public AutonomousStates Latch () {
        //robot.getLift().setPower(-.2);
        //robot.getLockServo().setPower(1);

        return AutonomousStates.LATCHED;
    }

    public AutonomousStates Drop() {
        /*
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

        //robot.getLockServo().setPower(-1);
        robot.getLift().setPower(0);

        watch.reset();
        while (watch.milliseconds() < 2500) {
            idle();
        }

        robot.getLift().setPower(.3);
        watch.reset();
        while (watch.milliseconds() < 1500) {
            idle();
        }

        robot.getLift().setPower(0);
        //robot.getLockServo().setPower(0);

        mecanum.MoveBackwards(.3, 50, this);
*/
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
        mecanum.MoveForward(.6, forwardDistance, this);

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

    public AutonomousStates BackAwayFromMIneral(int backDistance) {
        mecanum.MoveBackwards(.5, backDistance, this);

        return AutonomousStates.BACKED_AWAY_FROM_MINERAL;
    }

    public AutonomousStates PushMineralAndDriveToDepot(int knockForwardPosition) {
        mecanum.MoveForward(.5, knockForwardPosition / 2, this);

        if (mineralLocation == MineralLocation.LEFT) {
            mecanum.TurnRight(.5 , 1025, this);
            mecanum.MoveForward(.5, anglePushDistance, this);
        } else if (mineralLocation == MineralLocation.RIGHT) {
            mecanum.TurnLeft(.5 , 1025, this);
            mecanum.MoveForward(.5, anglePushDistance, this);
        }

        mecanum.MoveForward(.5, knockForwardPosition / 2, this);

        return AutonomousStates.AT_DEPOT;
    }

    public AutonomousStates ExtendArm() {
        ElapsedTime watch = new ElapsedTime();
        watch.reset();

        // slide arm out
        robot.getSlide().setPower(-.5);
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
        robot.getSlide().setPower(-1);
        // run for 1500 milliseconds
        while (watch.milliseconds() < 1000) {
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
        robot.getSlide().setPower(1);
        // run for 1500 milliseconds
        while (watch.milliseconds() < 1200) {
            idle();
        }

        robot.getSlide().setPower(0);

        return AutonomousStates.DROPPED_MARKER;
    }

    public AutonomousStates MoveToLeftWall(int distanceFromLeftMineral, int distanceFromCenterMineral, int distanceFromRightMineral) {
        if (mineralLocation == MineralLocation.RIGHT) {
            mecanum.SlideLeft(0.5, distanceFromRightMineral, this);
        } else if (mineralLocation == MineralLocation.LEFT) {
            mecanum.SlideLeft(0.5, distanceFromLeftMineral, this);
        } else if (mineralLocation == MineralLocation.MIDDLE) {
            mecanum.SlideLeft(0.5, distanceFromCenterMineral, this);
        }

        return AutonomousStates.AT_LEFT_WALL;
    }

    public AutonomousStates TurnLeftTowardsCrater() {
        mecanum.TurnLeft(.5 , 3050, this);

        return AutonomousStates.TURNED_TOWARDS_CRATER;
    }

    public AutonomousStates MoveTowardsCrater() {
        mecanum.MoveForward(.3, DrivePerInch * 12, this);

        return AutonomousStates.AT_CRATER;
    }

    public AutonomousStates TurnLeftTowardsDepot(){
        mecanum.TurnLeft(.5 , 3050, this);

        return AutonomousStates.TURNED_TOWARDS_DEPOT;
    }

    public AutonomousStates MoveTowardsDepot(){
        mecanum.MoveForward(.5,DrivePerInch * 24, this);

        return AutonomousStates.AT_DEPOT;
    }

    public AutonomousStates TurnTowardsCrater(){
        mecanum.TurnLeft(.5 , 4050, this);

        return AutonomousStates.FACING_CRATER;
    }

    public AutonomousStates DriveTowardsCrater(){
        mecanum.MoveForward(.5,DrivePerInch * 75, this);

        return AutonomousStates.AT_CRATER;
    }
}
