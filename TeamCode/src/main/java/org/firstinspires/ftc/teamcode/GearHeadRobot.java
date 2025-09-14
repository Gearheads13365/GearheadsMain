package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;


public class GearHeadRobot {


    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    private ElapsedTime runtime = new ElapsedTime();

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor LB = null;
    private DcMotor RB = null;
    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LA = null;
    private DcMotor RA = null;
    private DcMotor LM = null;

    //Declare Servos
    private Servo claw = null;
    private CRServo w1 = null;
    private CRServo w2 = null;

    //IMU Variables
    IMU imu;

    //claw variables
    public double open = 0.08;
    public double close = 0.18;

    ////////////////////
    //Webcam Variables//
    ////////////////////

    public int lastPressed = 0;
    public double AvgVal_RED = 0;
    public double AvgVal_BLUE = 0;

    public double iconAnalysisMiddle = 0;
    public double iconAnalysisRight = 0;


    OpenCvCamera webcam;

    AprilTagDetectionPipeline aprilTagDetectionPipeline;



    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int OurTag = 12;
    double x_position = 0;

    AprilTagDetection tagOfInterest = null;

    ///////////////////////////
    //End of Webcam Variables//
    ///////////////////////////



    // Define a constructor that allows the OpMode to pass a reference to itself.
    public GearHeadRobot(LinearOpMode opmode) {myOpMode = opmode;}

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */

    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LB = myOpMode.hardwareMap.get(DcMotor.class, "LB");
        RB = myOpMode.hardwareMap.get(DcMotor.class, "RB");
        LF = myOpMode.hardwareMap.get(DcMotor.class, "LF");
        RF = myOpMode.hardwareMap.get(DcMotor.class, "RF");
        LA = myOpMode.hardwareMap.get(DcMotor.class, "LA");
        RA = myOpMode.hardwareMap.get(DcMotor.class, "RA");
        LM = myOpMode.hardwareMap.get(DcMotor.class, "LM");


        // Servo Hardware Map
        claw = myOpMode.hardwareMap.get(Servo.class,"claw");
        w1 = myOpMode.hardwareMap.get(CRServo.class,"w1");
        w2 = myOpMode.hardwareMap.get(CRServo.class,"w2");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);
        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LA.setDirection(DcMotor.Direction.REVERSE);
        RA.setDirection(DcMotor.Direction.FORWARD);
        LM.setDirection(DcMotor.Direction.REVERSE);

        w1.setDirection(DcMotorSimple.Direction.REVERSE);
        w2.setDirection(DcMotorSimple.Direction.REVERSE);


        //Set the motors to run using encoders
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set Motor Stop Method (Dont know if this is needed??? )
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ////////////////////////
        //Webcam Initilization//
        ////////////////////////

        // Hardware Map the camera
        int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        ///////////////////////////////
        //End of Webcam Initilization//
        ///////////////////////////////



        //////////////////////
        //IMU Initialization//
        //////////////////////

        IMU.Parameters myIMUparameters  = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        // Initialize IMU using Parameters
        imu = myOpMode.hardwareMap.get(IMU.class,"imu");
        imu.initialize(myIMUparameters);


        /////////////////////////////
        //End of IMU Initialization//
        /////////////////////////////






        // Set up our telemetry dashboard
        //composeTelemetry();
        runtime.reset();

    }

    ////////////////////////////////////////////////////////////////////////
    //Methods - re-usable functions that can be called many times in main///
    ////////////////////////////////////////////////////////////////////////



// This is where you will find functions like driving, moving lifts,getting values etc. to use in Teleop and Auto

    //Returns the average of the 4 drive motors (for reading positioning in Teleop)
    public int GetMotorEncoders(){
        return
                ( Math.abs(LB.getCurrentPosition())
                + Math.abs(RB.getCurrentPosition())
                + Math.abs(LF.getCurrentPosition())
                + Math.abs(RF.getCurrentPosition()) )/4;
    }

    public int Get_LB_Encoder()
    {
        return LB.getCurrentPosition();
    }
    public int Get_LF_Encoder()
    {
        return LF.getCurrentPosition();
    }
    public int Get_RB_Encoder()
    {
        return RB.getCurrentPosition();
    }
    public int Get_RF_Encoder()
    {
        return RF.getCurrentPosition();
    }

// For encoder values on arms, lifts, other accessories (If you need a number for the position on any servo, use these methods)


    public double GetArmPower()
    {
        return (LA.getPower() + RA.getPower())/2;
    }
    public int Get_Arm_Encoder()
    {
        return RA.getCurrentPosition();
    }
    public int Get_Lift_Encoder()
    {
        return LM.getCurrentPosition();
    }
    public int getArmEncoders(){
        return RA.getCurrentPosition();
    }
    /////CLAW mutator and accessor methods ///
    void setClawPosition(double position)   {claw.setPosition(position);}
    double getClawPosition() {return claw.getPosition();}


  // SET ACCESORY POWERS ///////////////////////////////////////////////////////////////////////
    void setLiftPower(double power)
    {
        LM.setPower(power);
    }
    public void setArmPower(double armPower){
        LA.setPower(armPower);
        RA.setPower(armPower);
    }


    //////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // SetMecanumPower: This sets motor powers to the four wheels, allowing movement in teleop

    public void setMecanumPower(double LeftF, double LeftB, double RightF, double RightB) {
        LF.setPower(LeftF);
        LB.setPower(LeftB);
        RF.setPower(RightF);
        RB.setPower(RightB);
    }
    //////////////////////////////////////////////////////////////////////////////////////////////




    public void moveArmDistance(double power, int distance, double EndPower) throws InterruptedException {

        RA.setTargetPosition(distance);
        RA.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        RA.setPower(power);
        LA.setPower(power);


        while (RA.isBusy()) ;
        {
            //wait until motors get to their target position
        }

        RA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RA.setPower(EndPower);
        LA.setPower(EndPower);
    }

    public void moveLiftDistance(double power, int distance, double EndPower) throws InterruptedException {

        LM.setTargetPosition(distance);
        LM.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        LM.setPower(power);


        while (LM.isBusy()) ;
        {
            //wait until motors get to their target position
        }

        LM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LM.setPower(0);
    }

    public double getLiftEncoder()
    {
        return LM.getCurrentPosition();
    }

    public double getAprilTagPosition(int tagNumber){

        int OurTag = tagNumber;
        double x_position = 0;

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if(!currentDetections.isEmpty()) {

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == OurTag) {
                    tagOfInterest = tag;
                    x_position = tagOfInterest.center.x;
                    break;
                }
            }
        }

        // camera.stopStreaming();
        return x_position;
    }

    // Legacy/Regular Movements
    /*
    // HEADING

    // Read and Return Yaw angle from IMU (Heading)
    double getHeading() {
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();
        return robotOrientation.getYaw(AngleUnit.DEGREES);
    }

    // turnToHeading
    //
    // Input - desiredHeading - the angle you want the robot to stop at
    //
    // Function -   This method has the robot calculate the shortest path
    //              to the desired angle using the absolute value of the difference of the angles
    //              compared to the signed value to determine which way to turn
    //
    // Local Variables -    fastTurn , slowTurn , closeValue
    void turnToHeading(double desiredHeading) {
        //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //double ourHeading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        double ourHeading = robotOrientation.getYaw(AngleUnit.DEGREES);

        // speed for fast turning
        double fastSpeed = .5;
        //speed for slow turning
        double slowSpeed = .1;
        // degrees away from target speed changes from slow to fast
        int speedDegree = 30;

        // make the heading into an int to create a wider range for stopping on the desired heading
        int intHeading = (int) ourHeading;

        // Keep in the loop until the robots heading equals the desired heading
        while (intHeading != desiredHeading && myOpMode.opModeIsActive()) {
            //difference shows us when the robot is farther than 180 degrees away from its target
            // This helps us decide which path is shortest to the desired heading
            double difference = Math.abs(intHeading - desiredHeading);

            if (intHeading > desiredHeading && difference > speedDegree) {
                //If robot is closer than 180 degrees, turn right
                if (difference < 180) {
                    turnRight(fastSpeed);
                }
                // Otherwise the robot is farther than 180 and turning left is the shorter path.
                else {
                    turnLeft(fastSpeed);
                }

            }
            //Robot is closer than "speedDegree" degrees away from target, slow down turn
            if (intHeading > desiredHeading && difference < speedDegree) {
                turnRight(slowSpeed);
            }

            if (intHeading < desiredHeading && difference > speedDegree) {
                //If robot is farther than 180 degrees, turn right
                if (difference > 180) {
                    turnRight(fastSpeed);
                }
                // Otherwise the robot is closer to desired heading by turning left.
                else {
                    turnLeft(fastSpeed);
                }
            }
            //Robot is closer than "speedDegree" degrees away from target, slow down turn
            if (intHeading < desiredHeading && difference < speedDegree) {
                turnLeft(slowSpeed);
            }

            //ourheading is the angle read from the robots' gyro sensor
           // angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //ourHeading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
            robotOrientation = imu.getRobotYawPitchRollAngles();
            ourHeading = robotOrientation.getYaw(AngleUnit.DEGREES);

            intHeading = (int) ourHeading;
            myOpMode.telemetry.addData("loopHeading", ourHeading);
            myOpMode.telemetry.addData("intHeading", intHeading);
            myOpMode.telemetry.addData("difference", Math.abs(ourHeading - desiredHeading));
            myOpMode.telemetry.update();
        } // END OF WHILE LOOP
        StopDriving();
    } //END OF TURNTOHEADING

    ///////////////////////////
    //End of REV_IMU methods///
    //////////////////////////
    public void turnRight(double power) {
        LB.setPower(power);
        LF.setPower(power);
        RB.setPower(-power);
        RF.setPower(-power);
    }

    public void turnLeft(double power) {
        LB.setPower(-power);
        LF.setPower(-power);
        RB.setPower(power);
        RF.setPower(power);
    }

    //Drives Backward at power for time milliseconds
    public void DriveForwardTime(double power, long time) throws InterruptedException {
        LB.setPower(power);
        LF.setPower(power);
        RB.setPower(power);
        RF.setPower(power);
        sleep(time);
        StopDriving();
    }

    //Drives Backward at power for time milliseconds
    public void DriveBackwardTime(double power, long time) throws InterruptedException {
        LB.setPower(-power);
        LF.setPower(-power);
        RB.setPower(-power);
        RF.setPower(-power);
        sleep(time);
        StopDriving();
    }

    // Strafe Left at power for time in milliseconds
    public void StrafeLeft(double power) throws InterruptedException {
        LB.setPower(power);
        LF.setPower(-power);
        RB.setPower(-power);
        RF.setPower(power);
    }

    // Strafe Right at power for time in milliseconds
    public void StrafeRight(double power) throws InterruptedException {
        LB.setPower(-power);
        LF.setPower(power);
        RB.setPower(power);
        RF.setPower(-power);
    }

    // Stops the motors by setting both to zero... no parameters needed
    public void StopDriving() {
        LB.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        RF.setPower(0);

    }

    //Method to quickly reset all encoders to zero.
    public void resetEncoders() {
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //Method to drive forward a distance at power using motor encoders
    public void SlowDriveForwardDistance(double power, int distance) throws InterruptedException {
        //this resets the encoders to zero
        int slowDistance = 200;

        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //If the distance is greater than slowDistance then run at given speed to
        // distance-slowDistance location. Then to make sure we dont slide go at .1 power until original distance is met.
        if(distance > slowDistance) {
            LB.setTargetPosition(slowDistance);
            LF.setTargetPosition(slowDistance);
            RB.setTargetPosition(slowDistance);
            RF.setTargetPosition(slowDistance);


            //this is telling the motors to run to the desired position listed above
            LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //we are setting power to the motors
            LB.setPower(.1);
            LF.setPower(.1);
            RB.setPower(.1);
            RF.setPower(.1);

            //while the four motors connected to our wheels haven't found their positions nothing will happen
            while (LB.isBusy() && LF.isBusy() && RB.isBusy() && RF.isBusy()) ;
            {
                //wait until motors get to their target position
            }

        }
        //Set the target position to the original distance (if the original call was less than slowDistance this will
        // be the only speed it uses... if it is greater then it will use the speed provided for the first distance-slowDistance
        // and then use .1 for the remaining slowDistance
        LB.setTargetPosition(distance-slowDistance);
        LF.setTargetPosition(distance-slowDistance);
        RB.setTargetPosition(distance-slowDistance);
        RF.setTargetPosition(distance-slowDistance);

        LB.setPower(power);
        LF.setPower(power);
        RB.setPower(power);
        RF.setPower(power);

        //while the four motors connected to our wheels haven't found their positions nothing will happen
        while (LB.isBusy() && LF.isBusy() && RB.isBusy() && RF.isBusy()) ;
        {
            //wait until motors get to their target position
        }

        LB.setTargetPosition(distance);
        LF.setTargetPosition(distance);
        RB.setTargetPosition(distance);
        RF.setTargetPosition(distance);

        LB.setPower(.1);
        LF.setPower(.1);
        RB.setPower(.1);
        RF.setPower(.1);

        //while the four motors connected to our wheels haven't found their positions nothing will happen
        while (LB.isBusy() && LF.isBusy() && RB.isBusy() && RF.isBusy()) ;
        {
            //wait until motors get to their target position
        }

        StopDriving();

        //  we put the motors back to using encoders that way if the method needs to be run again it will still be counting rotations
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //Method to drive backward a distance at power using motor encoders
    public void DriveBackwardDistance(double power, int distance) throws InterruptedException {
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LB.setTargetPosition(-distance);
        LF.setTargetPosition(-distance);
        RB.setTargetPosition(-distance);
        RF.setTargetPosition(-distance);

        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LB.setPower(-power);
        LF.setPower(-power);
        RB.setPower(-power);
        RF.setPower(-power);

        while (LB.isBusy() && LF.isBusy() && RB.isBusy() && RF.isBusy()) ;
        {
            //wait until motors get to their target position
        }

        StopDriving();

        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    public void StrafeDistance(double power, int distance) throws InterruptedException {

        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LB.setTargetPosition(-distance);
        LF.setTargetPosition(distance);
        RB.setTargetPosition(distance);
        RF.setTargetPosition(-distance);

        LB.setPower(-power);
        LF.setPower(power);
        RB.setPower(power);
        RF.setPower(-power);

        //while the four motors connected to our wheels haven't found their positions nothing will happen
        while (LB.isBusy() && LF.isBusy() && RB.isBusy() && RF.isBusy()) ;
        {
            //wait until motors get to their target position
        }

        StopDriving();

        //  we put the motors back to using encoders that way if the method needs to be run again it will still be counting rotations
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void DriveForwardDistance(double power, int distance) throws InterruptedException {
        //this resets the encoders to zero
        int slowDistance = 200;

        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //If the distance is greater than slowDistance then run at given speed to
        // distance-slowDistance location. Then to make sure we dont slide go at .1 power until original distance is met.
        if(distance > slowDistance) {
            LB.setTargetPosition(distance - slowDistance);
            LF.setTargetPosition(distance - slowDistance);
            RB.setTargetPosition(distance - slowDistance);
            RF.setTargetPosition(distance - slowDistance);


            //this is telling the motors to run to the desired position listed above
            LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //we are setting power to the motors
            LB.setPower(power);
            LF.setPower(power);
            RB.setPower(power);
            RF.setPower(power);

            //while the four motors connected to our wheels haven't found their positions nothing will happen
            while (LB.isBusy() && LF.isBusy() && RB.isBusy() && RF.isBusy()) ;
            {
                //wait until motors get to their target position
            }

        }
        //Set the target position to the original distance (if the original call was less than slowDistance this will
        // be the only speed it uses... if it is greater then it will use the speed provided for the first distance-slowDistance
        // and then use .1 for the remaining slowDistance
        LB.setTargetPosition(distance);
        LF.setTargetPosition(distance);
        RB.setTargetPosition(distance);
        RF.setTargetPosition(distance);

        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LB.setPower(.1);
        LF.setPower(.1);
        RB.setPower(.1);
        RF.setPower(.1);

        //while the four motors connected to our wheels haven't found their positions nothing will happen
        while (LB.isBusy() && LF.isBusy() && RB.isBusy() && RF.isBusy()) ;
        {
            //wait until motors get to their target position
        }

        StopDriving();

        //  we put the motors back to using encoders that way if the method needs to be run again it will still be counting rotations
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //Method to drive forward a distance at power using motor encoders
    public void DriveForwardDistanceGradual(int distance, double maxPower) throws InterruptedException {
        //this resets the encoders to zero

        double power = .2;

        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LB.setTargetPosition(distance);
        LF.setTargetPosition(distance);
        RB.setTargetPosition(distance);
        RF.setTargetPosition(distance);

        int numberOfLoops = 14;
        int distancePerLoop = Math.floorDiv(distance,numberOfLoops);
        int traveledDistance = distancePerLoop;
        int loopMultiplier = 1;
        int loopMultiplier2 = 1;


        //this is telling the motors to run to the desired position listed above
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //we are setting power to the motors
        LB.setPower(power);
        LF.setPower(power);
        RB.setPower(power);
        RF.setPower(power);


        //while the four motors connected to our wheels haven't found their positions nothing will happen
        while (LB.isBusy() && LF.isBusy() && RB.isBusy() && RF.isBusy())
        {
            while(loopMultiplier<numberOfLoops/2) {
                if (RB.getCurrentPosition() > traveledDistance) {
                    loopMultiplier += 1;
                    if(power * loopMultiplier < maxPower)
                    {
                        LB.setPower(power * loopMultiplier);
                        LF.setPower(power * loopMultiplier);
                        RB.setPower(power * loopMultiplier);
                        RF.setPower(power * loopMultiplier);
                    }
                    else
                    {
                        LB.setPower(maxPower);
                        LF.setPower(maxPower);
                        RB.setPower(maxPower);
                        RF.setPower(maxPower);
                    }
                    traveledDistance = distancePerLoop * loopMultiplier;
                    loopMultiplier2 = loopMultiplier;
                }
            }
            if (RB.getCurrentPosition() > traveledDistance) {
                loopMultiplier2 -= 1;
                loopMultiplier += 1;
                if(loopMultiplier2>0)
                {
                    if (power * loopMultiplier2 < maxPower)
                    {
                        LB.setPower(power * loopMultiplier2);
                        LF.setPower(power * loopMultiplier2);
                        RB.setPower(power * loopMultiplier2);
                        RF.setPower(power * loopMultiplier2);
                    }
                    else
                    {
                        LB.setPower(maxPower);
                        LF.setPower(maxPower);
                        RB.setPower(maxPower);
                        RF.setPower(maxPower);
                    }
                }
                traveledDistance = distancePerLoop * loopMultiplier;
            }
        }


        //  we put the motors back to using encoders that way if the method needs to be run again it will still be counting rotations
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        StopDriving();

    }

    //Method to drive forward a distance at power using motor encoders
    public void DriveBackwardDistanceGradual(int distance, double maxPower) throws InterruptedException {

        //this resets the encoders to zero
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double power = -.2;

        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LB.setTargetPosition(distance);
        LF.setTargetPosition(distance);
        RB.setTargetPosition(distance);
        RF.setTargetPosition(distance);

        int numberOfLoops = 14;
        int distancePerLoop = Math.floorDiv(distance,numberOfLoops);
        int traveledDistance = distancePerLoop;
        int loopMultiplier = 1;
        int loopMultiplier2 = 1;


        //this is telling the motors to run to the desired position listed above
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //we are setting power to the motors
        LB.setPower(power);
        LF.setPower(power);
        RB.setPower(power);
        RF.setPower(power);


        //while the four motors connected to our wheels haven't found their positions nothing will happen
        while (LB.isBusy() && LF.isBusy() && RB.isBusy() && RF.isBusy())
        {
            while(loopMultiplier<numberOfLoops/2) {
                if (RB.getCurrentPosition() > traveledDistance) {
                    loopMultiplier += 1;
                    if(power * loopMultiplier < -maxPower)
                    {
                        LB.setPower(power * loopMultiplier);
                        LF.setPower(power * loopMultiplier);
                        RB.setPower(power * loopMultiplier);
                        RF.setPower(power * loopMultiplier);
                    }
                    else
                    {
                        LB.setPower(-maxPower);
                        LF.setPower(-maxPower);
                        RB.setPower(-maxPower);
                        RF.setPower(-maxPower);
                    }
                    traveledDistance = distancePerLoop * loopMultiplier;
                    loopMultiplier2 = loopMultiplier;
                }
            }
            if (RB.getCurrentPosition() > traveledDistance) {
                loopMultiplier2 -= 1;
                loopMultiplier += 1;
                if(loopMultiplier2>0)
                {
                    if (power * loopMultiplier2 < -maxPower)
                    {
                        LB.setPower(power * loopMultiplier2);
                        LF.setPower(power * loopMultiplier2);
                        RB.setPower(power * loopMultiplier2);
                        RF.setPower(power * loopMultiplier2);
                    }
                    else
                    {
                        LB.setPower(-maxPower);
                        LF.setPower(-maxPower);
                        RB.setPower(-maxPower);
                        RF.setPower(-maxPower);
                    }
                }
                traveledDistance = distancePerLoop * loopMultiplier;
            }
        }


        //  we put the motors back to using encoders that way if the method needs to be run again it will still be counting rotations
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        StopDriving();

    }


    public void Drive(int distance, double maxPower, double desiredHeading) throws InterruptedException {

        turnToHeading(desiredHeading);

        //this resets the encoders to zero

        double currentHeading = 0;
        double error = 0;
        double errorPower = 0;
        double power = .2;
        int sign = 1;

        YawPitchRollAngles robotOrientation;


        if (distance < 0)
        {
            sign = -1;
        }

        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LB.setTargetPosition(distance);
        LF.setTargetPosition(distance);
        RB.setTargetPosition(distance);
        RF.setTargetPosition(distance);

        int numberOfLoops = 14;
        int distancePerLoop = Math.floorDiv(distance,numberOfLoops);
        int traveledDistance = distancePerLoop;
        int loopMultiplier = 1;
        int loopMultiplier2 = 1;


        //this is telling the motors to run to the desired position listed above
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //we are setting power to the motors
        LB.setPower(sign*power);
        LF.setPower(sign*power);
        RB.setPower(sign*power);
        RF.setPower(sign*power);


        //while the four motors connected to our wheels haven't found their positions nothing will happen
        while (LB.isBusy() && LF.isBusy() && RB.isBusy() && RF.isBusy())
        {
            while(loopMultiplier<numberOfLoops/2)
            {
               // angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
               // currentHeading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
                robotOrientation = imu.getRobotYawPitchRollAngles();
                currentHeading = robotOrientation.getYaw(AngleUnit.DEGREES);
                //negative error if drifting left
                error = desiredHeading - currentHeading;

                errorPower = error/40;

                if (sign*RB.getCurrentPosition() > sign*traveledDistance) {
                    loopMultiplier += 1;
                    if(power * loopMultiplier < maxPower)
                    {
                        LB.setPower(sign*(power * loopMultiplier) - errorPower);
                        LF.setPower(sign*(power * loopMultiplier) - errorPower);
                        RB.setPower(sign*(power * loopMultiplier) + errorPower);
                        RF.setPower(sign*(power * loopMultiplier) + errorPower);
                    }
                    else
                    {
                        LB.setPower(sign*(maxPower) - errorPower);
                        LF.setPower(sign*(maxPower) - errorPower);
                        RB.setPower(sign*(maxPower) + errorPower);
                        RF.setPower(sign*(maxPower) + errorPower);
                    }
                    traveledDistance = distancePerLoop * loopMultiplier;
                    loopMultiplier2 = loopMultiplier;
                }
            }
            if (sign*RB.getCurrentPosition() > sign*traveledDistance)
            {

                //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //currentHeading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);

                robotOrientation = imu.getRobotYawPitchRollAngles();
                currentHeading = robotOrientation.getYaw(AngleUnit.DEGREES);

                //negative error if drifting left
                error = desiredHeading - currentHeading;

                errorPower = error/40;


                loopMultiplier2 -= 1;
                loopMultiplier += 1;
                if(loopMultiplier2>0)
                {
                    if (power * loopMultiplier2 < maxPower)
                    {
                        LB.setPower(sign*(power * loopMultiplier2) - errorPower);
                        LF.setPower(sign*(power * loopMultiplier2) - errorPower);
                        RB.setPower(sign*(power * loopMultiplier2) + errorPower);
                        RF.setPower(sign*(power * loopMultiplier2) + errorPower);
                    }
                    else
                    {
                        LB.setPower(sign*(maxPower) - errorPower);
                        LF.setPower(sign*(maxPower) - errorPower);
                        RB.setPower(sign*(maxPower) + errorPower);
                        RF.setPower(sign*(maxPower) + errorPower);
                    }
                }
                traveledDistance = distancePerLoop * loopMultiplier;



            }
        }


        //  we put the motors back to using encoders that way if the method needs to be run again it will still be counting rotations
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        StopDriving();

        turnToHeading(desiredHeading);



 */

    // Roadrunner


    }



    ///////////////////END OF METHODS///////////////////

