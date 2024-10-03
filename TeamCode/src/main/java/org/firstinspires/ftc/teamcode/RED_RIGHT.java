package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RED_RIGHT", group="AutoStuff")

public class RED_RIGHT extends LinearOpMode {
    public void runOpMode() throws InterruptedException {


        GearHeadRobot robot = new GearHeadRobot(this);
        // NOTE THE IMU IS LOGO BACKWARDS // USB UP AS OF TESTING THIS
        // IF THE CONTROL HUB IS MOUNTED DIFFERENTLY YOU HAVE TO CHANGE IT IN
        // THE GearHeadRobot CLASS!!
        robot.init();

        robot.setClawPosition(robot.close);

        robot.imu.resetYaw();

        waitForStart();

        ///////////////////////////////////////////////////////////////////////////////////////////////
        // Everything below this line is what the robot is actually doing during the autonomous mode //
        ///////////////////////////////////////////////////////////////////////////////////////////////
        robot.resetEncoders();

        int pixelPosition = robot.getIconPosition("RED_RIGHT");
        telemetry.addData("pixelPosition", pixelPosition);
        telemetry.addData("iconAnalysisMiddle", robot.iconAnalysisMiddle);
        telemetry.addData("iconAnalysisRight", robot.iconAnalysisRight);
        telemetry.update();


        // Pixel in position 6 (right side)
        if (pixelPosition == 6)
        {
            robot.StrafeDistance(0.5, 420);
            sleep(300);
            robot.Drive(1150, 0.5, 0);
            sleep(300);
            //robot.deliverPurplePixel();
            robot.Drive(-650, 0.5, 0);
            sleep(300);
        }
        // Pixel in position 5 (middle)
        else if (pixelPosition == 5)
        {
            robot.Drive(1630, 0.5, 0);
            sleep(300);
            //robot.deliverPurplePixel();
            robot.Drive(-1130, 0.5, 0);
            sleep(300);
            robot.StrafeDistance(0.5, 420);
            sleep(300);

        }
        // Pixel in position 4 (left side)
        else //pixelPosition == 4
        {
            robot.Drive(1250, 0.5, 0);
            sleep(300);
            robot.Drive(500, 0.5, 90);
            sleep(300);
            //robot.deliverPurplePixel();
            robot.Drive(-550, 0.5, 90);
            sleep(300);
            robot.Drive(-750, 0.5, 0);
            sleep(300);
            robot.StrafeDistance(0.5, 370);
            sleep(300);
        }

        robot.Drive(850, 0.5, -90);
        robot.setMecanumPower(0.01, 0.01,0.01,0.01);
        robot.StopDriving();
        sleep(300);
        robot.turnToHeading(-90);
        sleep(300);
        robot.StrafeDistance(0.5, -600);
        robot.resetEncoders();
        robot.StrafeLeft(0.1);

        telemetry.addData("pixelPosition", pixelPosition);
        telemetry.update();

        robot.webcam.setPipeline(robot.aprilTagDetectionPipeline);

        boolean missedAprilTag = false;
        boolean found = false;
        while(!found)
        {
            if(robot.getAprilTagPosition(6) > 800)
            {
                found = true;
            }
            else if (robot.GetMotorEncoders() > 1800)
            {
                found = true;
                missedAprilTag = true;
            }
            telemetry.addData("Tag Location", robot.getAprilTagPosition(6));
            telemetry.update();
        }
        robot.StopDriving();
        telemetry.addData("Tag Location", robot.getAprilTagPosition(6));
        telemetry.update();




        if (pixelPosition == 4 && missedAprilTag == false)
        {
            robot.StrafeDistance(0.2, -250);
            sleep(300);
            robot.deliverYellowPixel("RED");
            robot.StrafeDistance(0.5, -1600);
            sleep(300);
        }
        else if (pixelPosition == 5 && missedAprilTag == false)
        {
            //in correct position
            robot.deliverYellowPixel("RED");
            sleep(300);
            robot.StrafeDistance(0.5, -1300);
            sleep(300);
        }
        else if (pixelPosition == 6 && missedAprilTag == false)
        {
            robot.StrafeDistance(0.2, 400);
            sleep(300);
            robot.deliverYellowPixel("RED");
            robot.StrafeDistance(0.5, -1000);
            sleep(300);
        }
        else
        {
            robot.setArmPower(-0.5);
            sleep(300);
            robot.setArmPower(0);
        }


        robot.Drive(-650, 0.5, 90);
        sleep(300);


        telemetry.update();
        sleep(10000);

        robot.webcam.stopStreaming();
        robot.StopDriving();
        //End of Autonomous Program
    }
}