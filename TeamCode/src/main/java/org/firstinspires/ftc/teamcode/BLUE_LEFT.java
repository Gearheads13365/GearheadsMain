package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="BLUE_LEFT", group="AutoStuff")

public class BLUE_LEFT extends LinearOpMode {
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

        int pixelPosition = robot.getIconPosition("BLUE_LEFT");
        telemetry.addData("pixelPosition", pixelPosition);
        telemetry.addData("iconAnalysisMiddle", robot.iconAnalysisMiddle);
        telemetry.addData("iconAnalysisRight", robot.iconAnalysisRight);
        telemetry.update();


        // Pixel in position 3 (right side)
        if (pixelPosition == 3)
        {
            robot.Drive(1250, 0.5, 0);
            sleep(300);
            robot.Drive(500, 0.5, -90);
            sleep(300);
            //robot.deliverPurplePixel();
            robot.Drive(-550, 0.5, -90);
            sleep(300);
            robot.Drive(-750, 0.5, 0);
            sleep(300);
            robot.StrafeDistance(0.5, -370);
            sleep(300);
        }
        // Pixel in position 2 (middle)
        else if (pixelPosition == 2)
        {
            robot.Drive(1630 , 0.5, 0);
            sleep(300);
            //robot.deliverPurplePixel();
            robot.Drive(-1130, 0.5, 0);
            sleep(300);
            robot.StrafeDistance(0.5, -420);
            sleep(300);

        }
        // Pixel in position 1 (left side)
        else //pixelPosition == 1
        {
            robot.StrafeDistance(0.5, -420);
            sleep(300);
            robot.Drive(1150, 0.5, 0);
            sleep(300);
            //robot.deliverPurplePixel();
            robot.Drive(-650, 0.5, 0);
            sleep(300);
        }

        robot.Drive(850, 0.5, 90);
        robot.setMecanumPower(0.01, 0.01,0.01,0.01);
        robot.StopDriving();
        sleep(300);
        robot.turnToHeading(90);
        sleep(300);
        robot.StrafeDistance(0.5, 300);
        robot.resetEncoders();

        robot.webcam.setPipeline(robot.aprilTagDetectionPipeline);
        robot.StrafeRight(0.1);

        boolean missedAprilTag = false;
        boolean found = false;
        while(!found)
        {
            if(robot.getAprilTagPosition(2) < 850 && robot.getAprilTagPosition(2)>0)
            {
                found = true;
            }
            else if (robot.GetMotorEncoders() > 1800)
            {
                found = true;
                missedAprilTag = true;
            }
            telemetry.addData("Encoders", robot.GetMotorEncoders());
            telemetry.addData("Tag Location", robot.getAprilTagPosition(2));
            telemetry.update();
        }

        robot.StopDriving();
        telemetry.addData("Encoders", robot.GetMotorEncoders());
        telemetry.addData("Tag Location", robot.getAprilTagPosition(2));
        telemetry.update();



        if (pixelPosition == 1 && missedAprilTag == false)
        {
            //in correct position
            robot.deliverYellowPixel("BLUE");
            sleep(300);
            robot.StrafeDistance(0.5, 1000);
            sleep(300);
        }
        else if (pixelPosition == 2 && missedAprilTag == false)
        {
            robot.StrafeDistance(0.2, 300);
            sleep(300);
            robot.deliverYellowPixel("BLUE");
            robot.StrafeDistance(0.5, 1300);
            sleep(300);
        }
        else if (pixelPosition == 3 && missedAprilTag == false)
        {
            robot.StrafeDistance(0.2, 600);
            sleep(300);
            robot.deliverYellowPixel("BLUE");
            robot.StrafeDistance(0.5, 1600);
            sleep(300);
        }
        else
        {
            robot.setArmPower(-0.5);
            sleep(300);
            robot.setArmPower(0);
        }

        robot.Drive(-650, 0.5, -90);
        sleep(300);


        telemetry.update();

        robot.webcam.stopStreaming();
        robot.StopDriving();
        //End of Autonomous Program
    }
}
