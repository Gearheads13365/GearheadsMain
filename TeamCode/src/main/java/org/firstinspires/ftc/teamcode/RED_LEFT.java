package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RED_LEFT", group="AutoStuff")

public class RED_LEFT extends LinearOpMode {
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

        int pixelPosition = robot.getIconPosition("RED_LEFT");
        telemetry.addData("pixelPosition", pixelPosition);
        telemetry.addData("iconAnalysisMiddle", robot.iconAnalysisMiddle);
        telemetry.addData("iconAnalysisRight", robot.iconAnalysisRight);
        telemetry.update();



        // Pixel in position 6 (right side)
        if (pixelPosition == 6)
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
        // Pixel in position 5 (middle)
        else if (pixelPosition == 5)
        {
            robot.Drive(1630 , 0.5, 0);
            sleep(300);
            //robot.deliverPurplePixel();
            robot.Drive(-1130, 0.5, 0);
            sleep(300);
            robot.StrafeDistance(0.5, -420);
            sleep(300);
        }
        // Pixel in position 4 (left side)
        else //pixelPosition == 4
        {
            robot.StrafeDistance(0.5, -420);
            sleep(300);
            robot.Drive(1150, 0.5, 0);
            sleep(300);
            //robot.deliverPurplePixel();
            robot.Drive(-650, 0.5, 0);
            sleep(300);
        }

        robot.setArmPower(0);

        telemetry.update();

        robot.webcam.stopStreaming();
      robot.StopDriving();
      //End of Autonomous Program
    }
}