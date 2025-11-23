package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AUTO_RED", group="AutoStuff")

public class AUTO_RED2 extends LinearOpMode {
    public void runOpMode() throws InterruptedException {


        GearHeadRobot robot = new GearHeadRobot(this);
        // NOTE THE IMU IS LOGO BACKWARDS // USB UP AS OF TESTING THIS
        // IF THE CONTROL HUB IS MOUNTED DIFFERENTLY YOU HAVE TO CHANGE IT IN
        // THE GearHeadRobot CLASS!!
        robot.init();



        robot.imu.resetYaw();

        waitForStart();

        ///////////////////////////////////////////////////////////////////////////////////////////////
        // Everything below this line is what the robot is actually doing during the autonomous mode //
        ///////////////////////////////////////////////////////////////////////////////////////////////

        robot.shooterPower(.87);

        robot.Drive(-1500, .5, 0);
        robot.StrafeDistance(.5,80);
        robot.backStage(1);
        robot.frontStage(1);
        sleep(500);
        robot.setHL(0);
        sleep(500);
        robot.setHL(1);
        sleep(1000);
        robot.setHL(0);
        sleep(500);

        //2nd ball
        robot.shooterPower(.81);
        robot.frontStage(1);
        sleep(2000);
        robot.setHL(0);
        sleep(500);
        robot.setHL(1);
        sleep(500);
        robot.setHL(0);
        robot.frontStage(0);
        robot.intakePower(1);
        sleep(500);
        robot.backStage(0);
        // 3rd


        robot.StrafeDistance(.5,1200);














        //End of Autonomous Program
    }
}