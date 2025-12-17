package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="CLOSE_BLUE", group="AutoStuff")

public class AUTO_BLUE extends LinearOpMode {
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

        // TODO:
        // - Optimize launching sequence to take the minimum amount of time possible
        // - Get robot to move to a



        robot.intakePower(-.5);
        sleep(2000);
        robot.Drive(-1500, .5, 0);
        robot.StrafeDistance(.5,80);
        // shoot
        robot.shooterPower(.78);
        sleep(4000);
        robot.backStage(.85);
        sleep(800);

        // 2nd
        robot.frontStage(.85);
        sleep(1000);
        // 3rd
        robot.setHL(1);
        sleep(4000);
        robot.frontStage(0);

        sleep(1000);
        robot.backStage(0);
        robot.intakePower(0);
        // strafe out of the launch zone

// strafe out
        robot.StrafeDistance(.5,-1200);












/*
drive
intake power
arm catch
shooter on
drive to get to launch pos
wait a second
arm launch



move to collect balls, launch, repeat


 */


        //End of Autonomous Program
    }
}




