package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="BLUE_RIGHT", group="AutoStuff")

public class BLUE_RIGHT extends LinearOpMode {
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

      //End of Autonomous Program
    }
}