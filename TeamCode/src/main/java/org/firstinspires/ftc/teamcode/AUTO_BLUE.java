package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Autonomous(name="CLOSE_BLUE", group="AutoStuff")

public class AUTO_BLUE extends LinearOpMode {


    public NormalizedColorSensor color;
    boolean Green = false;
    boolean Purple = false;
    double hue;
    public void runOpMode() throws InterruptedException {


        GearHeadRobot robot = new GearHeadRobot(this);
        // NOTE THE IMU IS LOGO BACKWARDS // USB UP AS OF TESTING THIS
        // IF THE CONTROL HUB IS MOUNTED DIFFERENTLY YOU HAVE TO CHANGE IT IN
        // THE GearHeadRobot CLASS!!
        robot.init();
        color = robot.myOpMode.hardwareMap.get(NormalizedColorSensor.class, "color");

        robot.imu.resetYaw();

        waitForStart();

        ///////////////////////////////////////////////////////////////////////////////////////////////
        // Everything below this line is what the robot is actually doing during the autonomous mode //
        ///////////////////////////////////////////////////////////////////////////////////////////////
        // (1 intake): .25
        ///  2 Intake: .67
        ///  3 intake: 1
        ///  1 Shoot: .9
        // 2 Shoot: .05
        //  3 Shoot: .45

sleep(500);
        {
            NormalizedRGBA colors = color.getNormalizedColors();
            hue = JavaUtil.colorToHue(colors.toColor());

            if (hue > 90 && hue < 200) {
                telemetry.addData("Color", "Green");
                Green = true;
                Purple = false;
            } else if (hue > 225 && hue < 350) {
                telemetry.addData("Color", "Purple");
                Purple = true;
                Green = false;
            } else {
                telemetry.addData("Color", "None");
                Green = false;
                Purple = false;
            }
            color.close();
        }

        while (!Purple){
            robot.cyclerPos(.9);
            {
                NormalizedRGBA colors = color.getNormalizedColors();
                hue = JavaUtil.colorToHue(colors.toColor());

                if (hue > 90 && hue < 200) {
                    telemetry.addData("Color", "Green");
                    Green = true;
                    Purple = false;
                } else if (hue > 225 && hue < 350) {
                    telemetry.addData("Color", "Purple");
                    Purple = true;
                    Green = false;
                } else {
                    telemetry.addData("Color", "None");
                    Green = false;
                    Purple = false;
                }
                color.close();
            }
            sleep(1000);

            robot.cyclerPos(.45);
            {
                NormalizedRGBA colors = color.getNormalizedColors();
                hue = JavaUtil.colorToHue(colors.toColor());

                if (hue > 90 && hue < 200) {
                    telemetry.addData("Color", "Green");
                    Green = true;
                    Purple = false;
                } else if (hue > 225 && hue < 350) {
                    telemetry.addData("Color", "Purple");
                    Purple = true;
                    Green = false;
                } else {
                    telemetry.addData("Color", "None");
                    Green = false;
                    Purple = false;
                }
                color.close();
            }
            sleep(1000);
            robot.cyclerPos(.05);
            {
                NormalizedRGBA colors = color.getNormalizedColors();
                hue = JavaUtil.colorToHue(colors.toColor());

                if (hue > 90 && hue < 200) {
                    telemetry.addData("Color", "Green");
                    Green = true;
                    Purple = false;
                } else if (hue > 225 && hue < 350) {
                    telemetry.addData("Color", "Purple");
                    Purple = true;
                    Green = false;
                } else {
                    telemetry.addData("Color", "None");
                    Green = false;
                    Purple = false;
                }
                color.close();
            }
            sleep(1000);
        }
        robot.launcherPower(.8);
        sleep(800);
        robot.pusherPos(.44);
        sleep(2000);
        robot.pusherPos(.55);







    }
}




