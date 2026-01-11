package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="CLOSE_BLUE", group="AutoStuff")

public class AUTO_BLUE extends LinearOpMode {

    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;
    public NormalizedColorSensor color;
    boolean Green = false;
    boolean Purple = false;

    double hue;
    public void runOpMode() throws InterruptedException {

        int tagdetected = 0;
        GearHeadRobot robot = new GearHeadRobot(this);
        // NOTE THE IMU IS LOGO BACKWARDS // USB UP AS OF TESTING THIS
        // IF THE CONTROL HUB IS MOUNTED DIFFERENTLY YOU HAVE TO CHANGE IT IN
        // THE GearHeadRobot CLASS!!
        robot.init();
        color = robot.myOpMode.hardwareMap.get(NormalizedColorSensor.class, "color");
        initAprilTag();
        robot.imu.resetYaw();
        robot.resetEncoders();
        waitForStart();

        ///////////////////////////////////////////////////////////////////////////////////////////////
        // Everything below this line is what the robot is actually doing during the autonomous mode //
        ///////////////////////////////////////////////////////////////////////////////////////////////
        // (1 intake): .25
        ///  2 Intake: .67
        ///  3 intake: 1
        ///  1 Shoot: .87
        // 2 Shoot: .05
        //  3 Shoot: .45


        /// TODO:
        /// - Make accurate color sensing method for switching to the correct color ball
        /// - Have 3 different functions for the 3 different april tag motifs (if GPP, do xyz, if PGP do xyz)
        /// - RoadRunner movements setup
        /// - Test the autonomous program for accuracy

        robot.launcherPower(.75);

        robot.Drive(3384,1,0);
        // drive forward 3384
        robot.turnToHeading(35);
        robot.cyclerPos(.87);
        sleep(1000);
        robot.pusherPos(.34);
        sleep(1000);
        robot.pusherPos(.55);
        sleep(1300);
        robot.cyclerPos(.05);
        sleep(1000);
        robot.pusherPos(.34);
        sleep(1000);
        robot.pusherPos(.55);
        sleep(2000);
        robot.cyclerPos(.45);
        sleep(1000);
        robot.pusherPos(.34);
        sleep(1000);
        robot.pusherPos(.55);
robot.turnToHeading(87);
sleep(500);
robot.Drive(1175,1,90);
// add intake later
        sleep(3000);

        robot.Drive(-1175,1,90);
        robot.turnToHeading(35);
        robot.cyclerPos(.87);
        sleep(1000);
        robot.pusherPos(.34);
        sleep(1000);
        robot.pusherPos(.55);
        sleep(1300);
        robot.cyclerPos(.05);
        sleep(1000);
        robot.pusherPos(.34);
        sleep(1000);
        robot.pusherPos(.55);
        sleep(2000);
        robot.cyclerPos(.45);
        sleep(1000);
        robot.pusherPos(.34);
        sleep(1000);
        robot.pusherPos(.55);









}


    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);


    }   // end method initAprilTag()
private void ColorSensor() {
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
        }

}





