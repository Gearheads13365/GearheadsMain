package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;

@TeleOp(name="Teleop", group="Linear Opmode")

public class Teleop extends LinearOpMode {
    public NormalizedColorSensor color;
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;



    public void runOpMode() throws InterruptedException {
        double x_position = 0;
        GearHeadRobot robot = new GearHeadRobot(this);

        // color must be called seperately in each opmode it is used in - the color sensor only works by directly calling it in the opmode, not in GHR
        color = robot.myOpMode.hardwareMap.get(NormalizedColorSensor.class, "color");

        robot.init();
        robot.imu.resetYaw();
        robot.resetEncoders();
        initAprilTag();
        boolean Green = false;
        boolean Purple = false;

        double hue;

        ElapsedTime buttonTimer = new ElapsedTime();
        int precisePower = 1;


        waitForStart();



        while (opModeIsActive()) {

            {  // COLOR SENSING - must be copied to each opmode it is used in
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
                // END COLOR SENSING
            }
            telemetryAprilTag();

            // CONTROLS FOR TELEOP //
            //Set mecanum power
            robot.setMecanumPower(
                    ((-gamepad1.left_stick_y + gamepad1.left_stick_x) + (gamepad1.right_stick_x)) / precisePower,
                    ((-gamepad1.left_stick_y - gamepad1.left_stick_x) + (gamepad1.right_stick_x)) / precisePower,
                    ((-gamepad1.left_stick_y - gamepad1.left_stick_x) - (gamepad1.right_stick_x)) / precisePower,
                    ((-gamepad1.left_stick_y + gamepad1.left_stick_x) - (gamepad1.right_stick_x)) / precisePower);


            // GAMEPAD 1 BUTTONS

            if (gamepad1.a){
                robot.resetEncoders();
            }

            // (1 intake): .25
            ///  2 Intake: .67
            ///  3 intake: 1
            ///  1 Shoot: .87
            // 2 Shoot: .05
            //  3 Shoot: .45
            /// GAMEPAD 2

            if (gamepad2.a) {
                robot.pusherPos(.34);
            }
            else {
                robot.pusherPos(.55);
            }




            if (gamepad2.b) {
                robot.cyclerPos(.45);
            }

            if (gamepad2.x){
                robot.cyclerPos(.87);
            }
            if (gamepad2.y){
                robot.cyclerPos(.05);
            }

                // (1 intake): .25
                ///  2 Intake: .67
                ///  3 intake: 1
            if (gamepad2.dpad_up) {
                robot.cyclerPos(.67);
            }


            if (gamepad2.dpad_left) {
                robot.cyclerPos(.25);

            }
            if (gamepad2.dpad_right) {
                robot.cyclerPos(1);
            }

            if(gamepad2.left_bumper){
                robot.intakePower(-1);
            }
            if(gamepad2.right_bumper){
                robot.intakePower(0);
            }
            if (gamepad2.left_trigger > 0){
                robot.launcherPower(.8);
            }
            if (gamepad2.right_trigger > 0){
robot.launcherPower(0);
            }

telemetry.addData("Distance",robot.GetMotorEncoders());
            telemetry.update();

        }
/// END LOOP
        }

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);


    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop


    }   // end method telemetryAprilTag()








    }




        // END OF LOOP






