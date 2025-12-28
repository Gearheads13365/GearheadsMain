package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@TeleOp(name="Teleop", group="Linear Opmode")

public class Teleop extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        double x_position = 0;

        GearHeadRobot robot = new GearHeadRobot(this);
        robot.init();
        robot.imu.resetYaw();
        robot.resetEncoders();
        boolean BackOn = false;
        boolean FrontOn = false;


        ElapsedTime buttonTimer = new ElapsedTime();

        double liftPower = -1;
        int precisePower = 1;
        int liftState = 0;


        waitForStart();

        while (opModeIsActive()) {

            // CONTROLS FOR TELEOP //
            //Set mecanum power
            robot.setMecanumPower(
                    ((-gamepad1.left_stick_y + gamepad1.left_stick_x) + (gamepad1.right_stick_x)) / precisePower,
                    ((-gamepad1.left_stick_y - gamepad1.left_stick_x) + (gamepad1.right_stick_x)) / precisePower,
                    ((-gamepad1.left_stick_y - gamepad1.left_stick_x) - (gamepad1.right_stick_x)) / precisePower,
                    ((-gamepad1.left_stick_y + gamepad1.left_stick_x) - (gamepad1.right_stick_x)) / precisePower);


            // GAMEPAD 1 BUTTONS // Using only 1 Gamepad!!!
            // Open
            if (gamepad1.dpad_left) {
                robot.resetEncoders();
            }
            // Hold
            // Launch
            if (gamepad1.dpad_up) {
                robot.setHL(1);
            }

            if (gamepad1.dpad_down) {
                  robot.setHL(-1);

            }
            if (gamepad1.dpad_right){
                robot.setHL(0);
            }


            if (gamepad1.right_trigger > 0.5) {
                precisePower = 3;
            } else {
                precisePower = 1;
            }


            if (gamepad1.a && buttonTimer.milliseconds() > 200) {
                if (robot.GetIntakePower() < -.4) {
                    robot.intakePower(0);
                } else {
                    robot.intakePower(-.5);
                }
                buttonTimer.reset();
            }

            if (gamepad1.b && buttonTimer.milliseconds() > 200) {
                if (robot.GetShooterPower() > .5) {
                    robot.shooterPower(0);
                } else {
                    // WAS .85
                    robot.shooterPower(.85);
                }
                buttonTimer.reset();
            }
            if (gamepad1.y && buttonTimer.milliseconds() > 200) {
                if (BackOn) {
                    robot.backStage(0);
                    BackOn = false;
                } else {
                    robot.backStage(1);
                    BackOn = true;
                }
                buttonTimer.reset();
            }
            if (gamepad1.x) {
                robot.frontStage(1);
            } else {
                robot.frontStage(0);
            }


            if (gamepad1.right_bumper) {
                robot.intakePower(.5);
            }

            if (gamepad1.left_trigger > 0.2) {
            } else if (gamepad2.left_trigger == 0) {

            }

        }


        telemetry.update();
        /*
        robot.telemetryAprilTag();
        telemetry.addData("Number of AprilTags Detected", currentDetections.size());
        for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : currentDetections) {
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

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

         */
    }
}


        // END OF LOOP






