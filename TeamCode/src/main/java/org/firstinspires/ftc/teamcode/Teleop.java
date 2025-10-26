package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.List;

@TeleOp(name="Teleop", group="Linear Opmode")

public class Teleop extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        double x_position = 0;

        GearHeadRobot robot = new GearHeadRobot(this);
        robot.init();
        robot.imu.resetYaw();
robot.resetEncoders();




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




            // GAMEPAD 1 BUTTONS //
            // Open
            if (gamepad1.a) {
                robot.MoveRightArm(.5);
                robot.MoveLeftArm(.2);
            }
            // Hold
            if (gamepad1.b) {
                robot.MoveRightArm(.4);
                robot.MoveLeftArm(.35);
            }
            // Launch
            if (gamepad1.x) {
            robot.MoveRightArm(.25);
             robot.MoveLeftArm(.5);
            }
            if (gamepad1.y) {

            }

            if (gamepad1.dpad_up) {

            }
            if (gamepad1.dpad_down) {

            }
            if (gamepad1.dpad_left) {

            }
            if (gamepad1.dpad_right) {

            }


            if (gamepad1.left_trigger > 0) {

            }
            if (gamepad1.right_trigger > 0.5) {
                precisePower = 3;
            } else {
                precisePower = 1;
            }

            if (gamepad1.left_bumper) {

            }
            if (gamepad1.right_bumper) {

            }
            //// GAMEPAD 2 ////
            if (gamepad2.a) {
robot.IntakePower(1);
            }
            if (gamepad2.b) {
robot.ShooterPower(.75);
            }
            if (gamepad2.x) {
robot.IntakePower(0);
            }
            if (gamepad2.y) {
robot.ShooterPower(0);
            }

            if (gamepad2.dpad_up) {
robot.IntakePower(-1);
            }
            if (gamepad2.dpad_down) {

            }
            if (gamepad2.dpad_left) {

            }
            if (gamepad2.dpad_right) {

            }

            if (gamepad2.left_trigger > 0) {

            }


            if (gamepad2.left_bumper) {

            }
            if (gamepad2.right_bumper) {

            }



        }
        robot.telemetryAprilTag();
        telemetry.addData("distance",robot.GetMotorEncoders());
        telemetry.update();
        // END OF LOOP


    }

}

