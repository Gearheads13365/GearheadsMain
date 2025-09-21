package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Teleop", group="Linear Opmode")

public class Teleop extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        double x_position = 0;

        GearHeadRobot robot = new GearHeadRobot(this);
        robot.init();
        robot.imu.resetYaw();


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
            if (gamepad1.a) {

            }
            if (gamepad1.b) {

            }
            if (gamepad1.x) {

            }
            if (gamepad1.y) {

            }

            if (gamepad1.dpad_up) {
                robot.resetEncoders();
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

            }
            if (gamepad2.b) {

            }
            if (gamepad2.x) {

            }
            if (gamepad2.y) {

            }

            if (gamepad2.dpad_up) {

            }
            if (gamepad2.dpad_down) {

            }
            if (gamepad2.dpad_left) {

            }
            if (gamepad2.dpad_right) {

            }

            if (gamepad2.left_trigger > 0) {

            }
            if (gamepad2.right_trigger > 0.5) {
                precisePower = 3;
            } else {
                precisePower = 1;
            }

            if (gamepad2.left_bumper) {

            }
            if (gamepad2.right_bumper) {

            }

        }
    }
}

