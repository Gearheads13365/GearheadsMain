package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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


            // GAMEPAD 1 BUTTONS //
            // Open
            if (gamepad1.a) {
robot.resetEncoders();
            }
            // Hold
            if (gamepad1.b) {

            }
            // Launch
            if (gamepad1.x) {

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

            if (gamepad2.a && buttonTimer.milliseconds() > 200) {
                if (robot.GetIntakePower() > .5) {
                    robot.intakePower(0);
                } else {
                    robot.intakePower(1);
                }
                buttonTimer.reset();
            }

            if (gamepad2.b && buttonTimer.milliseconds() > 200) {
                if (robot.GetShooterPower() > .5) {
                    robot.shooterPower(0);
                } else {
                    robot.shooterPower(.85);
                }
                buttonTimer.reset();
            }
            if (gamepad2.x && buttonTimer.milliseconds() > 200) {
                if (BackOn) {
                    robot.backStage(0);
                    BackOn = false;
                } else {
                    robot.backStage(1);
                    BackOn = true;
                }
                buttonTimer.reset();
            }
            if (gamepad2.y) {
                robot.frontStage(1);
            } else {
                robot.frontStage(0);
            }


            if (gamepad2.dpad_up) {
                robot.intakePower(-1);
            }
            if (gamepad2.dpad_down) {

            }
            if (gamepad2.dpad_left) {

            }
            if (gamepad2.dpad_right) {

            }

            if (gamepad2.left_trigger > 0.2) {
            } else if (gamepad2.left_trigger == 0) {

            }


            if (gamepad2.left_bumper) {


            }

        if (gamepad2.right_bumper) {

        }

        telemetry.addData("distance", robot.GetMotorEncoders());
        telemetry.update();
    }

        }

        // END OF LOOP


    }



