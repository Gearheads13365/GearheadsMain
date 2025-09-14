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


            if (gamepad1.dpad_up) {

            }
            if (gamepad1.dpad_down) {

            }
            if (gamepad1.dpad_left) {

            }
            if (gamepad1.dpad_right) {

            }



            if (gamepad1.right_trigger > 0.5)
            {
                precisePower = 3;
            }
            else
            {
                precisePower = 1;
            }
            //Set mecanum power
            robot.setMecanumPower (
                    ((-gamepad1.left_stick_y + gamepad1.left_stick_x) + (gamepad1.right_stick_x))/precisePower,
                    ((-gamepad1.left_stick_y - gamepad1.left_stick_x) + (gamepad1.right_stick_x))/precisePower,
                   ((-gamepad1.left_stick_y - gamepad1.left_stick_x) - (gamepad1.right_stick_x))/precisePower,
                   ((-gamepad1.left_stick_y + gamepad1.left_stick_x) - (gamepad1.right_stick_x))/precisePower );



        }
    }
}
