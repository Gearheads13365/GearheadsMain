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
        robot.resetEncoders();

        robot.setClawPosition(robot.close);

        robot.webcam.setPipeline(robot.pipelineLTMP);

        double liftPower = -1;
        int precisePower = 1;
        int liftState = 0;

        waitForStart();

        while (opModeIsActive()) {


            if (gamepad1.dpad_up) {
                robot.webcam.setPipeline(robot.pipelineLTRP);
                robot.lastPressed = 1;
            }
            if (gamepad1.dpad_down) {
                robot.webcam.setPipeline(robot.pipelineRTMP);
                robot.lastPressed = 2;
            }
            if (gamepad1.dpad_left) {
                robot.webcam.setPipeline(robot.pipelineLTMP);
                robot.lastPressed = 3;
            }
            if (gamepad1.dpad_right) {
                robot.webcam.setPipeline(robot.pipelineRTRP);
                robot.lastPressed = 4;
            }


            if (robot.lastPressed == 1)
            {
                robot.AvgVal_RED = robot.pipelineLTRP.getAnalysisRED();
                robot.AvgVal_BLUE = robot.pipelineLTRP.getAnalysisBLUE();
            }
            if (robot.lastPressed == 2)
            {
                robot.AvgVal_RED = robot.pipelineRTMP.getAnalysisRED();
                robot.AvgVal_BLUE = robot.pipelineRTMP.getAnalysisBLUE();
            }
            if (robot.lastPressed == 3)
            {
                robot.AvgVal_RED = robot.pipelineLTMP.getAnalysisRED();
                robot.AvgVal_BLUE = robot.pipelineLTMP.getAnalysisBLUE();
            }
            if (robot.lastPressed == 4)
            {
                robot.AvgVal_RED = robot.pipelineRTRP.getAnalysisRED();
                robot.AvgVal_BLUE = robot.pipelineRTRP.getAnalysisBLUE();
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

            double heading = robot.getHeading();

            x_position = robot.getAprilTagPosition(2);
            telemetry.addData("x_position", x_position);
            telemetry.addData("Heading",heading);
            telemetry.addData("Encoders", robot.GetMotorEncoders() );
            //telemetry.addData("Serve1Position", robot.w1.getPosition());
            //telemetry.addData("Serve2Position", robot.w2.getPosition());


            telemetry.addData("LB_Encoder", robot.Get_LB_Encoder());
            telemetry.addData("LF_Encoder", robot.Get_LF_Encoder());
            telemetry.addData("RB_Encoder", robot.Get_RB_Encoder());
            telemetry.addData("RF_Encoder", robot.Get_RF_Encoder());



            robot.setArmPower(gamepad2.left_stick_y);

            liftPower = -1/precisePower;

            if (gamepad1.right_bumper)
            {
                liftState = 1;
                robot.setLiftPower(1);
            }
            else if (gamepad1.left_bumper)
            {
                liftState = 0;
                robot.setLiftPower(liftPower);
            }
            else if (liftState == 0)
            {
                robot.setLiftPower(0);
            }

            if (liftState == 1 && robot.getLiftEncoder() > 3469)
            {
                    robot.setLiftPower(0);
                    liftState = 0;
            }
            telemetry.addData("Lift_Encoder", robot.Get_Lift_Encoder());

            if (gamepad2.x)
            {
                robot.setClawPosition(robot.close);
            }
            if (gamepad2.b)
            {
                robot.setClawPosition(robot.open);
            }

            if (gamepad2.right_trigger > 0.15 || gamepad2.left_trigger > 0.15)
            {
                robot.setWristPower(gamepad2.left_trigger - gamepad2.right_trigger);
            }
            else
            {
                robot.setWristPower(0);
            }



            if (gamepad1.a)
            {
                robot.resetEncoders();
            }
            if (gamepad1.b)
            {
                robot.imu.resetYaw();
            }

            if (gamepad1.y)
            {
                robot.setPAPower(-1);
            }
            else
            {
                robot.setPAPower(0);
            }

            telemetry.addData("armPower", robot.GetArmPower());
            telemetry.addData("armEncoder", robot.Get_Arm_Encoder());
            telemetry.addData("servoReportedPos", robot.getClawPosition());
            telemetry.addData("AvgVal_RED", robot.AvgVal_RED);
            telemetry.addData("AvgVal_BLUE", robot.AvgVal_BLUE);
            telemetry.update();
        }
    }
}
