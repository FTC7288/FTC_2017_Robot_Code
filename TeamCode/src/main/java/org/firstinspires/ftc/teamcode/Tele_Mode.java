package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="TeleMode", group="Linear Opmode")

public class Tele_Mode extends LinearOpMode {
    Hardware7288   robot           = new Hardware7288();

    @Override
    public void runOpMode() {


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);

        robot.color.enableLed(false);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.color_arm.setPosition(0);
            robot.color_arm_rotate.setPosition(0.65);

            double leftPower;
            double rightPower;


            leftPower  = -gamepad1.left_stick_y ;
            rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            robot.left_motor.setPower(leftPower);
            robot.left_motor_front.setPower(leftPower);
            robot.right_motor.setPower(rightPower);
            robot.right_motor_front.setPower(rightPower);


            if(gamepad1.right_bumper ){
                robot.left_servo.setPosition(0.55);
                robot.right_servo.setPosition(0.40);
                robot.top_left.setPosition(0.75);//+.0.5
                robot.top_right.setPosition(0.35); //+0.5


            }else if(!gamepad1.right_bumper){
                robot.left_servo.setPosition(0.25);
                robot.right_servo.setPosition(0.75);
                robot.top_left.setPosition(0.57);
                robot.top_right.setPosition(0.55);



            }

            if(gamepad1.right_trigger > 0.2){
                robot.lift_motor.setPower(1);
            }else if(gamepad1.left_trigger > 0.2){
                robot.lift_motor.setPower(-1);
            }else{
                robot.lift_motor.setPower(0);
            }

            telemetry.addData("left", gamepad1.left_trigger);
            telemetry.addData("right", gamepad1.right_trigger);
            telemetry.update();


        }
    }
}
