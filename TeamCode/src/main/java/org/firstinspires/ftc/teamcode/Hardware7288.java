package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware7288
{
    Servo left_servo;
    Servo right_servo;
    Servo color_arm;
    Servo color_arm_rotate;
    Servo top_left;
    Servo top_right;

    DcMotor left_motor;
    DcMotor left_motor_front;
    DcMotor right_motor;
    DcMotor right_motor_front;
    DcMotor lift_motor;

    ColorSensor color;

    ModernRoboticsI2cGyro gyro;

    VoltageSensor voltage;



    HardwareMap hwMap;


    /* Constructor */
    public Hardware7288() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        right_servo = hwMap.get(Servo.class, "right_servo");
        left_servo = hwMap.get(Servo.class, "left_servo");
        color_arm = hwMap.get(Servo.class, "color_arm");
        color_arm_rotate = hwMap.get(Servo.class, "color_arm_rotate");
        top_right = hwMap.get(Servo.class, "top_right");
        top_left = hwMap.get(Servo.class,"top_left");

        right_motor = hwMap.get(DcMotor.class, "right_motor");
        left_motor = hwMap.get(DcMotor.class, "left_motor");
        right_motor_front = hwMap.get(DcMotor.class, "right_motor_front");
        left_motor_front = hwMap.get(DcMotor.class, "left_motor_front");
        lift_motor = hwMap.get(DcMotor.class, "lift");

        color = hwMap.get(ColorSensor.class,"color");
        gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");

        voltage = hwMap.get(VoltageSensor.class, "Motor Controller 1");

        left_motor.setDirection(DcMotor.Direction.REVERSE);
        left_motor_front.setDirection(DcMotor.Direction.REVERSE);
        right_motor.setDirection(DcMotor.Direction.FORWARD);
        right_motor_front.setDirection(DcMotor.Direction.FORWARD);




        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_motor_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_motor_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}
