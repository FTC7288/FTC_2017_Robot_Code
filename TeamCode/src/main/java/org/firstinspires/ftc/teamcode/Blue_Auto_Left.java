
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import java.util.Date;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name="Blue Left", group="AutoMote")

public class Blue_Auto_Left extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware7288 robot   = new Hardware7288();   // Use a Pushbot's hardware

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 1.0;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1.0 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.04;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.01;     // Larger is more responsive, but also less stable

    VuforiaLocalizer vuforia;

    boolean red = false;
    boolean blue = false;

    boolean RIGHT = false;
    boolean LEFT = false;
    boolean CENTER = false;

    double rotate_arm_pos = 0.65;

    String status = "";


    DataLogger data;
    Date day = new Date();



    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);

        data = new DataLogger(day.toString() + " Blue Auto Left");
        data.addField("Encoder Front Left");
        data.addField("Encoder Front Right");
        data.addField("Encoder Left");
        data.addField("Encoder Right");
        data.addField("Encoder Front Left Target");
        data.addField("Encoder Front Right Target");
        data.addField("Encoder Left Target");
        data.addField("Encoder Right Target");
        data.addField("Front Left Power");
        data.addField("Front Right Power");
        data.addField("Left Power");
        data.addField("Right Power");
        data.addField("Color Sensor Red");
        data.addField("Color Sensor Blue");
        data.addField("Gyro");
        data.addField("State");
        data.addField("Battery Voltage");
        data.newLine();


        robot.left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_motor_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_motor_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(500);

        robot.left_motor_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_motor_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.color.enableLed(true);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AV6QwdT/////AAAAGXc8gEwpck7Loz5Akh+cChh4PIzT57YuT6w7w+/xRzOejox7pd7yr0lPJsxYcJCH70KS29EYlA9mOP7qyndEy0KibM+rVvuggIpy1wuAqStHXoTlDhQF/bNKbC/BOEYkr7W3h+ULVQncCcTFA+QuFrqZSp0max+zyT6qNvTDx+oBThas0HbaikRbp5FhbLdJ6xEpqtBD534i/+t3WYvT9H2o2W4+i6oiFp/POIWkV0nEwSS5lsAZBYMP/21bTNtjYlDbsSMhiDArt4hgid31wU99jNkOPFNtoay5NWxtw6kpJNMiA/d8RfvcVgR4bL6p+MkvTbg7iWG0R1xWLA3x08+MvK1v4/WH5juiw8WE+INw";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary


        relicTrackables.activate();


        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        robot.gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && robot.gyro.isCalibrating())  {
            sleep(50);
            idle();
        }



        // Wait for the game to start (Display gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", robot.gyro.getIntegratedZValue());
            telemetry.update();
            status = "Robot Ready";
            idle();
        }

        status = "Robot Start";
        robot.gyro.resetZAxisIntegrator();

        robot.left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_motor_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_motor_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        if(vuMark == RelicRecoveryVuMark.CENTER){
            LEFT = false;
            RIGHT = false;
            CENTER = true;
        }else if(vuMark == RelicRecoveryVuMark.LEFT){
            LEFT = true;
            RIGHT = false;
            CENTER = false;
        }else if(vuMark == RelicRecoveryVuMark.RIGHT){
            LEFT = false;
            RIGHT = true;
            CENTER = false;
        }

        robot.left_servo.setPosition(0.25);
        robot.right_servo.setPosition(0.75);
        robot.top_left.setPosition(0.55);
        robot.top_right.setPosition(0.55);
        robot.color_arm.setPosition(0.0);
        robot.color_arm_rotate.setPosition(0.65);


        runtime.reset();

        //lift block
        while (opModeIsActive() && (runtime.seconds() < 0.50)){
            robot.lift_motor.setPower(0.45);
        }

        robot.lift_motor.setPower(0.0);

        robot.color_arm.setPosition(.45);
        sleep(500);
        robot.color_arm.setPosition(.55);
        sleep(500);

        runtime.reset();

        while(opModeIsActive() && runtime.seconds() < 2.0 && !red && !blue){
            status = "Color Sensor";
            dataRecord();
            if(robot.color.red() >= 1){
                red = true;
                blue = false;
            } else if (robot.color.blue() >=1){
                blue = true;
                red = false;
            }
            rotate_arm_pos = rotate_arm_pos + 0.01;
            robot.color_arm_rotate.setPosition(rotate_arm_pos);
            sleep(500);



        }

        if(red){
            robot.color_arm_rotate.setPosition(1.0);
        }
        if(blue){
            robot.color_arm_rotate.setPosition(0.0);
        }


        sleep(500);
        robot.color_arm.setPosition(0.0);
        sleep(500);
        robot.color_arm_rotate.setPosition(0.65);
        sleep(500);

        status = "Drive Back";
        encoderDrive(0.5,-25,-25,5);
        sleep(500);
        status = "Turn 177";
        gyroTurn(0.5, 177);
        sleep(500);
        robot.left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_motor_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_motor_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(RIGHT && !LEFT && !CENTER){
            //RIGHT
            status = "Strafe Right";
            encoderStrafe(0.5,25,5);

        }else if(!RIGHT && LEFT && !CENTER){
            //LEFT
            status = "Strafe Left";
            encoderStrafe(0.5,8,5);

        }else if(!RIGHT && !LEFT && CENTER){
            //CENTER
            status = "Strafe Center";
            encoderStrafe(0.5,16,5);
        }else{
            //CENTER
            status = "Strafe Center No Target";
            encoderStrafe(0.5,16,5);
        }


        sleep(250);
        status = "Score Block";
        encoderDrive(0.5,11,11,5);
        robot.left_servo.setPosition(0.55);
        robot.right_servo.setPosition(0.40);
        robot.top_left.setPosition(0.75);//+.0.5
        robot.top_right.setPosition(0.35); //+0.5

        status = "Back up from Block";
        encoderDrive(0.5,-8,-8,5);
        status = "Push block";
        encoderDrive(0.5,8,8,5);
        status = "Park";
        encoderDrive(0.5,-5,-5,5);







    }

















    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.left_motor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.right_motor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.left_motor_front.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.right_motor_front.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.left_motor.setTargetPosition(newLeftTarget);
            robot.right_motor.setTargetPosition(newRightTarget);
            robot.left_motor_front.setTargetPosition(newLeftFrontTarget);
            robot.right_motor_front.setTargetPosition(newRightFrontTarget);

            //Turn On RUN_TO_POSITION
            robot.left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left_motor_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_motor_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.left_motor.setPower(Math.abs(speed));
            robot.right_motor.setPower(Math.abs(speed));
            robot.left_motor_front.setPower(Math.abs(speed));
            robot.right_motor_front.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.left_motor.isBusy() && robot.right_motor.isBusy() && robot.left_motor_front.isBusy() && robot.right_motor_front.isBusy())) {

                // Display it for the driver.
                dataRecord();
            }

            // Stop all motion;
            robot.left_motor.setPower(0);
            robot.right_motor.setPower(0);
            robot.left_motor_front.setPower(0);
            robot.right_motor_front.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.left_motor_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_motor_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    public void encoderStrafe(double speed, double distance, double timeoutS) {
        int newLeftTarget_t = 0;
        int newRightTarget_t = 0;
        int newLeftFrontTarget_t = 0;
        int newRightFrontTarget_t = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget_t = robot.left_motor.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newRightTarget_t = robot.right_motor.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newLeftFrontTarget_t = robot.left_motor_front.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newRightFrontTarget_t = robot.right_motor_front.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            robot.left_motor.setTargetPosition(-newLeftTarget_t);
            robot.right_motor.setTargetPosition(newRightTarget_t);
            robot.left_motor_front.setTargetPosition(newLeftFrontTarget_t);
            robot.right_motor_front.setTargetPosition(-newRightFrontTarget_t);

            // Turn On RUN_TO_POSITION
            robot.left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left_motor_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_motor_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            if(distance > 0){
                robot.left_motor.setPower((-speed));
                robot.right_motor.setPower((speed));
                robot.left_motor_front.setPower((speed));
                robot.right_motor_front.setPower((-speed));
            }else if(distance < 0){
                robot.left_motor.setPower((speed));
                robot.right_motor.setPower((-speed));
                robot.left_motor_front.setPower((-speed));
                robot.right_motor_front.setPower((speed));
            }
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.left_motor.isBusy() && robot.right_motor.isBusy() && robot.left_motor_front.isBusy() && robot.right_motor_front.isBusy())) {

                // Display it for the driver.
                dataRecord();
            }

            // Stop all motion;
            robot.left_motor.setPower(0);
            robot.right_motor.setPower(0);
            robot.left_motor_front.setPower(0);
            robot.right_motor_front.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.left_motor_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_motor_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            dataRecord();
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.left_motor_front.setPower(0);
        robot.right_motor_front.setPower(0);
        robot.left_motor.setPower(0);
        robot.right_motor.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = -speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.left_motor_front.setPower(leftSpeed);
        robot.right_motor_front.setPower(rightSpeed);
        robot.left_motor.setPower(leftSpeed);
        robot.right_motor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        telemetry.addData("angle",robot.gyro.getIntegratedZValue());

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void dataRecord(){
        data.addField((float)robot.left_motor_front.getCurrentPosition());
        data.addField((float)robot.right_motor_front.getCurrentPosition());
        data.addField((float)robot.left_motor.getCurrentPosition());
        data.addField((float)robot.right_motor.getCurrentPosition());
        data.addField((float)robot.left_motor_front.getTargetPosition());
        data.addField((float)robot.right_motor_front.getTargetPosition());
        data.addField((float)robot.left_motor.getTargetPosition());
        data.addField((float)robot.right_motor.getTargetPosition());
        data.addField((float)robot.left_motor_front.getPower());
        data.addField((float)robot.right_motor_front.getPower());
        data.addField((float)robot.left_motor.getPower());
        data.addField((float)robot.right_motor.getPower());
        data.addField((float)robot.color.red());
        data.addField((float)robot.color.blue());
        data.addField((float)robot.gyro.getIntegratedZValue());
        data.addField((String)status);
        data.addField((float)robot.voltage.getVoltage());
        data.newLine();
    }

}
