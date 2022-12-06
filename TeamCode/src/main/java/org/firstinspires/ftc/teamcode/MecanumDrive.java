package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import static java.lang.Math.abs;
import static java.lang.Math.sin;
import static java.lang.Math.cos;
import static java.lang.Thread.sleep;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import static java.lang.Math.pow;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.io.File;

/**
 * This is an example minimal implementation of the mecanum drivetrain
 * for demonstration purposes.  Not tested and not guaranteed to be bug free.
 *
 * @author Brandon Gongnv
 */
@TeleOp(name="Teleop Mecanum Drive Field Centric", group="Iterative Opmode")
//@Disabled
public class MecanumDrive extends OpMode {

    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize four DcMotors.
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
    private Servo gripperAsServo;
    private DcMotor motor_lift = null;
    private BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;
    String filename = "SavedHeadings.json";
    File file = AppUtil.getInstance().getSettingsFile(filename);
    String readData= ReadWriteFile.readFile(file);
    double readbotHeading=0;
    private double gripper =0;
    private int x_debounce_counter=0;
    private int b_debounce_counter=0;
    private int debouncer_threshold =1;
    private int liftpos=0;
    private int UPPER_LIMIT =4200;
    private double drive_input=0;

    private double ms1= 0.1;
    private double ms2= 0.5;
    private double ms3= 0.8;

    private double mf1= 0.1;
    private double mf2= 0.5;
    private double mf3= 0.8;

    private double power=0.3;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     COUNTS_PER_INCH =100;
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        front_left   = hardwareMap.get(DcMotor.class, "front_left");
        front_right  = hardwareMap.get(DcMotor.class, "front_right");
        back_left    = hardwareMap.get(DcMotor.class, "back_left");
        back_right   = hardwareMap.get(DcMotor.class, "back_right");
        gripperAsServo = hardwareMap.get(Servo.class, "gripperAsServo");
        motor_lift   = hardwareMap.get(DcMotor.class, "motor_lift");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // Retrieve the IMU from the hardware map
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
        //telemetry.addData("Mode", "calibrating...");
        //telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        /*while (!imu.isGyroCalibrated())
        {
            try {
                sleep(50);
                throw new InterruptedException("Exception:Checking if gyro is calibrated.");
            } catch (InterruptedException e) {
                e.printStackTrace();
            }=
            //idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());*/
        readbotHeading=Double.parseDouble(readData.substring(8));

        motor_lift.setDirection(DcMotorSimple.Direction.REVERSE);
       /* motor_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motor_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
        telemetry.update();
    }

    @Override
    public void loop() {
        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        // 3 variables to smoothen gamepad input
        if (gamepad1.left_trigger > 0.9) {
            power = 0.2;
        } else if (gamepad1.left_trigger > 0.9) {
            power = 0.3;
        }/*
        if (abs(gamepad1.left_stick_y)<0.05)
        {
            drive_input = 0;
        }
        // if 5 to 20 then slope 1
        else if (abs(gamepad1.left_stick_y)<0.2)
        {
            if (power==0.2)
                drive_input = gamepad1.left_stick_y * ms1;
            else
                drive_input = gamepad1.left_stick_y * mf1;
        }
        // if 20 to to60 then slope 2
        else if (abs(gamepad1.left_stick_y)<0.6)
        {
            if (power==0.2)
                drive_input = gamepad1.left_stick_y * ms2;
            else
                drive_input = gamepad1.left_stick_y * mf2;
        }
        //if 600 to 90 then slope 3
        else if (abs(gamepad1.left_stick_y)<0.9)
        {
            if (power==0.2)
                drive_input = gamepad1.left_stick_y * ms3;
            else
                drive_input = gamepad1.left_stick_y * mf3;
        }
        // if > 90 then flatline
        else if (abs(gamepad1.left_stick_y)>0.9)
        {
            drive_input = 1;
        }*/

        double drive = -1 * (Math.pow(gamepad1.left_stick_y, 3));
        double strafe = Math.pow(gamepad1.left_stick_x, 3);
        double twist = Math.pow(gamepad1.right_stick_x, 3);
        // Read inverse IMU heading, as the IMU heading is CW positive

        double botHeading = -getAngle() + readbotHeading;

        double new_strafe = strafe * cos(botHeading) - drive * sin(botHeading);
        double new_drive = strafe * sin(botHeading) + drive * cos(botHeading);

        /*
         * If we had a gyro and wanted to do field-oriented control, here
         * is where we would implement it.
         *
         * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
         * coordinate (strafe, drive), and we just rotate it by the gyro
         * reading minus the offset that we read in the init() method.
         * Some rough pseudocode demonstrating:
         *
         * if Field Oriented Control:
         *     get gyro heading
         *     subtract initial offset from heading
         *     convert heading to radians (if necessary)
         *     new strafe = strafe * cos(heading) - drive * sin(heading)
         *     new drive  = strafe * sin(heading) + drive * cos(heading)
         *
         * If you want more understanding on where these rotation formulas come
         * from, refer to
         * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
         */

        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
                (new_drive + new_strafe + twist),
                (new_drive - new_strafe - twist),
                (new_drive - new_strafe + twist),
                (new_drive + new_strafe - twist)
        };

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++) {
            if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        front_left.setPower(speeds[0] * 0.3);// previous 0.5
        front_right.setPower(speeds[1] * 0.3);
        back_left.setPower(speeds[2] * 0.3);
        back_right.setPower(speeds[3] * 0.3);
        telemetry.addData("o gripper position %f", gripperAsServo.getPosition());
        front_left.setPower(speeds[0] * power);// previous 0.5
        front_right.setPower(speeds[1] * power);
        back_left.setPower(speeds[2] * power);
        back_right.setPower(speeds[3] * power);

        if (gamepad1.x) {
            x_debounce_counter += 1;

            if (x_debounce_counter >= debouncer_threshold) {
                gripper = 0.05;
                //gripperAsServo.getController().pwmEnable();
                telemetry.addData("Setting gripper position %f", gripper);
                if (gripperAsServo.getPosition() != gripper) {
                    gripperAsServo.setPosition(gripper);
                } else
                    x_debounce_counter = 0;
                //gripperAsServo.getController().pwmDisable();
            }
        }
        if (gamepad1.b) {
            b_debounce_counter += 1;
            if (b_debounce_counter >= debouncer_threshold) {
                gripper = 0.15;
                //gripperAsServo.getController().pwmEnable();
                telemetry.addData("Setting gripper position %f", gripper);
                if (gripperAsServo.getPosition() != gripper) {
                    gripperAsServo.setPosition(gripper);
                } else
                    b_debounce_counter = 0;
                //gripperAsServo.getController().pwmDisable();
            }
        }

        //gripper = 1;
        //gripperAsServo.setPosition(gripper);
        //gripperAsServo.getController();

        boolean motor_lift_button_up;
        boolean motor_lift_button_down;


        motor_lift_button_up = gamepad1.y;
        motor_lift_button_down = gamepad1.a;
        telemetry.addData("liftup%f", motor_lift.getController().getMotorCurrentPosition(liftpos));
        if (1==1){
        //if (motor_lift.getController().getMotorCurrentPosition(liftpos) < UPPER_LIMIT) {
            if (motor_lift_button_up && !motor_lift_button_down) {
                telemetry.addData("inside liftup%f", liftpos);
                motor_lift.setPower(0.55);
            }
            if (motor_lift_button_down && !motor_lift_button_up) {
                motor_lift.setPower(-0.30);
                telemetry.addData("liftup", "down");
            } else if (!motor_lift_button_down && !motor_lift_button_up) {
                motor_lift.setPower(0.05);
                telemetry.addData("liftup", "off");
            }
        }
            telemetry.addData("Heading is", botHeading);
            telemetry.update();


        }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -3.14)
            deltaAngle += 6.28;
        else if (deltaAngle > 3.14)
            deltaAngle -= 6.28;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public void encoderDrive(double speed,
                             double liftInches,
                             double timeoutS) {

        int newmotor_lift_Target;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newmotor_lift_Target = motor_lift.getCurrentPosition() + (int) (liftInches * COUNTS_PER_INCH);
        motor_lift.setTargetPosition(newmotor_lift_Target);

        // Turn On RUN_TO_POSITION
        motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        motor_lift.setPower(Math.abs(speed));


        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((runtime.seconds() < timeoutS) &&
                (motor_lift.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Running to", " %7d", newmotor_lift_Target);
            telemetry.addData("Currently at", " at %7d",
                    motor_lift.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        motor_lift.setPower(0);

        // Turn off RUN_TO_POSITION
        motor_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }}
