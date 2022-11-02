package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ReadWriteFile;

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
            }
            //idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());*/
        readbotHeading=Double.parseDouble(readData.substring(8));

        motor_lift.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.update();
    }

    @Override
    public void loop() {
        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double drive  = -1*(Math.pow(gamepad1.left_stick_y,3));
        double strafe = Math.pow(gamepad1.left_stick_x,3);
        double twist  = Math.pow(gamepad1.right_stick_x,3) ;
        // Read inverse IMU heading, as the IMU heading is CW positive

        double botHeading = -getAngle() +readbotHeading;

       double new_strafe = strafe * cos(botHeading) - drive * sin(botHeading);
       double new_drive  = strafe * sin(botHeading) + drive * cos(botHeading);

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
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        front_left.setPower(speeds[0]*0.5);
        front_right.setPower(speeds[1]*0.5);
        back_left.setPower(speeds[2]*0.5);
        back_right.setPower(speeds[3]*0.5);
        double gripper =0;
        if (gamepad1.x)
            gripper =1;
        else
            gripper =0;
        gripperAsServo.setPosition(gripper);
        boolean motor_lift_button_up;
        boolean motor_lift_button_down;


        motor_lift_button_up = gamepad1.y;
        motor_lift_button_down = gamepad1.a;
        if (motor_lift_button_up && !motor_lift_button_down) {
            motor_lift.setPower(0.65);
            telemetry.addData("liftup","up");
        }

        if (motor_lift_button_down && !motor_lift_button_up) {
            motor_lift.setPower(-0.15);
            telemetry.addData("liftup","down");
        }

        if (!motor_lift_button_down && !motor_lift_button_up) {
            motor_lift.setPower(0);
            telemetry.addData("liftup", "off");
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
}