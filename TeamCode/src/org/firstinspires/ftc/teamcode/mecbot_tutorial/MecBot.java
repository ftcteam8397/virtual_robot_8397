package org.firstinspires.ftc.teamcode.mecbot_tutorial;

import android.graphics.Color;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.AngleUtils;

/**
 * The MechBot class represents a mecanum-wheeled robot with a BNO055IMU and a color sensor. It has methods
 * that provide the basic functionality of such a robot
 */
public class MecBot {

    /*
     * Constants
     */
    public static final float WHEEL_CIRCUMFERENCE = 4 * (float)Math.PI;
    public static final float TICKS_PER_ROTATION = 560;
    public static final float MAX_TICKS_PER_SECOND = 2500;
    public static final float WL_AVG = 15;

    /*
     * Drive Motors
     */
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    /*
     * The color sensor
     */

    /*
     * The BNO055IMU (gyro)
     */
    BNO055IMU imu;

    /*
     * The heading offset (this gives us flexibility in specifying the world coordinate system).
     */
    private float headingOffsetRadians = 0;

    /*
     * The current pose of the robot
     */
    private Pose pose = new Pose(0, 0, 0);

    /*
     * The most recent previous readings of the drive motor ticks
     */
    int ticksBL, ticksFL, ticksFR, ticksBR;

    /**
     * Obtain instances of the robot hardware using the hardware map, and initialize the BNO055IMU
     * @param hwMap
     */
    public void init(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotor.class, "front_left_motor");
        frontRight = hwMap.get(DcMotor.class, "front_right_motor");
        backLeft = hwMap.get(DcMotor.class, "back_left_motor");
        backRight = hwMap.get(DcMotor.class, "back_right_motor");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";

        imu.initialize(parameters);
    }

    public Pose getPose() {
        return pose;
    }

    /**
     * Obtain the current robot heading using the IMU (note use of the heading offset)
     * @return heading in radians
     */
    public float getHeadingRadians(){
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return headingOffsetRadians + orientation.firstAngle;
    }

    /**
     * Set the robot's heading to the specified value, in degrees. Store this heading in the Pose object, adjust the
     * headingOffsetRadians value as necessary, and update the tick readings of the drive motors.
     * @param headingDegrees
     */
    public void setHeadingDegrees(float headingDegrees){
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        headingOffsetRadians = (float)Math.toRadians(headingDegrees) - orientation.firstAngle;
        pose.theta = (float)Math.toRadians(headingDegrees);
        updateTicks();
    }

    public void setPose(float x, float y, float headingDegrees){
        pose.x = x;
        pose.y = y;
        setHeadingDegrees(headingDegrees);
    }

    public void setPose(float x, float y){
        pose.x = x;
        pose.y = y;
        updateTicks();
    }

    /**
     * Update the tick readings of the drive motors.
     */
    public void updateTicks(){
        ticksBL = backLeft.getCurrentPosition();
        ticksFL = frontLeft.getCurrentPosition();
        ticksFR = frontRight.getCurrentPosition();
        ticksBR = backRight.getCurrentPosition();
    }

    /**
     * Get the current Hue, Saturation, and Value from the Color Sensor
     * @return
     */

    /**
     * Set the drive powers for the forward, rotation, and strafe directions. Note that these powers are in
     * the range of -1 to +1, and represent the FRACTION of maximal possible drive speed in each direction.
     * @param forward   forward (robot-Y-axis) power
     * @param rotate    rotation power (counter-clockwise if positive)
     * @param strafe    strafe power (robot-X-axis)
     */
    public void setDrivePower(float forward, float rotate, float strafe) {
        float frontLeftPower = forward - rotate + strafe;
        float frontRightPower = forward + rotate - strafe;
        float backLeftPower = forward - rotate - strafe;
        float backRightPower = forward + rotate + strafe;

        float largest = 1;
        largest = Math.max(largest, Math.abs(frontLeftPower));
        largest = Math.max(largest, Math.abs(frontRightPower));
        largest = Math.max(largest, Math.abs(backLeftPower));
        largest = Math.max(largest, Math.abs(backRightPower));

        frontLeft.setPower(frontLeftPower / largest);
        frontRight.setPower(frontRightPower / largest);
        backLeft.setPower(backLeftPower / largest);
        backRight.setPower(backRightPower / largest);
    }

    /**
     * Set robot drive speeds in INCHES PER SECOND (for vx and vy) or RADIANS PER SECOND (for va)
     * @param vx    Drive speed along robot-X-axis (rightward if positive)
     * @param vy    Drive speed along robot-Y-axis (forward if positive)
     * @param va    Rotation speed in radians/sec (counter-clockwise if positive)
     */
    public void setDriveSpeed(float vx, float vy, float va){
        float px = (vx / WHEEL_CIRCUMFERENCE) * TICKS_PER_ROTATION / MAX_TICKS_PER_SECOND;
        float py = (vy / WHEEL_CIRCUMFERENCE) * TICKS_PER_ROTATION / MAX_TICKS_PER_SECOND;
        float pa = (WL_AVG * va / WHEEL_CIRCUMFERENCE) * TICKS_PER_ROTATION / MAX_TICKS_PER_SECOND;
        setDrivePower(py, pa, px);
    }

    /**
     * Update robot pose (position on the field, and heading) for a single iteration of a control loop. This is a
     * gyro-assisted odometry algorithm.
     * @return
     */
    public Pose updateOdometry(){
        /*
         * Obtain new heading using the IMU. Then calculate the (small) interval change in the heading, as well as
         * the average heading during the (small) interval of time since the previous iteration. Note the use of the
         * AngleUtils.normalizeRadians method to keep angles in the -PI to +PI range.
         */
        float heading = getHeadingRadians();
        float headingChange = (float)AngleUtils.normalizeRadians(heading - pose.theta);
        float avgHeading = (float)AngleUtils.normalizeRadians(pose.theta + 0.5 * headingChange);

        /*
         * Get the current drive motor ticks
         */
        int currBL = backLeft.getCurrentPosition();
        int currFL = frontLeft.getCurrentPosition();
        int currFR = frontRight.getCurrentPosition();
        int currBR = backRight.getCurrentPosition();

        /*
         * Calculate the NEW ticks that have occurred since the previous update (new = current - previous)
         */
        int newBL = currBL - ticksBL;
        int newFL = currFL - ticksFL;
        int newFR = currFR - ticksFR;
        int newBR = currBR - ticksBR;

        /*
         * Update the fields that store the tick values, so they will be ready for the next iteration.
         */
        ticksBL = currBL;
        ticksFL = currFL;
        ticksFR = currFR;
        ticksBR = currBR;

        /*
         * Determine the distance that the surface of each wheel has rolled.
         *
         * NOTE: Analysis of DIMENSIONS can be helpful in getting this right. We want to get from "new ticks" to
         * "inches of travel".
         *
         * newBL has dimensions of ticks, and TICKS_PER_ROTATION has dimensions of ticks/rotation.
         * So (newBL / TICKS_PER_ROTATION) has dimensions of Rotations.
         * WHEEL_CIRCUMFERENCE has dimensions of inches/rotation.
         * So (newBL / TICKS_PER_ROTATION) * WHEEL_CIRCUMFERENCE has dimensions of  inches.
         *
         */
        float sBL = (newBL / TICKS_PER_ROTATION) * WHEEL_CIRCUMFERENCE;
        float sFL = (newFL / TICKS_PER_ROTATION) * WHEEL_CIRCUMFERENCE;
        float sFR = (newFR / TICKS_PER_ROTATION) * WHEEL_CIRCUMFERENCE;
        float sBR = (newBR / TICKS_PER_ROTATION) * WHEEL_CIRCUMFERENCE;

        /*
         * Determine small increment of robot motion in ROBOT COORDINATE SYSTEM
         */
        float dXR = 0.25f * (-sBL + sFL - sFR + sBR);
        float dYR = 0.25f * (sBL + sFL + sFR + sBR);

        /*
         * Convert this small increment of robot motion into WORLD COORDINATES
         */
        float dX = dXR * (float)Math.sin(avgHeading) + dYR * (float)Math.cos(avgHeading);
        float dY = -dXR * (float)Math.cos(avgHeading) + dYR * (float)Math.sin(avgHeading);

        /*
         * Update the Pose object with the new values for X, Y, and Heading
         */
        pose.x = pose.x + dX;
        pose.y = pose.y + dY;
        pose.theta = heading;

        /*
         * Return the updated Pose object
         */
        return pose;
    }
}
