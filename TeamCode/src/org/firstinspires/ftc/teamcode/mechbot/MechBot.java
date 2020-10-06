package org.firstinspires.ftc.teamcode.mechbot;

import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.logging.BetaLog;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
//import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
//import org.firstinspires.ftc.teamcode.i2c.BNO055EnhancedImpl;

import java.util.List;


/**
 * A <code>Mechbot</code> object represents a robot with four mechanum wheels, and an IMU. It is
 * capable of tracking its own pose using gyro-assisted odometry.
 */
public class MechBot
{

    protected final boolean LOG_ODOMETRY = true;       //Enable or disable logging of odometry
    protected final String ODOMETRY_TAG = "ODOMETRY";   //Tag for odometry logging

    private double wheelCircumference = 31.5;            //Wheel circumference in cm
    private MotorType motorType = MotorType.NeverestOrbital20;  //Type of drive motor
    private double gearRatio = 1.0;    //Gearing beyond the built-in gearbox of the motor type
    private double ticksPerCm = motorType.ticksPerRotation * gearRatio / wheelCircumference;    //Wheel ticks per cm
    private final double MAX_TICKS_PER_SEC = 2500;      //Maximum possible ticks per cm
    private double length = 33.0;                       //Drive base length
    private double width = 33.0;                        //Drive base width
    private final double ROLLER_ANGLE = 45 * Math.PI / 180;    //Roller angle and radians
    private float width_length_avg = (float)(width + length) / 2.0f;                    //Needed for odometry and setting drive speeds

    private float initHeadingRadians = 0;       //Initial Robot Heading in radians

    private MatrixF wheelRobotTransform = null;             //Wheel-to-robot transform (for odometry)

    private VectorF last_Wheel_Ticks = new VectorF(0,0,0,0); //Vector to keep track of most recent wheel ticks

    public DcMotor one,two,three,four;  //DC Motors (one=back-left, two=front-left, three=front-right, four=back-right)

    private HardwareMap hardwareMap = null;  //Hardware map to be obtained from op mode during init

    public BNO055IMU imu;          //IMU, our custom class that allows setting of axes and sign maps


    /**
     * Enum <code>MotorType</code> represents types of motors that can be used to power the drive.
     */
    public enum MotorType {
        Neverest40(1120, false),
        Neverest20(560, false),
        NeverestOrbital20(560, true),
        Neverest60(1680, false);

        MotorType(double ticksPerRotation, boolean reversed){
            this.ticksPerRotation = ticksPerRotation;
            this.reversed = reversed;
        }

        private double ticksPerRotation;
        private boolean reversed;
    }


    /**No-args constructor -- use default values for motor type (Neverest Orbital 20), wheel
     * circumference (31.5cm), length(33cm), width(33cm), imu setup (XYZ, PPP).
     */
    public MechBot(){}


    /**
     * Constructor --custom values for motor type, wheel circumference, length, width
     *default values for imu setup.
     *@param motorType The type of drive motor.
     * @param wheelCirc Wheel circumference in cm.
     * @param length Wheel base length in cm.
     * @param width Wheel base width in cm.
     */
    public MechBot(MotorType motorType, double wheelCirc, double length, double width){
        this.motorType = motorType;
        wheelCircumference = wheelCirc;
        ticksPerCm = motorType.ticksPerRotation * Math.abs(gearRatio) / wheelCircumference;
        this.length = length;
        this.width = width;
        width_length_avg = (float)(width + length) / 2.0f;
    }


    /**
     * Constructor --custom values for motor type, wheel circumference, length, width
     *default values for imu setup.
     *@param motorType The type of drive motor.
     * @param wheelCirc Wheel circumference in cm.
     * @param length Wheel base length in cm.
     * @param width Wheel base width in cm.
     * @param gearRatio Gear Ratio for gearing after built-in gearbox.
     */
    public MechBot(MotorType motorType, double wheelCirc, double length, double width, double gearRatio){
        this.motorType = motorType;
        wheelCircumference = wheelCirc;
        this.gearRatio = gearRatio;
        ticksPerCm = motorType.ticksPerRotation * Math.abs(gearRatio) / wheelCircumference;
        this.length = length;
        this.width = width;
        width_length_avg = (float)(width + length) / 2.0f;
    }


    /**
     * Constructor --custom values for everything except gear ratio.
     * @param motorType The type of drive motor.
     * @param wheelCirc Wheel circumference in cm.
     * @param length Wheel base length in cm.
     * @param width Wheel base width in cm.
     * @param axMap Axes Map for IMU.
     * @param axSign Axes Sign for IMU.
     */
//    public MechBot(MotorType motorType, double wheelCirc, double length, double width,
//                   BNO055Enhanced.AxesMap axMap, BNO055Enhanced.AxesSign axSign){
//        this(motorType, wheelCirc, length, width);
//        axesMap = axMap;
//        axesSign = axSign;
//    }


    /**
     * Constructor --custom values for everything.
     * @param motorType The type of drive motor.
     * @param wheelCirc Wheel circumference in cm.
     * @param length Wheel base length in cm.
     * @param width Wheel base width in cm.
     * @param axMap Axes Map for IMU.
     * @param axSign Axes Sign for IMU.
     * @param gearRatio Gear ratio for gearing after the built-in gearbox.
     */
//    public MechBot(MotorType motorType, double wheelCirc, double length, double width,
//                   BNO055Enhanced.AxesMap axMap, BNO055Enhanced.AxesSign axSign, double gearRatio){
//        this(motorType, wheelCirc, length, width, gearRatio);
//        axesMap = axMap;
//        axesSign = axSign;
//    }


    /**
     * Initialize default Hardware interfaces.
     * @param ahwMap Hardware map to be passed in from op mode.
     * @param initHeadingDegrees Initial heading of robot in degrees (-180 to 180)
     * @param initializeIMU If true, this method will initialize the IMU.
     */
    public void init(HardwareMap ahwMap, float initHeadingDegrees, boolean initializeIMU) {

        initHeadingRadians = initHeadingDegrees * (float)Math.PI / 180.0f;

        hardwareMap = ahwMap;

        one = hardwareMap.get(DcMotor.class,"back_left_motor");    //Back left
        two = hardwareMap.get(DcMotor.class,"front_left_motor");    //Front left
        three = hardwareMap.get(DcMotor.class,"front_right_motor");  //Front right
        four = hardwareMap.get(DcMotor.class,"back_right_motor");   //Back right

        if ((motorType.reversed && gearRatio > 0) || (!motorType.reversed && gearRatio < 0) ){
            three.setDirection(DcMotor.Direction.REVERSE);
            four.setDirection(DcMotor.Direction.REVERSE);
        }
        else{
            one.setDirection(DcMotorSimple.Direction.REVERSE);
            two.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        float cotan = 1.0f/(float) Math.tan(ROLLER_ANGLE);
        //Robot-to-Wheel Transform, used only to obtain the Wheel-to-Robot Transform
        GeneralMatrixF robotWheelTransform = new GeneralMatrixF(4,4,
                new float[] {-cotan, 1, -width_length_avg,  1
                        , cotan, 1, -width_length_avg, -1
                        ,-cotan, 1, width_length_avg, -1
                        , cotan, 1, width_length_avg,  1} );

        wheelRobotTransform = robotWheelTransform.inverted();

        //Instantiate the imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Initialize the imu, if so requested
        if (initializeIMU) initIMU();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule module : allHubs) {
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
    }


    /**
     * Initializes the IMU. Only needed if the <code>init</code> method is called with
     * initializeIMU false.
     */
    public void initIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BN055Cali.json"; // see the calibration sample opmode
        parameters.loggingTag = "IMU";
//        parameters.axesMap = axesMap;
//        parameters.axesSign = axesSign;
        imu.initialize(parameters);
    }


    /**
     * Set robot drive powers; scales to ensure that no motor is outside of the -1 to +1 range.
     * @param px Power for robot-X direction
     * @param py Power for robot-Y direction
     * @param pa Power for rotation.
     * @return The scaling factor that was used to keep motors in the -1 to +1 range.
     */
    public double setDrivePower(double px, double py, double pa){
        double w1 = -px + py - pa;
        double w2 =  px + py - pa;
        double w3 = -px + py + pa;
        double w4 =  px + py + pa;
        double max = Math.max(Math.abs(w1), Math.abs(w2));
        max = Math.max(max, Math.abs(w3));
        max = Math.max(max, Math.abs(w4));
        max = Math.max(max, 1.0);
        w1 = w1/max;
        w2 = w2/max;
        w3 = w3/max;
        w4 = w4/max;
        one.setPower(w1);
        two.setPower(w2);
        three.setPower(w3);
        four.setPower(w4);
        return 1.0/max;
    }

    /**
     *Sets DC Motor mode
     * @param mode dc motor run mode
     */

    public void setDriveMode(DcMotor.RunMode mode){
        one.setMode(mode);
        two.setMode(mode);
        three.setMode(mode);
        four.setMode(mode);
    }

    /**
     *Sets common ZeroPowerBehavior for all four motors.
     * @param beh Common ZeroPowerBehavior for all four motors.
     */

//    public void setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior beh){
//        one.setZeroPowerBehavior(beh);
//        two.setZeroPowerBehavior(beh);
//        three.setZeroPowerBehavior(beh);
//        four.setZeroPowerBehavior(beh);
//}

    /**
     * Set robot velocity (including x,y,angular components).
     * @param vx x velocity, cm/sec
     * @param vy y velocity, cm/sec
     * @param va angular velocity, radians/sec
     * @return Scaling factor that was used to keep all four motors in -1 to +1 range.
     */
    public double setDriveSpeed(double vx, double vy,double va){
        return setDrivePower(vx * ticksPerCm / MAX_TICKS_PER_SEC,
                             vy * ticksPerCm / MAX_TICKS_PER_SEC,
                            va * ticksPerCm * width_length_avg /MAX_TICKS_PER_SEC);
    }

    /**
     * Update last_Wheel_Ticks to the current motor encoder counts.
     */
    public void updateOdometry(){
        last_Wheel_Ticks.put(0,one.getCurrentPosition());
        last_Wheel_Ticks.put(1,two.getCurrentPosition());
        last_Wheel_Ticks.put(2,three.getCurrentPosition());
        last_Wheel_Ticks.put(3,four.getCurrentPosition());
    }

    /**
     * Determine new robot pose based on previous pose and intervening wheel ticks; then update last_Wheel_Ticks.
     * @param lastPos Previous XYTheta pose (cm, cm, radians).
     * @return New XYTheta pose (cm, cm, radians).
     */
    public float[] updateOdometry(float[] lastPos){
        float x = lastPos[0];
        float y = lastPos[1];
        float theta = lastPos[2];
        VectorF curTicks = new VectorF(one.getCurrentPosition(),two.getCurrentPosition(),three.getCurrentPosition(),four.getCurrentPosition());
        VectorF newTicks = curTicks.subtracted(last_Wheel_Ticks);
        VectorF deltaWheelCM = newTicks.multiplied((float)(1.0/ ticksPerCm));

        VectorF deltaRobotPos = wheelRobotTransform.multiplied(deltaWheelCM);
        final float sin = (float)(Math.sin(theta+deltaRobotPos.get(2)/2.0));
        final float cosin = (float)(Math.cos(theta+deltaRobotPos.get(2)/2.0));

        float newX = x+deltaRobotPos.get(0)*sin +deltaRobotPos.get(1)*cosin;
        float newY = y+deltaRobotPos.get(1)*sin - deltaRobotPos.get(0)*cosin;
        float newTheta = theta+deltaRobotPos.get(2);
        last_Wheel_Ticks = curTicks;
        return new float[]{newX,newY,newTheta};
    }


     /**
     * Determine new robot pose based on previous pose, new gyro heading, and intervening wheel ticks; then update last_Wheel_Ticks.
     * @param lastPos Previous XYTheta pose (cm, cm, radians).
     * @param newOdomHeading New heading, in radians (typically would be obtained using gyro.
     * @return New XYTheta pose (cm, cm, radians).
     */
    public float[] updateOdometry(float[] lastPos, float newOdomHeading){
            if (LOG_ODOMETRY) {
                BetaLog.dd(ODOMETRY_TAG, "Entering updateOdometry");
                BetaLog.dd(ODOMETRY_TAG, "lastPos = %.2f %.2f %.2f", lastPos[0], lastPos[1], lastPos[2]);
                BetaLog.dd(ODOMETRY_TAG, "newOdomHeading = %.2f", newOdomHeading);
                BetaLog.dd(ODOMETRY_TAG, "lastWheelTicks = %.0f %.0f %.0f %.0f", last_Wheel_Ticks.get(0),
                        last_Wheel_Ticks.get(1), last_Wheel_Ticks.get(2), last_Wheel_Ticks.get(3));
            }
            float x = lastPos[0];
            float y = lastPos[1];
            float theta = lastPos[2];
            VectorF curTicks = new VectorF(one.getCurrentPosition(), two.getCurrentPosition(), three.getCurrentPosition(), four.getCurrentPosition());
            VectorF newTicks = curTicks.subtracted(last_Wheel_Ticks);
            VectorF deltaWheelCM = newTicks.multiplied((float) (1.0 / ticksPerCm));
            VectorF deltaRobotPos = wheelRobotTransform.multiplied(deltaWheelCM);
            if (LOG_ODOMETRY) {
                BetaLog.dd(ODOMETRY_TAG, "curTicks = %.0f %.0f %.0f %.0f", curTicks.get(0), curTicks.get(1), curTicks.get(2), curTicks.get(3));
                BetaLog.dd(ODOMETRY_TAG, "newTicks = %.0f %.0f %.0f %.0f", newTicks.get(0), newTicks.get(1), newTicks.get(2), newTicks.get(3));
                BetaLog.dd(ODOMETRY_TAG, "deltaWheelCM = %.2f %.2f %.2f %.2f", deltaWheelCM.get(0), deltaWheelCM.get(1),
                        deltaWheelCM.get(2), deltaWheelCM.get(3));
                BetaLog.dd(ODOMETRY_TAG, "deltaRobotPos = %.2f %.2f %.2f %.2f", deltaRobotPos.get(0), deltaRobotPos.get(1),
                        deltaRobotPos.get(2), deltaRobotPos.get(3));
            }

            float headingChange = (float) VuforiaNavigator.NormalizeAngle(newOdomHeading - theta);
            float averageHeading = theta + headingChange / 2.0f;

            final float sin = (float) (Math.sin(averageHeading));
            final float cosin = (float) (Math.cos(averageHeading));

            float newX = x + deltaRobotPos.get(0) * sin + deltaRobotPos.get(1) * cosin;
            float newY = y + deltaRobotPos.get(1) * sin - deltaRobotPos.get(0) * cosin;

            if (LOG_ODOMETRY) {
                BetaLog.dd(ODOMETRY_TAG, "newPos = %.2f %.2f %.2f", newX, newY, newOdomHeading);
                BetaLog.dd(ODOMETRY_TAG, "headingChange: %.2f", headingChange);
                BetaLog.dd(ODOMETRY_TAG, "UH OH: %b", newOdomHeading == theta && newTicks.dotProduct(newTicks) > 0.5);
            }
            last_Wheel_Ticks = curTicks;
            return new float[]{newX, newY, newOdomHeading};
    }

    /**
     * Get current robot heading.
     * @return Heading in radians
     */
    public float getHeadingRadians(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        float heading = angles.firstAngle + this.initHeadingRadians;
        return (float) VuforiaNavigator.NormalizeAngle(heading);
    }

    /**
     * Get initial robot heading.
     * @return Initial heading in radians.
     */
    public float getInitHeadingRadians(){
        return initHeadingRadians;
    }

    /**
     * Get initial robot heading.
     * @return Initial heading in degrees.
     */
    public float getInitHeadingDegrees (){
        return initHeadingRadians*180.0f/(float)Math.PI;
    }

    /**
     * Set new gyro heading at current robot pose.
     * @param newHeadingRadians new heading in radians.
     */
    public void resetHeadingRadians(float newHeadingRadians){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        initHeadingRadians = (float)VuforiaNavigator.NormalizeAngle(newHeadingRadians - angles.firstAngle);
    }

    /**
     * Set new gyro heading at current robot pose.
     * @param newHeadingDegrees new heading in degrees.
     */
    public void resetHeadingDegrees(float newHeadingDegrees){
        float newHeadingRadians = newHeadingDegrees * (float)Math.PI / 180.0f;
        resetHeadingRadians(newHeadingRadians);
    }

}

