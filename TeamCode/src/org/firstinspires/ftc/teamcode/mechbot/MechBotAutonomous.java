package org.firstinspires.ftc.teamcode.mechbot;

        import android.graphics.PointF;

        import com.qualcomm.robotcore.hardware.DistanceSensor;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
        import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
        import org.firstinspires.ftc.teamcode.logging.BetaLog;
        import org.firstinspires.ftc.teamcode.logging.LoggingLinearOpMode;
        import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
        import org.firstinspires.ftc.teamcode.util.CubicSpline;
        import org.firstinspires.ftc.teamcode.util.ParametricFunction;
        import org.firstinspires.ftc.teamcode.util.geometry.Circle;
        import org.firstinspires.ftc.teamcode.util.geometry.VectorMath;

/**
 * Abstract class which extends LoggingLinearOpMode, has a MechBot, and contains methods for
 * operating a MechBot.
 *
 * Note: Subclasses of MechBotAutonomous must call the MechBotAutonomous.setBot(MechBot) method to
 * provide a reference to a MechBot object (or, more typically an instance of a class that extends
 * MechBot). This must occur before any of the other MechBotAutonomous methods are called. It is the
 * responsibility of the subclass to initialize the MechBot.
 *
 * In general, the robot-driving methods in this class will keep odometry updated in their control
 * loops, even if they are not relying on odometry for navigation. By convention, they assume that
 * odometry has been updated just prior to entry. They perform a final update of odometry just prior
 * to exit.
 */

public abstract class MechBotAutonomous extends LoggingLinearOpMode {


    /**
     * The robot: must be instantiated AND initialized in a class that extends MechbotAutonomous, and
     * must be an instance of MechBot, or of a class that extends MechBot.
     */
    private MechBot bot = null;

    /**
     * The robot pose (x in cm, y in cm, heading in radians).
     */
    protected float[] robotXYTheta = null;

    protected enum FollowSide {LEFT, RIGHT}

    protected enum RotationDirection{COUNTER_CLOCK,CLOCK}

    /**
     * A utility interface.
     */
    protected interface Predicate{
        boolean isTrue();
    }

    /**
     * Interface to control robot speed.
     */
    protected interface DriveSpeedControl {
        boolean isTrue();
        float speed();
    }

    protected interface DriveControl {
        float [] getXYThetaOffset();
        boolean isFinished();
    }

    /**
     * Interface to be used as a call-back to obtain robot pose via any available method.
     * Note: Upon implementing this interface, it will (in most expected cases) be necessary
     * to include an update of odometry within the getXYTheta method.
     */
    protected interface PoseCallBack{
        float[] getXYTheta(float[] xyTheta, float heading, MechBot mechBot);
    }

    /**
     * Interface to control motion in methods involving driving a path and changing orientation
     * at the same time.
     */
    protected interface DriveTurnControl{
        boolean finished(float s);
        float targetHeadingDegrees(float s);
    }

    protected interface KalmanFilter{
        void update();
    }

    protected float findClosestPt(float x0, float y0, float s0, ParametricFunction pf) {
        float epsilon = .0001f;
        float delta = 100;
        while(delta > epsilon && opModeIsActive()) {
            VectorF p = pf.p(s0);
            VectorF d1 = pf.d1(s0);
            VectorF d2 = pf.d2(s0);
            float f = (p.get(0) - x0) * d1.get(0) + (p.get(1) - y0) * d1.get(1);
            float fDeriv = (p.get(0) - x0) * d2.get(0) + d1.get(0) * d1.get(0) + (p.get(1) - y0) * d2.get(1) + d1.get(1) * d1.get(1);
            float s = s0 - f / fDeriv;
            delta = Math.abs(s0 - s);
            s0 = s;
        }
        return s0;
    }

    protected abstract class Kalman1D implements KalmanFilter {
        float[] oldXYTheta;
        float P;
        int index;
        public Kalman1D(float variance, int index) {
            oldXYTheta = robotXYTheta.clone();
            P = variance;
            this.index = index;
        }

        public float getProcessVariance() {
            return (float)Math.pow(Math.abs((robotXYTheta[index] - oldXYTheta[index]) * 0.2), 2);
        }

        public abstract float getMeasurement();

        public abstract float getPositionFromMeasurement(float measurement);

        public void update() {
            float oldXEstimate = oldXYTheta[index];
            float xEstimateMinus = robotXYTheta[index];
            float PMinus = P + getProcessVariance();
            float Z = getMeasurement();
            float R = (float)(0.05 * Z * 0.05 * Z);
            float xMeasurement = getPositionFromMeasurement(Z);
            float xEstimate = (R*xEstimateMinus + PMinus * Z) / (R + PMinus);
            P = PMinus * R / (PMinus + R);
            robotXYTheta[index] = xEstimate;
            oldXYTheta = robotXYTheta.clone();
        }
    }

    protected final String DRIVE_DIRECTION_GYRO_TAG = "DRIVE_DIRECTION_GYRO";
    protected final boolean DRIVE_DIRECTION_GYRO_LOG = false;

    protected final String TURN_ANGLE_TAG = "TURN_ANGLE_TAG";
    protected final boolean TURN_ANGLE_LOG = false;

    protected final String TURN_TO_HEADING_TAG = "TURN_TO_HEADING";
    protected final boolean TURN_TO_HEADING_LOG = false;

    protected final String DRIVE_GYRO_TIME_TAG = "DRIVE_GYRO_TIME";
    protected final boolean DRIVE_GYRO_TIME_LOG = false;

    protected final String FOLLOW_LINE_PROP_TAG = "LINE_FOLLOW_PROP";
    protected final boolean FOLLOW_LINE_PROP_LOG = false;

    protected final String TURN_WITH_PREDICATE_TAG = "TURN_WITH_PREDICATE";
    protected final boolean TURN_WITH_PREDICATE_LOG = false;

    protected final String FOLLOW_WALL_TAG = "FOLLOW_WALL";
    protected final boolean FOLLOW_WALL_LOG = false;

    protected final String ADJUST_HEADING_WITH_WALL_TAG = "ADJUST_HEADING_WITH_WALL";
    protected final boolean ADJUST_HEADING_WITH_WALL_LOG = false;

    protected final boolean ADJUST_HEADING_LOG = false;
    protected final String ADJUST_HEADING_TAG = "ADJUST_HEADING";

    protected final boolean DRIVE_TO_POSITION_GYRO_LOG = true;
    protected final String DRIVE_TO_POSITION_GYRO_TAG = "DRIVE_TO_POSITON_GYRO";

    public final float STANDARD_TOLERANCE = 2f; //Degrees
    public final float STANDARD_LATENCY = 0.3f; //Seconds
    private final float HEADING_CORRECTION_FACTOR = 2.0f;
    private final float DISTANCE_CORRECTION_FACTOR = 2.0f;
    private final float DISTANCE_INTEGRAL_CORRECTION_FACTOR = 0.5f;
    private final float MAX_CORRECTION_SPEED = 20.0f;
    public final float HSV_SAT_CUT_OFF = .5f;

    /**
     * Provide MechBotAutonomous with a reference to a MechBot.
     * This method must be called before using other MechBotAutonomous methods.
     * @param mechBot
     */
    protected void setBot(MechBot mechBot){
        bot = mechBot;
    }


    /**
     * Follow a wall using a distance sensor to maintain desired distance
     * @param followDirectionDegrees Direction of desired travel in degrees (field coordinates)
     * @param side  Side of wall where robot is located (when facing in direction of travel)
     * @param targetHeadingDegrees  Target gyro heading (orientation) of robot during travel
     * @param targetDistance  Target distance from wall in cm
     * @param followSpeed  Travel speed in cm/sec
     * @param sensor  The distance sensor being used
     * @param sensorPos  The (x,y) position of the sensor on the robot, in cm
     * @param finish  Predicate used to determine when to stop
     */
    protected void followWall(float followDirectionDegrees, FollowSide side, float targetHeadingDegrees, float targetDistance,
                              float followSpeed, DistanceSensor sensor, PointF sensorPos, Predicate finish){

        float followDirectionRads = followDirectionDegrees * (float)Math.PI/180.0f;
        float targetHeadingRads = targetHeadingDegrees * (float)Math.PI/180.0f;
        if (FOLLOW_WALL_LOG) BetaLog.dd(FOLLOW_WALL_TAG, "Entering follow wall");


        while (opModeIsActive()){

            float heading = bot.getHeadingRadians();
            robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
            if (FOLLOW_WALL_LOG) BetaLog.dd(FOLLOW_WALL_TAG,"X = %.1f  Y = %.1f  Heading = %.1f",
                    robotXYTheta[0], robotXYTheta[1], robotXYTheta[2] * 180.0/Math.PI);
            if (finish.isTrue()) break;

            float angleDiff = heading - followDirectionRads;
            float cos = (float)Math.cos(angleDiff);
            float sin = (float)Math.sin(angleDiff);
            float sDistance = (float)sensor.getDistance(DistanceUnit.CM);
            float xCenter = side == FollowSide.RIGHT? sDistance - sensorPos.x * cos + sensorPos.y * sin :
                    -sDistance - sensorPos.x * cos + sensorPos.y * sin;
            float errDist = side == FollowSide.RIGHT? targetDistance - xCenter : -targetDistance - xCenter;

            float correctionSpeed = DISTANCE_CORRECTION_FACTOR * errDist;
            if (Math.abs(correctionSpeed) > MAX_CORRECTION_SPEED) correctionSpeed = MAX_CORRECTION_SPEED * Math.signum(correctionSpeed);

            float vxRobot = followSpeed * sin + correctionSpeed * cos;
            float vyRobot = followSpeed * cos - correctionSpeed * sin;
            float vaRobot = HEADING_CORRECTION_FACTOR * (float)VuforiaNavigator.NormalizeAngle(targetHeadingRads - heading);

            bot.setDriveSpeed(vxRobot, vyRobot, vaRobot);

        }

        bot.setDrivePower(0,0,0);
        float heading = bot.getHeadingRadians();
        robotXYTheta = bot.updateOdometry(robotXYTheta, heading);

    }

    /**
     * Follow a wall using a distance sensor to maintain desired distance
     * @param followDirectionDegrees Direction of desired travel in degrees (field coordinates)
     * @param side  Side of wall where robot is located (when facing in direction of travel)
     * @param targetHeadingDegrees  Target gyro heading (orientation) of robot during travel
     * @param targetDistance  Target distance from wall in cm
     * @param followSpeed  Travel speed in cm/sec
     * @param sensor  The distance sensor being used
     * @param sensorPos  The (x,y) position of the sensor on the robot, in cm
     * @param control  Predicate used to determine when to stop
     */
    protected void followWall(float followDirectionDegrees, FollowSide side, float targetHeadingDegrees, float targetDistance,
                              float followSpeed, DistanceSensor sensor, PointF sensorPos, DriveSpeedControl control, KalmanFilter kalmanFilter){

        float followDirectionRads = followDirectionDegrees * (float)Math.PI/180.0f;
        float targetHeadingRads = targetHeadingDegrees * (float)Math.PI/180.0f;
        if (FOLLOW_WALL_LOG) BetaLog.dd(FOLLOW_WALL_TAG, "Entering follow wall");


        while (opModeIsActive()){

            float heading = bot.getHeadingRadians();
            robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
            kalmanFilter.update();
            if (FOLLOW_WALL_LOG) BetaLog.dd(FOLLOW_WALL_TAG,"X = %.1f  Y = %.1f  Heading = %.1f",
                    robotXYTheta[0], robotXYTheta[1], robotXYTheta[2] * 180.0/Math.PI);
            if (control.isTrue()) break;

            float actualSpeed = followSpeed * control.speed();

            float angleDiff = heading - followDirectionRads;
            float cos = (float)Math.cos(angleDiff);
            float sin = (float)Math.sin(angleDiff);
            float sDistance = (float)sensor.getDistance(DistanceUnit.CM);
            float xCenter = side == FollowSide.RIGHT? sDistance - sensorPos.x * cos + sensorPos.y * sin :
                    -sDistance - sensorPos.x * cos + sensorPos.y * sin;
            float errDist = side == FollowSide.RIGHT? targetDistance - xCenter : -targetDistance - xCenter;

            float correctionSpeed = DISTANCE_CORRECTION_FACTOR * errDist;
            if (Math.abs(correctionSpeed) > MAX_CORRECTION_SPEED) correctionSpeed = MAX_CORRECTION_SPEED * Math.signum(correctionSpeed);

            float vxRobot = actualSpeed * sin + correctionSpeed * cos;
            float vyRobot = actualSpeed * cos - correctionSpeed * sin;
            float vaRobot = HEADING_CORRECTION_FACTOR * (float)VuforiaNavigator.NormalizeAngle(targetHeadingRads - heading);

            bot.setDriveSpeed(vxRobot, vyRobot, vaRobot);

        }

        bot.setDrivePower(0,0,0);
        float heading = bot.getHeadingRadians();
        robotXYTheta = bot.updateOdometry(robotXYTheta, heading);

    }


    /**
     * Follow a wall using a distance sensor to maintain desired distance
     * @param followDirectionDegrees Direction of desired travel in degrees (field coordinates)
     * @param side  Side of wall where robot is located (when facing in direction of travel)
     * @param targetHeadingDegrees  Target gyro heading (orientation) of robot during travel
     * @param targetDistance  Target distance from wall in cm
     * @param followSpeed  Travel speed in cm/sec
     * @param sensor  The distance sensor being used
     * @param sensorPos  The (x,y) position of the sensor on the robot, in cm
     * @param finish  Predicate used to determine when to stop
     */
    protected void followWallPI(float followDirectionDegrees, FollowSide side, float targetHeadingDegrees, float targetDistance,
                              float followSpeed, DistanceSensor sensor, PointF sensorPos, Predicate finish){

        float followDirectionRads = followDirectionDegrees * (float)Math.PI/180.0f;
        float targetHeadingRads = targetHeadingDegrees * (float)Math.PI/180.0f;
        if (FOLLOW_WALL_LOG) BetaLog.dd(FOLLOW_WALL_TAG, "Entering follow wall");

        ElapsedTime et = new ElapsedTime();
        double prevSecs = et.seconds();
        float errorIntegral = 0.0f;

        while (opModeIsActive()){

            double secs = (float)et.seconds();
            float dt = (float)(secs - prevSecs);
            prevSecs = secs;

            float heading = bot.getHeadingRadians();
            robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
            if (FOLLOW_WALL_LOG) BetaLog.dd(FOLLOW_WALL_TAG,"X = %.1f  Y = %.1f  Heading = %.1f",
                    robotXYTheta[0], robotXYTheta[1], robotXYTheta[2] * 180.0/Math.PI);
            if (finish.isTrue()) break;

            float angleDiff = heading - followDirectionRads;
            float cos = (float)Math.cos(angleDiff);
            float sin = (float)Math.sin(angleDiff);
            float sDistance = (float)sensor.getDistance(DistanceUnit.CM);
            float xCenter = side == FollowSide.RIGHT? sDistance - sensorPos.x * cos + sensorPos.y * sin :
                    -sDistance - sensorPos.x * cos + sensorPos.y * sin;
            float errDist = side == FollowSide.RIGHT? targetDistance - xCenter : -targetDistance - xCenter;
            errorIntegral += errDist * dt;

            float correctionSpeed = DISTANCE_CORRECTION_FACTOR * errDist + DISTANCE_INTEGRAL_CORRECTION_FACTOR * errorIntegral;
            if (Math.abs(correctionSpeed) > MAX_CORRECTION_SPEED) correctionSpeed = MAX_CORRECTION_SPEED * Math.signum(correctionSpeed);

            float vxRobot = followSpeed * sin + correctionSpeed * cos;
            float vyRobot = followSpeed * cos - correctionSpeed * sin;
            float vaRobot = HEADING_CORRECTION_FACTOR * (float)VuforiaNavigator.NormalizeAngle(targetHeadingRads - heading);

            bot.setDriveSpeed(vxRobot, vyRobot, vaRobot);

        }

        bot.setDrivePower(0,0,0);
        float heading = bot.getHeadingRadians();
        robotXYTheta = bot.updateOdometry(robotXYTheta, heading);

    }


    /**
     * Using gyro, turns robot the specified number of degrees.
     * @param angle Angle to turn in degrees, can be positive (counter-clock) or negative (clock)
     * @param tolerance Indicates the acceptable range of error, plus/minus degrees
     * @param latency Estimated latency, in seconds, between successive heading measurements
     */
    protected void turnAngleGyro(float angle, float tolerance, float latency) {
        //Tolerance in degrees latency seconds.
        //Convert to radians.
        angle = angle * (float)Math.PI/180f;
        tolerance = tolerance * (float)Math.PI/180f;

        final float vaMin = 1.5f * tolerance / latency;
        final float C = 0.75f / latency;
        final float vaMax = 0.3f * (float)Math.PI;
        float heading = bot.getHeadingRadians();
        float targetHeading = heading + angle;
        float offset = (float) VuforiaNavigator.NormalizeAngle(targetHeading - heading);

        while (opModeIsActive() && Math.abs(offset) > tolerance) {
            float absAdjustedOffset = Math.abs(offset) - tolerance;
            float absVa = vaMin + C * absAdjustedOffset;
            absVa = Math.min(absVa, vaMax);
            float va = absVa * Math.signum(offset);
            if(TURN_ANGLE_LOG) BetaLog.dd(TURN_ANGLE_TAG,"Turning va = %.2f hd = %.0f, off = %.0f absAdjOff = %.0f", va, heading, offset, absAdjustedOffset);
            bot.setDriveSpeed(0, 0, va);
            heading = bot.getHeadingRadians();
            robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
            offset = targetHeading - heading;
        }
        bot.setDrivePower(0, 0, 0);
        heading = bot.getHeadingRadians();
        robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
    }

    /**
     * Turns robot to a specific heading, in whatever direction results in smallest turn.
     * @param targetHeading Target Heading in degrees
     * @param tolerance Indicates acceptable range of error in +/- degrees
     * @param latency Estimate of latency, in seconds, between successive heading measurements
     */
    protected void turnToHeadingGyro(float targetHeading, float tolerance, float latency, float maxDegreesPerSecond){
        if(TURN_TO_HEADING_LOG) {
            BetaLog.dd(TURN_TO_HEADING_TAG, "Entering turnToHeadingGyro");
        }
        //Tolerance in degrees latency seconds.
        tolerance = tolerance * (float)Math.PI/180f;
        targetHeading = targetHeading * (float)Math.PI/180f;

        final float vaMin = 1.5f * tolerance / latency;
        final float C = 0.75f / latency;
        final float vaMax = maxDegreesPerSecond * (float)Math.PI / 180;
        float heading;
        float offset;

        while (opModeIsActive()) {
            heading = bot.getHeadingRadians();
            if (TURN_TO_HEADING_LOG){
                BetaLog.dd(TURN_TO_HEADING_TAG, "Heading: %.2f  Target Heading: %.2f  Entering odometry...", heading * 180.0/Math.PI,
                        targetHeading * 180.0/Math.PI);
            }
            robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
            offset = (float)VuforiaNavigator.NormalizeAngle(targetHeading - heading);
            if(Math.abs(offset) <= tolerance) break;

            float absAdjustedOffset = Math.abs(offset) - tolerance;
            float absVa = vaMin + C * absAdjustedOffset;

            absVa = Math.min(absVa, vaMax);
            float va = absVa * Math.signum(offset);
            bot.setDriveSpeed(0, 0, va);
        }
        bot.setDrivePower(0, 0, 0);
        heading = bot.getHeadingRadians();
        robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
        if(TURN_TO_HEADING_LOG) {
            BetaLog.dd(TURN_TO_HEADING_TAG, "Exiting turnToHeadingGyro, Final Heading: %.2f", heading * 180/ Math.PI);
        }
    }

    protected void turnToHeadingGyro(float targetHeading, float tolerance, float latency) {
        turnToHeadingGyro(targetHeading, tolerance, latency, 108);
    }


    /**
     * Turns robot to a specific heading, in whatever direction results in smallest turn.
     * @param targetHeading Target Heading in degrees
     * @param tolerance Indicates acceptable range of error in +/- degrees
     * @param latency Estimate of latency, in seconds, between successive heading measurements
     * @param finished Predicate object can terminate operation early, and also add functionality during control loop
     */
    protected void turnToHeadingGyro(float targetHeading, float tolerance, float latency, float maxDegreesPerSecond, Predicate finished){
        //Tolerance in degrees latency seconds.
        tolerance = tolerance * (float)Math.PI/180f;
        targetHeading = targetHeading * (float)Math.PI/180f;

        final float vaMin = 1.5f * tolerance / latency;
        final float C = 0.75f / latency;
        final float vaMax = maxDegreesPerSecond * (float)Math.PI / 180;
        float heading;
        float offset;
        if(TURN_TO_HEADING_LOG) {
            BetaLog.dd(TURN_TO_HEADING_TAG, "Entering Turn to heading");
        }
        while (opModeIsActive()) {
            heading = bot.getHeadingRadians();
            robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
            offset = (float)VuforiaNavigator.NormalizeAngle(targetHeading - heading);
            if(TURN_TO_HEADING_LOG) {
                BetaLog.dd(TURN_TO_HEADING_TAG, "Heading=%.1f  Offset=%.1f", heading * 180.0 / Math.PI, offset * 180.0 / Math.PI);
            }
            if(Math.abs(offset) <= tolerance || finished.isTrue()) break;

            float absAdjustedOffset = Math.abs(offset) - tolerance;
            float absVa = vaMin + C * absAdjustedOffset;

            absVa = Math.min(absVa, vaMax);
            float va = absVa * Math.signum(offset);
            //if(TURN_TO_HEADING_LOG)BetaLog.dd(TURN_TO_HEADING_TAG,"Turning va = %.2f hd = %.0f, off = %.0f absAdjOff = %.0f", va, heading, offset, absAdjustedOffset);
            bot.setDriveSpeed(0, 0, va);
        }
        bot.setDrivePower(0, 0, 0);
        heading = bot.getHeadingRadians();
        robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
    }


    /**
     * Turns robot to a specific heading, in the specified direction.
     * @param targetHeading Target Heading in degrees
     * @param tolerance Indicates acceptable range of error in +/- degrees
     * @param latency Estimate of latency, in seconds, between successive heading measurements
     * @param rotationDirection Indicates the direction in which robot is to turn
     */
    protected void turnToHeadingGyro(float targetHeading, float tolerance, float latency, float maxDegreesPerSecond, RotationDirection rotationDirection){
        //Tolerance in degrees latency seconds.
        tolerance = tolerance * (float)Math.PI/180f;
        targetHeading = targetHeading * (float)Math.PI/180f;
        final float maxOvershootRadians = 20.0f * (float)Math.PI / 180.0f;  //This means that the maximum possible turn is 340 degrees but solves problem of overshoot.

        final float vaMin = 1.5f * tolerance / latency;
        final float C = 0.75f / latency;
        final float vaMax = maxDegreesPerSecond * (float)Math.PI / 180; //Was .5 on 1/11/18
        float heading;
        float offset;

        while (opModeIsActive()) {
            heading = bot.getHeadingRadians();
            robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
            offset = (float)VuforiaNavigator.NormalizeAngle(targetHeading - heading);
            if(Math.abs(offset) <= tolerance) break;

            if(rotationDirection == RotationDirection.COUNTER_CLOCK){
                if(offset < (-maxOvershootRadians))offset += 2.0f*(float)Math.PI;
            }else{
                if(offset > maxOvershootRadians)offset -= 2.0f*(float)Math.PI;
            }

            float absAdjustedOffset = Math.abs(offset) - tolerance;
            float absVa = vaMin + C * absAdjustedOffset;

            absVa = Math.min(absVa, vaMax);
            float va = absVa * Math.signum(offset);
            if(TURN_TO_HEADING_LOG)BetaLog.dd(TURN_TO_HEADING_TAG,"Turning va = %.2f hd = %.0f, off = %.0f absAdjOff = %.0f", va, heading, offset, absAdjustedOffset);
            bot.setDriveSpeed(0, 0, va);
        }
        bot.setDrivePower(0, 0, 0);
        heading = bot.getHeadingRadians();
        robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
    }

    /**
     * Turns robot to a specific heading, in the specified direction.
     * @param targetHeading Target Heading in degrees
     * @param tolerance Indicates acceptable range of error in +/- degrees
     * @param latency Estimate of latency, in seconds, between successive heading measurements
     * @param rotationDirection Indicates the direction in which robot is to turn
     */
    protected void turnToHeadingGyro(float targetHeading, float tolerance, float latency, float maxDegreesPerSecond, Predicate finished, RotationDirection rotationDirection){
        //Tolerance in degrees latency seconds.
        tolerance = tolerance * (float)Math.PI/180f;
        targetHeading = targetHeading * (float)Math.PI/180f;
        final float maxOvershootRadians = 20.0f * (float)Math.PI / 180.0f;  //This means that the maximum possible turn is 340 degrees but solves problem of overshoot.

        final float vaMin = 1.5f * tolerance / latency;
        final float C = 0.75f / latency;
        final float vaMax = maxDegreesPerSecond * (float)Math.PI / 180; //Was .5 on 1/11/18
        float heading;
        float offset;

        while (opModeIsActive()) {
            heading = bot.getHeadingRadians();
            robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
            offset = (float)VuforiaNavigator.NormalizeAngle(targetHeading - heading);
            if(Math.abs(offset) <= tolerance || finished.isTrue()) break;

            if(rotationDirection == RotationDirection.COUNTER_CLOCK){
                if(offset < (-maxOvershootRadians))offset += 2.0f*(float)Math.PI;
            }else{
                if(offset > maxOvershootRadians)offset -= 2.0f*(float)Math.PI;
            }

            float absAdjustedOffset = Math.abs(offset) - tolerance;
            float absVa = vaMin + C * absAdjustedOffset;

            absVa = Math.min(absVa, vaMax);
            float va = absVa * Math.signum(offset);
            if(TURN_TO_HEADING_LOG)BetaLog.dd(TURN_TO_HEADING_TAG,"Turning va = %.2f hd = %.0f, off = %.0f absAdjOff = %.0f", va, heading, offset, absAdjustedOffset);
            bot.setDriveSpeed(0, 0, va);
        }
        bot.setDrivePower(0, 0, 0);
        heading = bot.getHeadingRadians();
        robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
    }

    protected void turnToHeadingGyro(float targetHeading, float tolerance, float latency, RotationDirection rotationDirection){
        turnToHeadingGyro(targetHeading, tolerance, latency, 108, rotationDirection);
    }

    /**
     * Turn robot at specified angular speed until the predicate evaluates to true.
     * @param va Angular speed
     * @param finish A predicate object; rotation will stop when isTrue() evaluates to true.
     */
    protected void turnWithPredicate(float va, Predicate finish){

        if (TURN_WITH_PREDICATE_LOG) BetaLog.dd(TURN_WITH_PREDICATE_TAG, "Entering TURN_WITH_PREDICATE");
        float headingRadians;

        bot.setDriveSpeed(0,0, va );

        while (opModeIsActive()){
            headingRadians = bot.getHeadingRadians();
            robotXYTheta = bot.updateOdometry(robotXYTheta, headingRadians);
            if (TURN_WITH_PREDICATE_LOG) BetaLog.dd(TURN_WITH_PREDICATE_TAG, "Z %.1f  X %.1f  Phi %.1f",
                    robotXYTheta[0], robotXYTheta[1], robotXYTheta[2] * 180.0/Math.PI);
            if (finish.isTrue()) break;
        }

        bot.setDrivePower(0,0,0);
        headingRadians = bot.getHeadingRadians();
        bot.updateOdometry(robotXYTheta, headingRadians);
    }


    /**
     * Drives robot with specified velocity for specified duration.
     * @param vx x-component (in robot coordinate system) of velocity in cm/sec
     * @param vy y-component (in robot coordinate system) of velocity in cm/sec
     * @param duration duration of drive, in milliseconds
     */
    protected void driveStraightGyroTime(float vx, float vy, float duration) {
        try {
            BetaLog.initialize();
            final float C_ANGLE = 2.0f;
            final float initialHeading = bot.getHeadingRadians();
            ElapsedTime et = new ElapsedTime();
            double scale = bot.setDriveSpeed(vx, vy, 0);
            if(DRIVE_GYRO_TIME_LOG)BetaLog.dd(DRIVE_GYRO_TIME_TAG,"<Debug> DriveStrGyroTime Pre Scale duration = %.0f vx = %.0f vy = %.0f", duration, vx, vy);

            duration /= scale;
            vx *= scale;
            vy *= scale;
            if(DRIVE_GYRO_TIME_LOG)BetaLog.dd(DRIVE_GYRO_TIME_TAG,"<Debug> DriveStrGyroTime Post Scale( %.2f ) duration = %.0f vx = %.0f vy = %.0f", scale, duration, vx, vy);

            while (opModeIsActive()) {
                double etms = et.milliseconds();
                if (etms > duration) break;
                float currentHeading = bot.getHeadingRadians();
                float va = (initialHeading - currentHeading) * C_ANGLE;
                if(DRIVE_GYRO_TIME_LOG)BetaLog.dd(DRIVE_GYRO_TIME_TAG,"<Debug> Initial Angle = %.1f Current Angle = %.1f", initialHeading, currentHeading);
                bot.setDriveSpeed(vx, vy, va);
            }
            bot.setDriveSpeed(0, 0, 0);
        }
        finally {
            {
                BetaLog.close();
            }
        }
    }


    /**
     * Drive robot at specified speed, in specified direction (in field coordinate system), until specified condition is met,
     * while maintaining robot orientation (heading) of 0 degrees.
     * @param speedCMs Drive speed in cm/sec
     * @param directionAngleDegrees Drive direction in degrees (in field coordinates, based on gyro heading)
     * @param finish Predicate object, with isTrue() method that indicates when to stop (stop when true)
     */
    protected void driveDirectionGyro(float speedCMs, float directionAngleDegrees, Predicate finish){
        if (DRIVE_DIRECTION_GYRO_LOG) BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "Entering driveDirectionGyro");
        float directionAngleRadians = directionAngleDegrees * (float) Math.PI / 180.0f;
        while (opModeIsActive()) {

            float heading = bot.getHeadingRadians();

            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "Heading = %.2f",
                        heading * 180.0 / Math.PI ); //Fixed convert error

            this.robotXYTheta = bot.updateOdometry(robotXYTheta, heading);

            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "z = %.2f  x = %.2f  Phi = %.2f",
                        robotXYTheta[0], robotXYTheta[1], robotXYTheta[2] * 180.0 / Math.PI); //Fixed convert error

            if (finish.isTrue()) break;

            float vx = -speedCMs * (float) Math.sin(directionAngleRadians - heading);
            float vy = speedCMs * (float) Math.cos(directionAngleRadians - heading);
            float va = -HEADING_CORRECTION_FACTOR * heading;

            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "vx = %.2f  vy = %.2f  va = %.2f", vx, vy, va * 180.0 / Math.PI);

            bot.setDriveSpeed(vx, vy, va);

        }
        bot.setDriveSpeed(0,0,0);
        float heading = bot.getHeadingRadians();
        robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
    }


    /**
     * Drive robot at specified speed, in specified direction (in field coordinate system), until specified condition is met,
     * while maintaining specified robot orientation (heading).
     * @param speedCMs Drive speed in cm/sec
     * @param directionAngleDegrees Drive direction in degrees (in field coordinates, based on gyro heading)
     * @param driveControl DriveControl object, with isFinished() method that indicates when to stop (stop when true) and getXYThetaOffset() that returns offsets in the desired x, y, and theta
     */
    protected void driveDirectionGyro(float speedCMs, float directionAngleDegrees, float targetHeadingDegrees, DriveControl driveControl){

        if (DRIVE_DIRECTION_GYRO_LOG) BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "Entering driveDirectionGyro");
        float directionAngleRadians = directionAngleDegrees * (float) Math.PI / 180.0f;
        float targetHeadingRadians = targetHeadingDegrees * (float) Math.PI / 180.0f;

        while (opModeIsActive()) {
            float heading = bot.getHeadingRadians();

            this.robotXYTheta = bot.updateOdometry(robotXYTheta, heading);

            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "z = %.2f  x = %.2f  Phi = %.2f",
                        robotXYTheta[0], robotXYTheta[1], robotXYTheta[2] * 180.0 / Math.PI);

            if (driveControl.isFinished()) break;

            float[] xyThetaError = driveControl.getXYThetaOffset();
            float sin = (float)Math.sin(heading);
            float cos = (float)Math.cos(heading);
            float xRError = xyThetaError[0] * sin - xyThetaError[1] * cos;
            float yRError = xyThetaError[0] * cos + xyThetaError[1] * sin;

            float vxCorr = - xRError * DISTANCE_CORRECTION_FACTOR;
            float vyCorr = - yRError * DISTANCE_CORRECTION_FACTOR;

            float vx = -speedCMs * (float) Math.sin(directionAngleRadians - heading) + vxCorr;
            float vy = speedCMs * (float) Math.cos(directionAngleRadians - heading) + vyCorr;

            float va = -HEADING_CORRECTION_FACTOR * (float)VuforiaNavigator.NormalizeAngle(heading - targetHeadingRadians);


            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "vx = %.2f  vy = %.2f  va = %.2f", vx, vy, va * 180.0 / Math.PI);

            bot.setDriveSpeed(vx, vy, va);

        }
        bot.setDrivePower(0,0,0);
        float heading = bot.getHeadingRadians();
        robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
    }

    protected VectorF fieldToBot(VectorF vField, float heading) {
        float sinTheta = (float)Math.sin(heading);
        float cosTheta = (float)Math.cos(heading);
        return new VectorF(vField.get(0) * sinTheta - vField.get(1) * cosTheta, vField.get(0) * cosTheta + vField.get(1) * sinTheta);
    }

    protected void driveFunction(float speed, float s0, ParametricFunction pf, Predicate finish) {
        while (opModeIsActive()) {
            float heading = bot.getHeadingRadians();
            robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
            System.out.println("x="+robotXYTheta[0]+"   y="+robotXYTheta[1]);
            if(finish.isTrue())
                break;

            s0 = findClosestPt(robotXYTheta[0], robotXYTheta[1], s0, pf);
            System.out.println(s0);
            VectorF targetPos = pf.p(s0);
            VectorF posOffset = targetPos.subtracted(new VectorF(robotXYTheta[0], robotXYTheta[1]));
            VectorF d1 = pf.d1(s0);
            VectorF d2 = pf.d2(s0);

            VectorF totalV = posOffset.multiplied(2*DISTANCE_CORRECTION_FACTOR).added(d1.multiplied(speed/d1.magnitude()));
            VectorF totalVBot = fieldToBot(totalV, heading);

            float targetHeading = (float)Math.atan2(d1.get(1), d1.get(0));
            float targetHeadingChangeRate = (d2.get(1) * d1.get(0) - d2.get(0) * d1.get(1)) / (float)Math.pow(d1.dotProduct(d1), 1.5f) * speed;
            float headingOffset = (float)VuforiaNavigator.NormalizeAngle(targetHeading - heading);
            float va = targetHeadingChangeRate + headingOffset * HEADING_CORRECTION_FACTOR;

            bot.setDriveSpeed(totalVBot.get(0), totalVBot.get(1), va);
        }
    }

    protected void driveSpline(float speed, boolean reverse, CubicSpline spline){
        spline.setIndex(0);
        float s0 = 0;
        speed = (float)Math.abs(speed);
        System.out.println("driveSpline: " + spline.getNumSegments() + " segments.");
        while (opModeIsActive()) {
            float heading = bot.getHeadingRadians();
            robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
            s0 = spline.nextClosestPt(robotXYTheta[0], robotXYTheta[1], s0, this);


            if (s0 >=1 && spline.getIndex()==(spline.getNumSegments()-1)) break;
            VectorF targetPos = spline.p(s0);
            VectorF posOffset = targetPos.subtracted(new VectorF(robotXYTheta[0], robotXYTheta[1]));
            VectorF d1 = spline.d1(s0);
            VectorF d2 = spline.d2(s0);

            VectorF fwdVel = d1.multiplied(speed/d1.magnitude());
            VectorF corrVel = posOffset.multiplied(2*DISTANCE_CORRECTION_FACTOR);
            VectorF totalV = fwdVel.added(corrVel);
            VectorF totalVBot = fieldToBot(totalV, heading);

            float targetHeading =  reverse? (float)Math.atan2(-d1.get(1), -d1.get(0)) : (float)Math.atan2(d1.get(1), d1.get(0));
            float targetHeadingChangeRate = (d2.get(1) * d1.get(0) - d2.get(0) * d1.get(1)) / (float)Math.pow(d1.dotProduct(d1), 1.5f) * speed;
            float headingOffset = (float)VuforiaNavigator.NormalizeAngle(targetHeading - heading);
            float va = targetHeadingChangeRate + headingOffset * HEADING_CORRECTION_FACTOR;

            System.out.println("  seg = " + spline.getIndex() + "    s = " + s0 + "  pos = " + robotXYTheta[0] + "  " + robotXYTheta[1]);
            System.out.println();

            bot.setDriveSpeed(totalVBot.get(0), totalVBot.get(1), va);
        }

        bot.setDriveSpeed(0, 0, 0);
    }

    protected void driveToPositionGyro(float vMax, float vMin, float targetX, float targetY, float targetThetaDegrees) {
        driveToPositionGyro(vMax, vMin, targetX, targetY, targetThetaDegrees, 4, 3f);
    }

    protected void driveToPositionGyro(float vMax, float vMin, float targetX, float targetY, float targetThetaDegrees, float cp, float tolerance) {
        float headingTargetRadians = targetThetaDegrees * (float)Math.PI / 180;

        while (opModeIsActive()) {
            float heading = bot.getHeadingRadians();
            this.robotXYTheta = bot.updateOdometry(robotXYTheta, heading);

            float xError = targetX - robotXYTheta[0];
            float yError = targetY - robotXYTheta[1];
            float thetaError = (float)VuforiaNavigator.NormalizeAngle(headingTargetRadians - robotXYTheta[2]);

            if(Math.hypot(xError, yError) < tolerance) {
                break;
            }

            float sinTheta = (float)Math.sin(heading);
            float cosTheta = (float)Math.cos(heading);

            float xErrorRobot = xError * sinTheta - yError * cosTheta;
            float yErrorRobot = xError * cosTheta + yError * sinTheta;

            float vx = xErrorRobot * cp;
            float vy = yErrorRobot * cp;
            float v = (float)Math.hypot(vx, vy);
            if(v > vMax) {
                vx *= vMax / v;
                vy *= vMax / v;
            } else if(v < vMin) {
                vx *= vMin / v;
                vy *= vMin / v;
            }
            float va = HEADING_CORRECTION_FACTOR * thetaError;

            if(DRIVE_TO_POSITION_GYRO_LOG) {
                BetaLog.dd(DRIVE_TO_POSITION_GYRO_TAG, "X: %.2f   Y: %.2f   Theta: %.2f", robotXYTheta[0], robotXYTheta[1], robotXYTheta[2]);
            }

            bot.setDriveSpeed(vx, vy, va);

        }
        bot.setDriveSpeed(0, 0,0);
        float finalHeading = bot.getHeadingRadians();
        robotXYTheta = bot.updateOdometry(robotXYTheta, finalHeading);
    }

    /**
     * Drive robot at specified speed, in specified direction (in field coordinate system), until specified condition is met,
     * while maintaining specified robot orientation (heading).
     * @param speedCMs Drive speed in cm/sec
     * @param directionAngleDegrees Drive direction in degrees (in field coordinates, based on gyro heading)
     * @param headingTargetDegrees heading at which to maintain robot orientation, in degrees
     * @param finish Predicate object, with isTrue() method that indicates when to stop (stop when true)
     */
    protected void driveDirectionGyro(float speedCMs, float directionAngleDegrees, float headingTargetDegrees, Predicate finish){

        if (DRIVE_DIRECTION_GYRO_LOG) BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "Entering driveDirectionGyro");
        float directionAngleRadians = directionAngleDegrees * (float) Math.PI / 180.0f;
        float headingTargetRadians = headingTargetDegrees * (float) Math.PI / 180.0f;

        while (opModeIsActive()) {
            float heading = bot.getHeadingRadians();

            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "Heading = %.2f  Target = %.2f",
                        heading * 180.0 / Math.PI, headingTargetDegrees);

            this.robotXYTheta = bot.updateOdometry(robotXYTheta, heading);

            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "z = %.2f  x = %.2f  Phi = %.2f",
                        robotXYTheta[0], robotXYTheta[1], robotXYTheta[2] * 180.0 / Math.PI);

            if (finish.isTrue()) break;

            float vx = -speedCMs * (float) Math.sin(directionAngleRadians - heading);
            float vy = speedCMs * (float) Math.cos(directionAngleRadians - heading);

            float headingError = (float)VuforiaNavigator.NormalizeAngle(heading - headingTargetRadians);
            if (DRIVE_DIRECTION_GYRO_LOG)BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "HeadingError = %.3f ", headingError);
            float va = -HEADING_CORRECTION_FACTOR * headingError;

            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "vx = %.2f  vy = %.2f  va = %.2f", vx, vy, va * 180.0 / Math.PI);

            bot.setDriveSpeed(vx, vy, va);

        }
        bot.setDrivePower(0,0,0);
        float heading = bot.getHeadingRadians();
        robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
    }

    /**
     * Drive robot at specified speed, in specified direction (in field coordinate system), until specified condition is met,
     * while maintaining specified robot orientation (heading).
     * @param directionAngleDegrees Drive direction in degrees (in field coordinates, based on gyro heading)
     * @param headingTargetDegrees heading at which to maintain robot orientation, in degrees
     * @param speedControl DriveSpeedControl object, with isTrue() method that indicates when to stop (stop when true) and speed() method that indicates the overall velocity
     */
    protected void driveDirectionGyro(float directionAngleDegrees, float headingTargetDegrees, DriveSpeedControl speedControl){

        if (DRIVE_DIRECTION_GYRO_LOG) BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "Entering driveDirectionGyro");
        float directionAngleRadians = directionAngleDegrees * (float) Math.PI / 180.0f;
        float headingTargetRadians = headingTargetDegrees * (float) Math.PI / 180.0f;

        while (opModeIsActive()) {
            float heading = bot.getHeadingRadians();

            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "Heading = %.2f  Target = %.2f",
                        heading * 180.0 / Math.PI, headingTargetDegrees);

            this.robotXYTheta = bot.updateOdometry(robotXYTheta, heading);

            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "z = %.2f  x = %.2f  Phi = %.2f",
                        robotXYTheta[0], robotXYTheta[1], robotXYTheta[2] * 180.0 / Math.PI);

            if (speedControl.isTrue()) break;

            float v = speedControl.speed();
            float vx = -v * (float) Math.sin(directionAngleRadians - heading);
            float vy = v * (float) Math.cos(directionAngleRadians - heading);

            float headingError = (float)VuforiaNavigator.NormalizeAngle(heading - headingTargetRadians);
            if (DRIVE_DIRECTION_GYRO_LOG)BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "HeadingError = %.3f ", headingError);
            float va = -HEADING_CORRECTION_FACTOR * headingError;

            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "vx = %.2f  vy = %.2f  va = %.2f", vx, vy, va * 180.0 / Math.PI);

            bot.setDriveSpeed(vx, vy, va);

        }
        bot.setDrivePower(0,0,0);
        float heading = bot.getHeadingRadians();
        robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
    }


    /**
     * Drive bot along a line specified by a direction angle and a point, with proportionate control of distance from line and heading.
     * @param speedCMs Drive speed, cm/sec
     * @param directionAngleDegrees Direction of the line along which to travel, in degrees
     * @param pointOnLine A single point on the line, in field coordinates
     * @param headingTargetDegrees Target robot heading (i.e., orientation) in degrees
     * @param poseCallBack getPose method must return updated robotXYTheta AND update odometry.
     * @param finish Callback to determine when to stop
     */
    protected void driveLineGyro(float speedCMs, float directionAngleDegrees, PointF pointOnLine, float headingTargetDegrees,
                                 PoseCallBack poseCallBack, Predicate finish) {
        float directionAngleRads = directionAngleDegrees * (float)Math.PI / 180.0f;
        float headingTargetRads = headingTargetDegrees * (float)Math.PI / 180.0f;
        float[] temp;

        while (opModeIsActive()){
            float heading = bot.getHeadingRadians();
            robotXYTheta = poseCallBack.getXYTheta(robotXYTheta, heading, bot);

            if (finish.isTrue()) break;

            //Distance from line:
            float xPrime = (robotXYTheta[0] - pointOnLine.x) * (float)Math.sin(directionAngleRads)
                    - (robotXYTheta[1] - pointOnLine.y) * (float)Math.cos(directionAngleRads);

            float angleDiff = (float)VuforiaNavigator.NormalizeAngle(robotXYTheta[2] - directionAngleRads);
            float sin = (float)Math.sin(angleDiff);
            float cos = (float)Math.cos(angleDiff);

            float vx = speedCMs * sin - DISTANCE_CORRECTION_FACTOR * xPrime * cos;
            float vy = speedCMs * cos + DISTANCE_CORRECTION_FACTOR * xPrime * sin;
            float va = HEADING_CORRECTION_FACTOR * (float)VuforiaNavigator.NormalizeAngle(headingTargetRads - robotXYTheta[2]);

            bot.setDriveSpeed(vx, vy, va);
        }

        bot.setDrivePower(0,0,0);
        float heading = bot.getHeadingRadians();
        robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
    }

    /**
     * Drive an arc at specified speed, while controlling robot orientation using a DriveTurnControl object.
     * @param speed speed in cm/sec, positive for CounterClockwise, negative for Clockwise.
     * @param circle the circle to drive in field coordinates.
     * @param driveTurnControl Methods to provide current heading target and whether travel is finished.
     */
    public void driveArc(float speed, Circle circle, DriveTurnControl driveTurnControl){

        //Robot heading in radians
        float heading;

        //Initial angular position of robot relative to the center of the circle, measured from the
        //plux X direction, in degrees.
        float initArcAngle = (float)Math.atan2(robotXYTheta[1]-circle.center.y, robotXYTheta[0]-circle.center.x)
                * 180.0f / (float)Math.PI;

        //Target heading from previous iteration
        float prevTargetHeadingRadians = 0;

        //Rate of target heading change (radians/sec) during previous iteration
        float rateOfTargetHeadingChange;

        //Indicates whether current iteration is the first iteration through the control loop.
        boolean firstPass = true;

        //Number of full orbits around circle
        float numOrbits = speed >= 0? 0.0f : 1.0f;

        //Previous value of arcAngleTraversed
        float prevArcAngleTraversed = 0.0f;

        //Elapsed time object to allow measurement of the rate of target heading change
        ElapsedTime et = new ElapsedTime();

        //Main Control loop
        while (opModeIsActive()){

            //Get elapsed time for previous iteration
            float elapsedSeconds = (float)et.seconds();
            if (elapsedSeconds < 0.025) continue;
            et.reset();

            //Get current heading and update odometry to obtain current robot position on field
            heading = bot.getHeadingRadians();
            robotXYTheta = bot.updateOdometry(robotXYTheta, heading);

            //Vector from center of circle to center of robot
            float diffX = robotXYTheta[0] - circle.center.x;
            float diffY = robotXYTheta[1] - circle.center.y;

            //Angle between +X axis and vector from center of circle to center of robot, degrees
            float currentArcAngle = (float)Math.atan2(diffY, diffX) * 180.0f / (float)Math.PI;

            //Angle of arc traversed by robot along circle, so far, in degrees, from 0 to +/- 360
            float arcAngleTraversed = (float)VuforiaNavigator.normalizeDegrees360(currentArcAngle - initArcAngle);
            if (prevArcAngleTraversed > 330 && arcAngleTraversed < 30) numOrbits += 1.0f;
            else if (prevArcAngleTraversed < 30 && arcAngleTraversed > 330) numOrbits -= 1.0f;
            float totalArcAngleTraversed = speed >= 0? arcAngleTraversed + numOrbits * 360.0f :
                    arcAngleTraversed - 360.0f + numOrbits * 360.0f;
            prevArcAngleTraversed = arcAngleTraversed;

            if (driveTurnControl.finished(totalArcAngleTraversed)) break;

            float distanceFromCenter = (float)Math.sqrt( diffX * diffX + diffY * diffY);

            float error = distanceFromCenter - circle.radius;

            float vx = -speed * diffY / distanceFromCenter - DISTANCE_CORRECTION_FACTOR * error * diffX / distanceFromCenter;
            float vy = speed * diffX / distanceFromCenter - DISTANCE_CORRECTION_FACTOR * error * diffY / distanceFromCenter;

            float sin = (float)Math.sin(heading);
            float cos = (float)Math.cos(heading);

            float vxRobot = vx * sin - vy * cos;
            float vyRobot = vx * cos + vy * sin;

            float targetHeadingDegrees = driveTurnControl.targetHeadingDegrees(totalArcAngleTraversed);
            float targetHeadingRadians = targetHeadingDegrees * (float)Math.PI / 180.0f;

            if (firstPass){
                firstPass = false;
                rateOfTargetHeadingChange = 0;
                prevTargetHeadingRadians = targetHeadingRadians;
            } else {
                rateOfTargetHeadingChange = (float)VuforiaNavigator.NormalizeAngle(targetHeadingRadians - prevTargetHeadingRadians) / elapsedSeconds;
                prevTargetHeadingRadians = targetHeadingRadians;
            }

            float headingErrorRadians = (float)VuforiaNavigator.NormalizeAngle(heading - targetHeadingRadians);
            float vaRobot = rateOfTargetHeadingChange - HEADING_CORRECTION_FACTOR * headingErrorRadians;

            bot.setDriveSpeed(vxRobot, vyRobot, vaRobot);

        }

        bot.setDrivePower(0,0,0);
        heading = bot.getHeadingRadians();
        robotXYTheta = bot.updateOdometry(robotXYTheta, heading);


    }

    protected void driveDirectionGyro(float speedCMs, float directionAngleDegrees, DriveTurnControl driveTurnControl) {
        float x0 = robotXYTheta[0];
        float y0 = robotXYTheta[1];

        float directionAngleRadians = directionAngleDegrees * (float) Math.PI / 180.0f;
        float headingTargetRadians = driveTurnControl.targetHeadingDegrees(0) * (float) Math.PI / 180.0f;

        boolean firstPass = true;
        float rateOfTargetHeadingChange = 0;
        float prevTargetHeadingRadians = 0;
        ElapsedTime et = new ElapsedTime();
        float prevSeconds = 0;
        float elapsedSeconds = 0;
        float prevX = robotXYTheta[0];
        float prevY = robotXYTheta[1];
        float prevHeading = bot.getHeadingRadians();
        float heading = prevHeading;

        while (opModeIsActive()) {
            float seconds = (float)et.seconds();
            elapsedSeconds = seconds - prevSeconds;
            prevSeconds = seconds;
            prevX = robotXYTheta[0];
            prevY = robotXYTheta[1];
            prevHeading = heading;
            heading = bot.getHeadingRadians();
            this.robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
            float distance = (float)Math.sqrt(Math.pow(robotXYTheta[0]-x0, 2) + Math.pow(robotXYTheta[1]-y0, 2));

            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "z = %.2f  x = %.2f  Phi = %.2f",
                        robotXYTheta[0], robotXYTheta[1], robotXYTheta[2] * 180.0 / Math.PI);

            if (driveTurnControl.finished(distance)) break;

            float targetHeadingDegrees = driveTurnControl.targetHeadingDegrees(distance);
            float targetHeadingRadians = targetHeadingDegrees * (float)Math.PI / 180.0f;

            if (firstPass){
                firstPass = false;
                prevTargetHeadingRadians = targetHeadingRadians;
            } else {
                rateOfTargetHeadingChange = (float)VuforiaNavigator.NormalizeAngle(targetHeadingRadians - prevTargetHeadingRadians) / elapsedSeconds;
                prevTargetHeadingRadians = targetHeadingRadians;
            }
            VectorF diff = new VectorF(robotXYTheta[0] - x0, robotXYTheta[1] - y0, 0);

            VectorF expectedVector = new VectorF((float)Math.cos(directionAngleRadians), (float)Math.sin(directionAngleRadians), 0);

            VectorF translationError = VectorMath.crossProduct(diff, expectedVector);
            translationError = VectorMath.crossProduct(expectedVector, translationError);


            float alpha = (float)Math.atan2(translationError.get(1), translationError.get(0));
            float errorX = translationError.magnitude() * (float)Math.sin(heading - alpha);
            float errorY = translationError.magnitude() * (float)Math.cos(heading - alpha);

            float vx = -speedCMs * (float) Math.sin(directionAngleRadians - heading) - DISTANCE_CORRECTION_FACTOR * errorX;
            float vy = speedCMs * (float) Math.cos(directionAngleRadians - heading) - DISTANCE_CORRECTION_FACTOR * errorY;

            float headingErrorRadians = (float)VuforiaNavigator.NormalizeAngle(heading - targetHeadingRadians);
            float va = rateOfTargetHeadingChange - HEADING_CORRECTION_FACTOR * headingErrorRadians;

            bot.setDriveSpeed(vx, vy, va);


        }
        bot.setDrivePower(0,0,0);
        heading = bot.getHeadingRadians();
        robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
    }


    /**
     * Reset odometry to new (x,y) field coordinates, with heading determined using gyro
     * @param x Current x-position (field coordinates)
     * @param y Current y-position (field coordinates)
     */
    protected void resetOdometry(float x, float y){
        robotXYTheta = new float[]{x, y, bot.getHeadingRadians()};
        bot.updateOdometry();
    }

    /**
     * Reset odometry, given new values of x, y, and heading.
     * This method might be useful if robot pose can be determined accurately using some method other
     * than odometry, or if the user wishes to switch to using a different coordinate system.
     * @param x Curent x-position (field coordinates)
     * @param y Current y-position (field coordinates)
     * @param headingDegrees Current heading in degrees
     */
    protected void resetOdometry(float x, float y, float headingDegrees){
        robotXYTheta = new float[]{x, y, headingDegrees * (float)Math.PI / 180.0f};
        bot.resetHeadingDegrees(headingDegrees);
        bot.updateOdometry();
    }

    protected void adjustHeading(float degreesToAdd){
        float gyroHeading = bot.getHeadingRadians();
        float gyroHeadingDegrees = gyroHeading * 180.0f / (float)Math.PI;
        float newHeadingDegrees = (float)VuforiaNavigator.NormalizeDegrees(gyroHeadingDegrees + degreesToAdd);
        resetOdometry(robotXYTheta[0], robotXYTheta[1], newHeadingDegrees);
        if (ADJUST_HEADING_LOG) {
            BetaLog.dd(ADJUST_HEADING_TAG,"gyroheadingDegrees = %.1f  newHeadingDegrees = %.1f",
                    gyroHeadingDegrees, newHeadingDegrees);
            BetaLog.dd(ADJUST_HEADING_TAG, "robotXYTheta = %.1f  %.1f  %.1f", robotXYTheta[0],
                    robotXYTheta[1], robotXYTheta[2] * 180.0 / Math.PI);
        }
    }

    protected boolean adjustHeadingWithWall (float baseHeadingDegrees, float defaultHeadingErrorDegrees, DistanceSensor sensor1, DistanceSensor sensor2,
                                             PointF sensor1Pos, PointF sensor2Pos) {

//        if (ADJUST_HEADING_WITH_WALL_LOG) BetaLog.dd(ADJUST_HEADING_WITH_WALL_TAG, "Entering adjustHeadingWithWall.");
//
//        float intersensordistance = (float) Math.sqrt((sensor2Pos.x - sensor1Pos.x) * (sensor2Pos.x - sensor1Pos.x)
//                + (sensor2Pos.y - sensor1Pos.y) * (sensor2Pos.y - sensor1Pos.y));
//
//        if (ADJUST_HEADING_WITH_WALL_LOG) BetaLog.dd(ADJUST_HEADING_WITH_WALL_TAG, "IntersensorDistance = %.1f", intersensordistance);
//
//        float alpha;
//        if (sensor1Pos.x >0){
//            alpha = (float)Math.asin((sensor1Pos.x - sensor2Pos.x)/intersensordistance);
//        }
//        else {
//            alpha = (float)Math.asin((sensor2Pos.x - sensor1Pos.x)/intersensordistance);
//        }
//
//        if (ADJUST_HEADING_WITH_WALL_LOG) BetaLog.dd(ADJUST_HEADING_WITH_WALL_TAG, "Alpha (degrees = %.1f",
//                alpha * 180.0/Math.PI);
//
//        float d1 = (float)sensor1.getDistance(DistanceUnit.CM);
//        float d2 = (float)sensor2.getDistance(DistanceUnit.CM);
//
//        if (ADJUST_HEADING_WITH_WALL_LOG) BetaLog.dd(ADJUST_HEADING_WITH_WALL_TAG, "d1 = %.1f  d2 = %.1f", d1, d2);
//
//        if (d1>60 || d2>60 || d1<5 || d2<5 || Math.abs(d2-d1)/intersensordistance>0.5){
//            float gyroHeading = bot.getHeadingRadians();
//            float gyroHeadingDegrees = gyroHeading*180.0f/(float)Math.PI;
//            float newHeadingDegrees = (float)VuforiaNavigator.NormalizeDegrees(gyroHeadingDegrees+defaultHeadingErrorDegrees);
//            resetOdometry(robotXYTheta[0], robotXYTheta[1], newHeadingDegrees);
//            if (ADJUST_HEADING_WITH_WALL_LOG){
//                BetaLog.dd(ADJUST_HEADING_WITH_WALL_TAG, "gyroHeadingDegrees = %.1f  newHeadingDegrees = %.1f",
//                        gyroHeadingDegrees, newHeadingDegrees);
//                BetaLog.dd(ADJUST_HEADING_WITH_WALL_TAG, "robotXYTheta = %.1f  %.1f  %.1f",
//                        robotXYTheta[0], robotXYTheta[1], robotXYTheta[2]*180.0/Math.PI);
//            }
//            return false;
//        }
//        else {
//            float headingError = (float)Math.asin((d2-d1)/intersensordistance) - alpha;
//            float headingErrorDegrees = headingError*180.0f/(float)Math.PI;
//            float newHeadingDegrees = (float)VuforiaNavigator.NormalizeDegrees(baseHeadingDegrees+headingErrorDegrees);
//            resetOdometry(robotXYTheta[0], robotXYTheta[1], newHeadingDegrees);
//            if (ADJUST_HEADING_WITH_WALL_LOG){
//                BetaLog.dd(ADJUST_HEADING_WITH_WALL_TAG, "headingErrorDegrees = %.1f  newHeadingDegrees = %.1f",
//                        headingErrorDegrees, newHeadingDegrees);
//                BetaLog.dd(ADJUST_HEADING_WITH_WALL_TAG, "robotXYTheta = %.1f  %.1f  %.1f",
//                        robotXYTheta[0], robotXYTheta[1], robotXYTheta[2]*180.0/Math.PI);
//            }
//            return true;
//        }
        float gyroHeading = bot.getHeadingRadians();
        float gyroHeadingDegrees = gyroHeading * 180.0f / (float)Math.PI;
        float newHeadingDegrees = (float)VuforiaNavigator.NormalizeDegrees(gyroHeadingDegrees + defaultHeadingErrorDegrees);
        resetOdometry(robotXYTheta[0], robotXYTheta[1], newHeadingDegrees);
        if (ADJUST_HEADING_WITH_WALL_LOG) {
            BetaLog.dd(ADJUST_HEADING_WITH_WALL_TAG,"gyroheadingDegrees = %.1f  newHeadingDegrees = %.1f",
                    gyroHeadingDegrees, newHeadingDegrees);
            BetaLog.dd(ADJUST_HEADING_WITH_WALL_TAG, "robotXYTheta = %.1f  %.1f  %.1f", robotXYTheta[0],
                    robotXYTheta[1], robotXYTheta[2] * 180.0 / Math.PI);
        }

        return false;
    }
}
