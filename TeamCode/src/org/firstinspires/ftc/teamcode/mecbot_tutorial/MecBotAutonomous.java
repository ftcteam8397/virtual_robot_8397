package org.firstinspires.ftc.teamcode.mecbot_tutorial;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.AngleUtils;

/**
 * An abstract class that extends LinearOpMode and provides navigation methods that can be called by autonomous op modes
 * that utilize a MechBot object.
 */
public abstract class MecBotAutonomous extends LinearOpMode {

    /*
     * The MechBot object that will be used by the navigation methods in this class. This must be assigned a value
     * by calling the setBot method.
     */
    MecBot bot;

    public interface Predicate {
        boolean isTrue();
    }



    /**
     * runOpMode is declared "abstract", so we won't actually provide code for it. The code must be provided
     * by classes that extend MechBotAutonomous.
     *
     * TODO: We can delete this. runOpMode is already declared as an abstract method in LinearOpMode; that doesn't need to be repeated here.
     */
    public abstract void runOpMode();

    /**
     * Assign a MechBot object to bot. This object should already be initialized before being provided.
     * @param b
     */
    public void setBot(MecBot b){
        bot = b;
    }

    /**
     * Drive the robot straight in the specified direction, at the specified speed, while maintaining a specified
     * orientation, until it has travelled the specified number of inches.
     *
     * @param speed                     Speed, in inches per second.
     * @param directionDegrees          Direction of travel (world coordinates), in degrees
     * @param targetHeadingDegrees      Target orientation (world coordinates), in degrees
     * @param finished                  Predicate which tells the bot when to stop
     */
    public void driveStraight(float speed, float directionDegrees,
                              float targetHeadingDegrees, Predicate finished){

        /*
         * Get the robot's starting coordinates, so we can keep track of how far we have travelled.
         */
        float xStart = bot.pose.x;
        float yStart = bot.pose.y;

        float directionRadians = (float)Math.toRadians(directionDegrees);
        float targetHeadingRadians = (float)Math.toRadians(targetHeadingDegrees);

        /*
         * Control loop for this operation. Break from the loop after the specified distance has been travelled.
         */
        while (opModeIsActive()){
            bot.updateOdometry();                                               //Determine current bot position             //Distance travelled (inches)
            if (finished.isTrue()) {
                break;                                                //Break from loop if we've travelled far enouch
            }

            //TODO: Update robot drive speed based on current position and orientation.

            float vx = -speed * (float)Math.sin(directionRadians - bot.pose.theta);
            float vy = speed * (float)Math.cos(directionRadians - bot.pose.theta);

            float angleOffset = (float)AngleUtils.normalizeRadians(targetHeadingRadians - bot.pose.theta);
            float va = 0.2f * angleOffset;

            bot.setDriveSpeed(vx, vy, va);
        }
        // We have travelled far enough and broken from loop, so stop the robot.
        bot.setDrivePower(0, 0, 0);
    }

    public void turnToHeading(float targetHeadingDegrees, float toleranceDegrees,
                              float propCoeff) {
        float targetHeadingRadians = targetHeadingDegrees * (float)Math.PI / 180;
        float toleranceRadians = toleranceDegrees * (float)Math.PI / 180;
        while(opModeIsActive()) {
            bot.updateOdometry();
            float currentHeading = bot.pose.theta;
            float angleDiff = (float)AngleUtils.normalizeRadians(targetHeadingRadians - currentHeading);
            if(Math.abs(angleDiff) < toleranceRadians) {
                break;
            } else {
                bot.setDriveSpeed(0, 0, propCoeff * angleDiff);
            }
        }
        bot.setDrivePower(0, 0,0);
    }


}
