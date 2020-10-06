package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.mechbot.MechBot;
import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;
import org.firstinspires.ftc.teamcode.util.CubicSpline;

@Autonomous(name = "DriveSpline", group="MechBot")
public class DriveSpline extends MechBotAutonomous {

    public MechBot bot = new MechBot(MechBot.MotorType.NeverestOrbital20, 31.92, 35.6, 40.6);

    public void runLoggingOpmode(){
        bot.init(hardwareMap, 0, true);
        super.setBot(bot);

        /**
         * Create the cubic spline by supplying an array of points (x0, y0, x1, y1, x2, y2, ...) and a
         * beginning and ending travel direction (in degrees).
         *
         * Field coordinates: (0,0) is center of field. X-axis is right; Y-axis is up. Each square is 61 cm wide.
         *
         * Start robot at lower left intersection of squares (-122, -122), facing up (heading = 90 degrees)
         */

        float[] points = new float[]{-122, -122, -61, 61, 0, 0, 61, -61, 122, 122};
        CubicSpline spline = new CubicSpline(points, 90, 90);

        waitForStart();

        /**
         * Must reset odometry to indicate starting robot position and heading.
         */
        resetOdometry(-122, -122, 90);

        driveSpline(60, false, spline);
    }
}
