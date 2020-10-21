package org.firstinspires.ftc.teamcode.mecbot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "Arm Demo", group = "tutorial")
public class ArmDemo extends MecBotAutonomous {
    ArmBot armBot;

    public void runOpMode() {
        bot = new ArmRobot();
        bot.grabber.setPosition(1);
        bot.init (hardwareMap);
        super.setBot(bot);
        bot.setPose(48, 9, 90);
        waitForStart();

        driveStraight(100, 102, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                return bot.getPose().y > 89;
            }
        });
        bot.arm.setPower(1);
        sleep(500);
        bot.arm.setPower(0);
        bot.grabber.setPosition(0);
        sleep(200);
        driveStraight(100, -30, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                return bot.getPose().y < 74;
            }
        });
    }
}
