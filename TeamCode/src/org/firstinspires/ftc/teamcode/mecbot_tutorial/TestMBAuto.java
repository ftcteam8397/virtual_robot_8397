package org.firstinspires.ftc.teamcode.mecbot_tutorial;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.mecbot_tutorial.MecBotAutonomous;

@Autonomous(name = "Test MB Auto", group = "tutorial")
public class TestMBAuto extends MecBotAutonomous{

    MecBot bot = new MecBot();

    public void runOpMode(){
        bot.init(hardwareMap);
        super.setBot(bot);

        waitForStart();

        driveStraight(20, 22, 10,
                new Predicate() {
                    @Override
                    public boolean isTrue() {
                        return bot.pose.x > 48;
                    }
                });
        turnToHeading(90, 0.1f, 1);
    }

}
