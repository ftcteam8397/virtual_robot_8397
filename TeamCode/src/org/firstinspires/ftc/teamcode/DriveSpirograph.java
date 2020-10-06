package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.mechbot.MechBot;
import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;
import org.firstinspires.ftc.teamcode.util.ParametricFunction;

@Autonomous(name="DriveSpirograph", group="MechBot")
public class DriveSpirograph extends MechBotAutonomous {
    public MechBot bot = new MechBot(MechBot.MotorType.NeverestOrbital20, 32.75, 31.75, 31.75);
    float r1 = 50;
    float r2 = 75;
    float p = 16;

    @Override
    public void runLoggingOpmode() {
        bot.init(hardwareMap, 0, true);
        super.setBot(bot);
        waitForStart();
        resetOdometry(r1+r2, 0, 90);
        driveFunction(100, 0, new ParametricFunction() {
            @Override
            public VectorF p(float s) {
                return new VectorF(r1*(float)Math.cos(s) + r2*(float)Math.cos(p*s), r1*(float)Math.sin(s) + r2*(float)Math.sin(p*s));
            }

            @Override
            public VectorF d1(float s) {
                return new VectorF(-r1*(float)Math.sin(s) - p*r2*(float)Math.sin(p*s), r1*(float)Math.cos(s) + p*r2*(float)Math.cos(p*s));
            }

            @Override
            public VectorF d2(float s) {
                return new VectorF(-r1*(float)Math.cos(s) - p*p*r2*(float)Math.cos(p*s), -r1*(float)Math.sin(s) - p*p*r2*(float)Math.sin(p*s));
            }
        }, new Predicate() {
            @Override
            public boolean isTrue() {
                return false;
            }
        });
    }
}
