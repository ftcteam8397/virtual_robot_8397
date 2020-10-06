package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.mechbot.MechBot;
import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;
import org.firstinspires.ftc.teamcode.util.ParametricFunction;

@Autonomous(name="DriveLissajous", group="MechBot")
public class DriveLissajous extends MechBotAutonomous {
    public MechBot bot = new MechBot(MechBot.MotorType.NeverestOrbital20, 32.75, 31.75, 31.75);

    @Override
    public void runLoggingOpmode() {
        bot.init(hardwareMap, 0, true);
        super.setBot(bot);
        waitForStart();
        resetOdometry(100, 0, 90);
        driveFunction(100, 0, new ParametricFunction() {
            float kx = 5f;
            float ky = 4f;
            @Override
            public VectorF p(float s) {
                return new VectorF(100*(float)Math.cos(kx*s), 100*(float)Math.sin(ky*s));
            }

            @Override
            public VectorF d1(float s) {
                return new VectorF(-100*kx*(float)Math.sin(kx*s),  100*ky*(float)Math.cos(ky*s));
            }

            @Override
            public VectorF d2(float s) {
                return new VectorF(-100*kx*kx*(float)Math.cos(kx*s), -100*ky*ky*(float)Math.sin(ky*s));
            }
        }, new Predicate() {
            @Override
            public boolean isTrue() {
                return false;
            }
        });
    }
}
