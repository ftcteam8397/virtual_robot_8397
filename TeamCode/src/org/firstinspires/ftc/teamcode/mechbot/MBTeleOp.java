package org.firstinspires.ftc.teamcode.mechbot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="MBTeleOp", group="MechBot")
public class MBTeleOp extends MechBotTeleOp {
    public MechBot bot = new MechBot(MechBot.MotorType.NeverestOrbital20, 32.75, 31.75, 31.75);

    @Override
    public void runLoggingOpmode() {
        bot.init(hardwareMap, 0, true);
        super.setup(bot);
        waitForStart();
        resetOdometry(0,0,0);
        while(opModeIsActive()) {
            float heading = bot.getHeadingRadians();
            robotXYTheta = bot.updateOdometry(robotXYTheta, heading);
            System.out.println("x="+robotXYTheta[0]+"   y="+robotXYTheta[1]);
            doDriveControl();
            telemetry.update();
        }
    }
}
