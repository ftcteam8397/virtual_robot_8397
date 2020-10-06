/**
 * Diagnostic OpMode for use with our 1st MechBot (with Neverest 40s, gear ratio 1.0, and Rev Hub mounted horizontally.
 */

package org.firstinspires.ftc.teamcode.mechbot.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.mechbot.MechBot;
import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;

@TeleOp(name = "MechBotDiagnostics1", group = "Test")
@Disabled
public class MechBotDiagnostics1 extends MechBotAutonomous {

    private MechBot bot = new MechBot(MechBot.MotorType.Neverest40, 31.5, 31.75, 31.75  );
    boolean startPressed = false;

    @Override
    public void runLoggingOpmode() throws InterruptedException {

        bot.init(hardwareMap, 0, true);
        super.setBot(bot);

        robotXYTheta = new float[]{0,0,0};

        waitForStart();

        while (opModeIsActive()){

            float heading = bot.getHeadingRadians();
            robotXYTheta = bot.updateOdometry(robotXYTheta, heading);

            if (gamepad1.start) startPressed = true;
            else if (startPressed){
                startPressed = false;
                resetOdometry(0, 0);
            }

            if (gamepad1.a) handleMotorPowers(0.2f,0,0,0);
            else if (gamepad1.x) handleMotorPowers(0,0.2f,0,0);
            else if (gamepad1.y) handleMotorPowers(0,0,0.2f,0);
            else if (gamepad1.b) handleMotorPowers(0,0,0,0.2f);
            else if (gamepad1.dpad_right) handleBotPowers(0.2f, 0, 0);
            else if (gamepad1.dpad_up) handleBotPowers(0, 0.2f, 0);
            else if (gamepad1.dpad_down) handleBotPowers(0, 0, 0.2f);
            else {
                float px = gamepad1.left_stick_x;
                float py = -gamepad1.left_stick_y;
                float pa = -gamepad1.right_stick_x;
                if (Math.abs(px) < 0.05) px = 0;
                if (Math.abs(py) < 0.05) py = 0;
                if (Math.abs(pa) < 0.05) pa = 0;
                handleBotPowers(px, py, pa);
            }

            telemetry.addData("Heading", "%.1f Degrees", heading * 180.0 / Math.PI);
            telemetry.addData("Odometry", "X = %.1f  Y = %.1f  Theta = %.1f",
                    robotXYTheta[0], robotXYTheta[1], robotXYTheta[2]*180.0 / Math.PI);
            telemetry.addData("Encoders", "m1 %d  m2 %d  m3 %d  m4 %d",
                    bot.one.getCurrentPosition(), bot.two.getCurrentPosition(),
                    bot.three.getCurrentPosition(), bot.four.getCurrentPosition());

            telemetry.update();

        }



    }

    private void handleBotPowers(float px, float py, float pa){
        bot.setDrivePower(px, py, pa);
        telemetry.addData("Robot Power", "PX = %.3f  PY = %.3f  PA = %.3f", px, py, pa);
    }

    private void handleMotorPowers(float p1, float p2, float p3, float p4){
        bot.one.setPower(p1);
        bot.two.setPower(p2);
        bot.three.setPower(p3);
        bot.four.setPower(p4);
        telemetry.addData("Motor Powers", "1: %.3f  2: %.3f  3: %.3f  4: %.3f", p1, p2, p3, p4);
    }

}
