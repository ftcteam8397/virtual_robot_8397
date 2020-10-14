package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;

@Autonomous(name="StateMachine", group="tutorial")
public class StateMachine extends OpMode {
    MecBot bot;

    State state;

    enum State {
        DRIVE_FORWARD,
        TURN,
        DRIVE_ARC,
        FINISHED
    }

    public void init() {
        bot = new MecBot();
        bot.init(hardwareMap);
        state = State.DRIVE_FORWARD;
    }

    public State driveForward() {
        if (bot.getPose().x < 50) {
            bot.setDrivePower(1, 0, 0);
            return State.DRIVE_FORWARD;
        } else {
            return State.TURN;
        }
    }

    public State turn() {
        if (bot.getPose().theta < Math.PI / 2) {
            bot.setDrivePower(0, 1, 0);
            return State.TURN;
        } else {
            return State.FINISHED;
        }
    }

    public void loop() {
        bot.updateOdometry();
        switch (state) {
            case DRIVE_FORWARD:
                state = driveForward();
                break;
            case TURN:
                state = turn();
                break;
            case FINISHED:
                bot.setDrivePower(0, 0, 0);
                break;
        }
    }

}
