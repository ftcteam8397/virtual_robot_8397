package org.firstinspires.ftc.teamcode.mecbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmRobot extends MecBot {
   public DcMotor arm;
   public Servo grabber;

   @Override
    public void init (HardwareMap hwMap) {
        super.init(hwMap);
        arm = hwMap.get(DcMotor.class, "arm_motor");
        grabber = hwMap.get(Servo.class, "hand_servo")
    }
}
