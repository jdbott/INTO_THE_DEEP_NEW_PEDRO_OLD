package org.firstinspires.ftc.teamcode.Legacy.HardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {
    private final Servo leftGripperServo;
    private final Servo rightGripperServo;

    public Gripper(HardwareMap hardwareMap) {
        leftGripperServo = hardwareMap.get(Servo.class, "leftGripperServo");
        rightGripperServo = hardwareMap.get(Servo.class, "rightGripperServo");
    }

    public void OpenGripper() {
        leftGripperServo.setPosition(0.4);
        rightGripperServo.setPosition(0.6);
    }

    public void CloseGripper() {
        leftGripperServo.setPosition(0.7);
        rightGripperServo.setPosition(0.3);
    }
}