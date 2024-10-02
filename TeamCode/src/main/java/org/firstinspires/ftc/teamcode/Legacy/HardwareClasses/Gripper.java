package org.firstinspires.ftc.teamcode.Legacy.HardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {
    private final Servo gripperServo;

    public Gripper(HardwareMap hardwareMap) {
        gripperServo = hardwareMap.get(Servo.class, "gripperServo");
    }

    public void CloseGripper() {
        gripperServo.setPosition(0.5);
    }

    public void OpenGripper() {
        gripperServo.setPosition(0.2);
    }
}