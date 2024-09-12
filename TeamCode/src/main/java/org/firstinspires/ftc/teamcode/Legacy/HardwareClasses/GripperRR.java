package org.firstinspires.ftc.teamcode.Legacy.HardwareClasses;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GripperRR {
    private final Servo leftGripperServo;
    private final Servo rightGripperServo;

    public GripperRR(HardwareMap hardwareMap) {
        leftGripperServo = hardwareMap.get(Servo.class, "leftGripperServo");
        rightGripperServo = hardwareMap.get(Servo.class, "rightGripperServo");
    }

    public Action OpenGripper() {
        return packet -> {
            leftGripperServo.setPosition(0.4);
            rightGripperServo.setPosition(0.6);
            packet.put("Opening gripper", "");
            return false;
        };
    }

    public Action CloseGripper() {
        return packet -> {
            leftGripperServo.setPosition(0.7);
            rightGripperServo.setPosition(0.3);
            packet.put("Closing gripper", "");
            return false;
        };
    }
}