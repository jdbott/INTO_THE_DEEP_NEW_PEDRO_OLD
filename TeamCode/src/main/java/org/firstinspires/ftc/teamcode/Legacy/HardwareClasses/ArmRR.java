package org.firstinspires.ftc.teamcode.Legacy.HardwareClasses;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class ArmRR {
    private final DcMotorEx armMotor;

    double lastSetArmPos = 0;
    boolean armMotorBusy = false;

    public ArmRR(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Action moveArmToPosition(double targetPos) {
        return packet -> {
            lastSetArmPos = targetPos;

            double error = targetPos - armMotor.getCurrentPosition();

            if (error > 20) {
                armMotor.setPower(Range.clip((error * 0.005), -1, 1));
                packet.put("Moving towards target position, the error is", error);
                armMotorBusy = true;
                return true;
            } else {
                armMotor.setPower(0);
                packet.put("At target position, stopping. The error is", error);
                armMotorBusy = false;
                return false;
            }
        };
    }

    public Action correctArmPosition() {
        return packet -> {
            if (!armMotorBusy) {
                double error = lastSetArmPos - armMotor.getCurrentPosition();
                armMotor.setPower(Range.clip((error * 0.005), -0.5, 0.5));
                packet.put("Correcting towards target position, the error is", error);
            }
            return true;
        };
    }
}