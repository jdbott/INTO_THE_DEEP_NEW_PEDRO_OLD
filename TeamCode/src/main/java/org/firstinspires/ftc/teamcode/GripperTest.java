package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareClasses.ColorV3;
import org.firstinspires.ftc.teamcode.Legacy.HardwareClasses.Gripper;

@TeleOp
public class GripperTest extends LinearOpMode {
    private Gripper gripper;

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        gripper = new Gripper(hardwareMap);

        gripper.CloseGripper();

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, check proximity and detect color
        while (opModeIsActive()) {
            if (gamepad1.a) {
                gripper.OpenGripper();
            } else if (gamepad1.b) {
                gripper.CloseGripper();
            }
        }
    }
}