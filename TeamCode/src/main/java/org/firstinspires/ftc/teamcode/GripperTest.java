package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareClasses.ColorV3;
import org.firstinspires.ftc.teamcode.HardwareClasses.DepositGripper;
import org.firstinspires.ftc.teamcode.Legacy.HardwareClasses.Gripper;

@TeleOp
public class GripperTest extends LinearOpMode {
    private DepositGripper gripper;

    private boolean isGripperOpen = false;
    private boolean toggleA = false;
    private boolean actionState = false;
    private boolean toggleB = false;

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        gripper = new DepositGripper(hardwareMap);

        gripper.CloseGripper();
        gripper.gripperGrabSpecimen();

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, check proximity and detect color
        while (opModeIsActive()) {
            // Toggle gripper open/close using A button
            if (gamepad1.a && !toggleA) {
                isGripperOpen = !isGripperOpen;
                toggleA = true;
                if (isGripperOpen) {
                    gripper.OpenGripper();
                } else {
                    gripper.CloseGripper();
                }
            } else if (!gamepad1.a) {
                toggleA = false;
            }

            // Toggle between other two actions using B button
            if (gamepad1.b && !toggleB) {
                actionState = !actionState;
                toggleB = true;
                if (actionState) {
                    gripper.gripperGrabSpecimen();
                } else {
                    gripper.gripperPlaceSpecimen();
                }
            } else if (!gamepad1.b) {
                toggleB = false;
            }

            if (gamepad1.x) {
                gripper.gripperTransferSample();
            }
        }
    }
}