package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.HardwareClasses.DepositGripper;
import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlide;
import org.firstinspires.ftc.teamcode.HardwareClasses.Pivot;
import org.firstinspires.ftc.teamcode.PedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.PedroPathing.localization.Pose;

@TeleOp
public class GripperTest extends LinearOpMode {
    private DepositGripper gripper;
    private LinearSlide linearSlide;
    private Follower follower;
    private Pivot pivot;

    private boolean isGripperOpen = false;
    private boolean toggleA = false;
    private boolean actionState = false;
    private boolean toggleB = false;

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        gripper = new DepositGripper(hardwareMap);
        pivot = new Pivot(hardwareMap);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));

        String[] motorNames = {"slideMotor"};
        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.REVERSE};
        LinearSlide linearSlide = new LinearSlide(hardwareMap, motorNames, directions, 81.5625, 0, 38); // Example ticksPerInch and limits

        gripper.CloseGripper();
        gripper.grabSpecimen();
        follower.startTeleopDrive();

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, check proximity and detect color
        while (opModeIsActive()) {
            follower.setTeleOpMovementVectors(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );

            if (gamepad1.dpad_left) {
                pivot.movePivotToAngle(15);
                linearSlide.moveSlidesToPositionInches(19);
                gripper.placeSample();
            } else if (gamepad1.dpad_right) {
                pivot.movePivotToAngle(0);
                linearSlide.moveSlidesToPositionInches(2);
                gripper.grabSpecimen();
            } else if (gamepad1.dpad_up) {
                pivot.movePivotToAngle(50);
                linearSlide.moveSlidesToPositionInches(15);
                gripper.placeSpecimen();
            }
            
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

            linearSlide.update();
            pivot.update();
            follower.update();
        }
    }
}