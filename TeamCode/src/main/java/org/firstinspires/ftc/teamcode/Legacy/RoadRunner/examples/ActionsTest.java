package org.firstinspires.ftc.teamcode.Legacy.RoadRunner.examples;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Legacy.HardwareClasses.ArmRR;
import org.firstinspires.ftc.teamcode.Legacy.RoadRunner.follower.MecanumDriveJasonChassis;

@TeleOp
public class ActionsTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDriveJasonChassis drive = new MecanumDriveJasonChassis(hardwareMap, new Pose2d(60, -58.5, Math.toRadians(90)));
        ArmRR arm = new ArmRR(hardwareMap);
        RevBlinkinLedDriver lights = hardwareMap.get(RevBlinkinLedDriver.class, "led");

        Action traj = drive.actionBuilder(drive.pose)
                .afterDisp(20, new InstantAction(() -> lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA)))
                .splineTo(new Vector2d(30, -30), Math.toRadians(180))
                .afterDisp(5, arm.moveArmToPosition(500))
                .strafeTo(new Vector2d(55, -30))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        traj,
                        arm.correctArmPosition()
                )
        );
    }
}