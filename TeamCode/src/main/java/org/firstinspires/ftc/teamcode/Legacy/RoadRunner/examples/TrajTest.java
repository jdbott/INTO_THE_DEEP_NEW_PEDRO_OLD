package org.firstinspires.ftc.teamcode.Legacy.RoadRunner.examples;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Legacy.RoadRunner.follower.MecanumDriveJasonChassis;

@TeleOp
public class TrajTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDriveJasonChassis drive = new MecanumDriveJasonChassis(hardwareMap, new Pose2d(15, -63, Math.toRadians(90)));

        Action traj = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(15, -30))
                .turnTo(Math.toRadians(180))
                .build();

        Action traj2 = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(0, -30), Math.toRadians(180))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                traj2
        );
    }
}