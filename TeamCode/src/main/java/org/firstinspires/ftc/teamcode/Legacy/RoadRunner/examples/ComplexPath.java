package org.firstinspires.ftc.teamcode.Legacy.RoadRunner.examples;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Legacy.RoadRunner.follower.MecanumDriveJasonChassis;

@TeleOp (name = "Complex Path")
public final class ComplexPath extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDriveJasonChassis drive = new MecanumDriveJasonChassis(hardwareMap, new Pose2d(60, -60, -90));

        waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(0, -60))
                            .setTangent(Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-24, -36, Math.toRadians(180)), Math.toRadians(90))
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(0, -12, Math.toRadians(-90)), Math.toRadians(0))
                            .setTangent(Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(24, -36, Math.toRadians(0)), Math.toRadians(-90))
                            .setTangent(Math.toRadians(-90))
                            .splineToLinearHeading(new Pose2d(0, -60, Math.toRadians(90)), Math.toRadians(180))
                            .build());
    }
}