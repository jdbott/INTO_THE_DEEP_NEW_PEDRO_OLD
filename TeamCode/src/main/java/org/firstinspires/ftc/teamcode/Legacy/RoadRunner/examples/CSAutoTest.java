package org.firstinspires.ftc.teamcode.Legacy.RoadRunner.examples;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Legacy.RoadRunner.follower.MecanumDriveJasonChassis;

@Autonomous(name = "Centerstage Auto Test")
public class CSAutoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDriveJasonChassis drive = new MecanumDriveJasonChassis(hardwareMap, new Pose2d(15, -63, Math.toRadians(90)));

        Action trajectoryParkCorner;
        Action trajectoryParkCenter;
        Action trajectoryLeftBackdrop;
        Action trajectoryLeftAudience;
        Action trajectoryLeftBackdropStageDoorPlus2;
        Action trajectoryLeftBackdropStageDoorPlus4;
        Action trajectoryLeftBackdropStageDoorPlus6;
        Action trajectoryCenterBackdrop;
        Action trajectoryCenterAudience;
        Action trajectoryCenterBackdropStageDoorPlus2;
        Action trajectoryCenterBackdropStageDoorPlus4;
        Action trajectoryCenterBackdropStageDoorPlus6;
        Action trajectoryRightBackdrop;
        Action trajectoryRightAudience;
        Action trajectoryRightBackdropStageDoorPlus2;
        Action trajectoryRightBackdropStageDoorPlus4;
        Action trajectoryRightBackdropStageDoorPlus6;
        Action desiredTrajectory = null;

        String desiredTrajectoryTelemetry = "Choose Auto";

        trajectoryParkCorner = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(45.4, -60.8), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(62, -60.8), Math.toRadians(0))
                .build();

        trajectoryParkCenter = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-41, -35, Math.toRadians(180)), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-35, -60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(32, -60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45.4, -29.6), Math.toRadians(0))
                .setTangent(Math.toRadians(-135))
                .splineToConstantHeading(new Vector2d(32, -60), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-35, -60), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-58.8, -35.1), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-35, -60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(32, -60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45.4, -43.2), Math.toRadians(0))
                .setTangent(Math.toRadians(-135))
                .splineToConstantHeading(new Vector2d(32, -60), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-35, -60), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-58.8, -35.1), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-35, -60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(32, -60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45.4, -43.2), Math.toRadians(0))
                .setTangent(Math.toRadians(-135))
                .splineToConstantHeading(new Vector2d(32, -60), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-35, -60), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-58.8, -24.1), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-35, -60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(32, -60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45.4, -43.2), Math.toRadians(0))
                .build();

        trajectoryLeftBackdrop = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(8.3, -37.3), Math.toRadians(136.7))
                .strafeToSplineHeading(new Vector2d(50, -33), Math.toRadians(180))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(40, -52), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(57.8, -63), Math.toRadians(180))
                .build();

        trajectoryLeftBackdropStageDoorPlus2 = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(7, -35, Math.toRadians(180)), Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(45.4, -29.6), Math.toRadians(180))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-58.8, -11.8), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45.4, -29.6), Math.toRadians(-45))
                .build();

        trajectoryLeftBackdropStageDoorPlus4 = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(7, -35, Math.toRadians(180)), Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(45.4, -29.6), Math.toRadians(180))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-58.8, -11.8), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45.4, -29.6), Math.toRadians(-45))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-58.8, -11.8), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45.4, -29.6), Math.toRadians(-45))
                .build();

        trajectoryLeftBackdropStageDoorPlus6 = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(7, -35, Math.toRadians(180)), Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(45.4, -29.6), Math.toRadians(180))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-58.8, -11.8), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45.4, -29.6), Math.toRadians(-45))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-58.8, -11.8), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45.4, -29.6), Math.toRadians(-45))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-25, -11.8), Math.toRadians(180))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-58.8, -24.1), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-25, -11.8), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45.4, -29.6), Math.toRadians(-45))
                .build();

        trajectoryCenterBackdrop = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(12, -30))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(45.4, -36.4, Math.toRadians(180)), Math.toRadians(0))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(45.4, -60.8), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(62, -60.8), Math.toRadians(0))
                .build();

        trajectoryCenterBackdropStageDoorPlus6 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(12, -30))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(45.4, -36.4, Math.toRadians(180)), Math.toRadians(0))
                .setTangent(Math.toRadians(115))
                .splineToConstantHeading(new Vector2d(-58.8, -11.8), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45.4, -36.4), Math.toRadians(-65))
                .setTangent(Math.toRadians(115))
                .splineToConstantHeading(new Vector2d(-58.8, -11.8), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45.4, -36.4), Math.toRadians(-65))
                .setTangent(Math.toRadians(105))
                .splineToConstantHeading(new Vector2d(-25, -11.8), Math.toRadians(180))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-58.8, -24.1), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-25, -11.8), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45.4, -36.4), Math.toRadians(-75))
                .build();

        trajectoryRightBackdrop = drive.actionBuilder(drive.pose)
//                .lineToX(-80)
//                .lineToX(0)
//                .lineToX(-80)
//                .lineToX(0)
//                .lineToX(-80)
//                .lineToX(0)
//                .lineToX(-80)
//                .lineToX(0)
//                .lineToX(-80)
//                .lineToX(0)
//                .lineToX(-80)
//                .lineToX(0)
                .build();

        trajectoryRightBackdropStageDoorPlus6 = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(29, -35, Math.toRadians(180)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(45.4, -43.2), Math.toRadians(180))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-58.8, -11.8), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45.4, -43.2), Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-58.8, -11.8), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45.4, -43.2), Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-25, -11.8), Math.toRadians(180))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-58.8, -24.1), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-25, -11.8), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45.4, -43.2), Math.toRadians(-90))
                .build();

        while (opModeInInit()) {
            telemetry.addData("Choose Auto", desiredTrajectoryTelemetry);
            telemetry.update();

            if (gamepad1.x) {
                desiredTrajectory = trajectoryLeftBackdrop;
                desiredTrajectoryTelemetry = "trajectoryLeftBackdrop";
            } else if (gamepad1.y) {
                desiredTrajectory = trajectoryLeftBackdropStageDoorPlus6;
                desiredTrajectoryTelemetry = "trajectoryLeftBackdropStageDoorPlus6";
            } else if (gamepad1.a) {
                desiredTrajectory = trajectoryCenterBackdrop;
                desiredTrajectoryTelemetry = "trajectoryCenterBackdrop";
            } else if (gamepad1.b) {
                desiredTrajectory = trajectoryCenterBackdropStageDoorPlus6;
                desiredTrajectoryTelemetry = "trajectoryCenterBackdropStageDoorPlus6";
            } else if (gamepad1.dpad_down) {
                desiredTrajectory = trajectoryRightBackdrop;
                desiredTrajectoryTelemetry = "trajectoryRightBackdrop";
            } else if (gamepad1.dpad_right) {
                desiredTrajectory = trajectoryRightBackdropStageDoorPlus6;
                desiredTrajectoryTelemetry = "trajectoryRightBackdropStageDoorPlus6";
            }

            sleep(50);
        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        desiredTrajectory
                )
        );
    }
}