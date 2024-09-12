package org.firstinspires.ftc.teamcode.Legacy.PedroPathingExamples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Legacy.HardwareClasses.Flag;
import org.firstinspires.ftc.teamcode.Legacy.HardwareClasses.Lights;
import org.firstinspires.ftc.teamcode.Legacy.HardwareClasses.OldRobotConstants;
import org.firstinspires.ftc.teamcode.Legacy.HardwareClasses.TouchSensor;
import org.firstinspires.ftc.teamcode.PedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.PedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.Point;

@Autonomous(name = "Pedro Pathing Actions Test 2", group = "Pedro Auto")
public final class PedroPathingActionsTest2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        final Point redBackdropStartPose = OldRobotConstants.RED_BACKDROP_START_POSE;
        final Point redToLeftSpikeMarkMiddlePose = OldRobotConstants.RED_TO_LEFT_SPIKE_MARK_MIDDLE_POSE;
        final Point redLeftSpikeMark = OldRobotConstants.RED_LEFT_SPIKE_MARK;
        final Point redLeftBackdrop = OldRobotConstants.RED_LEFT_BACKDROP;
        final Point redToCornerParkingMiddlePose = OldRobotConstants.RED_TO_CORNER_PARKING_MIDDLE_POSE;
        final Point redCornerParking = OldRobotConstants.RED_CORNER_PARKING;
        Pose currentPos;

        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(15, -63, Math.toRadians(90)));

        Lights lights = new Lights(hardwareMap);
        Flag flag = new Flag(hardwareMap);
        TouchSensor touchSensor = new TouchSensor(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
        flag.flagDown();

        waitForStart();

        timer.reset();

        currentPos = follower.getPose();

        Path purplePath = new Path(new BezierCurve(
                new Point(currentPos),
                redToLeftSpikeMarkMiddlePose,
                redLeftSpikeMark));

        purplePath.setLinearHeadingInterpolation(currentPos.getHeading(), Math.toRadians(180));

        follower.followPath(purplePath);

        while (follower.isBusy() && !isStopRequested() && opModeIsActive()) {
            if (follower.getCurrentTValue() > 0.5) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
            }
            follower.update();
        }

        currentPos = follower.getPose();

        Path yellowPath = new Path(new BezierLine(
                new Point(currentPos),
                redLeftBackdrop));

        yellowPath.setConstantHeadingInterpolation(Math.toRadians(180));

        follower.followPath(yellowPath, true);

        while (follower.isBusy() && !isStopRequested() && opModeIsActive()) {
            follower.update();
        }

        while (!touchSensor.getState() && !isStopRequested() && opModeIsActive()) {
            follower.update();
            telemetry.addLine("Waiting for touch sensor to be pressed");
            telemetry.update();
        }

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        flag.flagDown();

        currentPos = follower.getPose();

        Path parkInCorner = new Path(new BezierCurve(
                new Point(currentPos),
                redToCornerParkingMiddlePose,
                redCornerParking
        ));
        parkInCorner.setConstantHeadingInterpolation(Math.toRadians(180));

        follower.followPath(parkInCorner);

        while (follower.isBusy() && !isStopRequested() && opModeIsActive()) {
            follower.update();
        }

        requestOpModeStop();
    }
}