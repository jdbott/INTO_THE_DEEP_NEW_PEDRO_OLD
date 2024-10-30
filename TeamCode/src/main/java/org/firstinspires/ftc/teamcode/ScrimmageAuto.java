package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClasses.DepositGripper;
import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlide;
import org.firstinspires.ftc.teamcode.HardwareClasses.Pivot;
import org.firstinspires.ftc.teamcode.PedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.PedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.PedroPathing.util.Timer;

@Autonomous(name = "Scrimmage Auto", group = "Pedro Auto")
public class ScrimmageAuto extends OpMode {

    // Initialize path following stuff
    private Follower follower;
    private Path toChamber, toBasket, toSpike1, toSpike2, toSpike3;
    private Timer pathTimer;
    private int pathState;
    private int times;

    // Initialize telemetry and any other subsystems and variables
    private Telemetry telemetryA;

    public DepositGripper gripper;
    public LinearSlide linearSlide;
    public Pivot pivot;

    @Override
    public void init() {
        // Initialize path stuff with hardwareMap
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(-36, -63.5, Math.toRadians(90)));
        follower.setMaxPower(0.5);
        pathTimer = new Timer();
        pathState = 0;

        gripper = new DepositGripper(hardwareMap);
        pivot = new Pivot(hardwareMap);

        String[] motorNames = {"slideMotor"};
        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.REVERSE};
        linearSlide = new LinearSlide(hardwareMap, motorNames, directions, 81.5625, 0, 38); // Example ticksPerInch and limits

        gripper.CloseGripper();
        gripper.grabSpecimen();

        // Initialize telemetry
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetryA.addData("Status:", "initialized");
        telemetryA.update();
    }

    @Override
    public void init_loop() {
        pivot.movePivotToAngle(90);
        pivot.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autoPathUpdate();
        follower.telemetryDebug(telemetryA);
        pivot.update();
        linearSlide.update();
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                toChamber = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(0, -50, Point.CARTESIAN)));
                toChamber.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(toChamber, true);
                pivot.movePivotToAngle(40);
                linearSlide.moveSlidesToPositionInches(15);
                gripper.placeSpecimen();
                //Move lift to pos to score specimen
                setPathState(1);
                times = 0;
                break;

            case 1:
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(1);
                        times = 1;
                    }
                    gripper.placeSpecimenFully();
                    linearSlide.moveSlidesToPositionInches(8);

                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        gripper.OpenGripper();
                        setPathState(2);
                    }
                }
                break;

            case 2:
                //go to grab samples and move lift to grab them
                pivot.movePivotToAngle(0);
                linearSlide.moveSlidesToPositionInches(2);
                gripper.grabSpecimen();
                break;

            case 3:
                if (!follower.isBusy()) {
                    // grab sample
                    setPathState(4);
                }
                break;

            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(0);
                }
                break;

            default:
                // No further action
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }
}