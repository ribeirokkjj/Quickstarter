package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.FlywheelRunCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeRunCommand;
import org.firstinspires.ftc.teamcode.commands.TurretTrackCommand;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * Autonomous using a manual state machine with the Command-based structure.
 */
@Autonomous(name = "Autonomous Front Blue")
public class CommandAutoFrontBlue extends CommandOpMode {

    private Robot robot;
    private Paths paths;
    private IntakeRunCommand intake;
    private boolean stateInit, arrived = false;
    private Timer pathTimer;
    private Timer teleopTimer;
    private int pathState = 0;
    private double startHeading = Math.toRadians(180);
    public Pose startPose = new Pose(33.7, 133, startHeading);
    private List<Double> routine;

    @Override
    public void initialize() {

        // Initialize robot, timer, and paths
        robot = new Robot(hardwareMap, startPose);
        pathTimer = new Timer();
        teleopTimer = new Timer();

        paths = new Paths(robot.follower);

        //Set alliance
        robot.setAlliance(Robot.Alliance.AUTO_BLUE);

        //Data saver
        routine = new ArrayList<>();

        // Deixa a flywheel ligada 100% do tempo
        robot.flywheel.setDefaultCommand(
                new FlywheelRunCommand(robot.flywheel, robot)
        );
        // Also explicitly schedule the flywheel command to ensure it starts immediately
        // (some command schedulers require an explicit schedule to kick off default-like behavior)
        telemetry.addData("Auto", "Scheduled FlywheelRunCommand");
        // Set up the turret to track the goal continuously
        robot.turret.setDefaultCommand(
                new TurretTrackCommand(
                        robot.turret,
                        robot.turret::getCurrentAngle
                )
        );
    }

    @Override
    public void run() {
        // Essential updates for robot sensors and commands
        robot.update();
        CommandScheduler.getInstance().run();

        // Run the manual state machine
        statePathUpdate();

        robot.flywheel.setVelocityForDistance(robot.getDistanceToGoal());
        robot.flywheel.setHoodPosition(0.8);
        Intake();
        endIntake();

        if (pathState == 7 || pathState == 8 || pathState == 9) {
            //File saver
            String routineString = routine.toString();
            routineString = routineString.substring(1, routineString.length() - 1);

            File file = AppUtil.getInstance().getSettingsFile("File.txt");
            ReadWriteFile.writeFile(file, routineString);

            routine.clear();
            routine.add(robot.getX());
            routine.add(robot.getY());
            routine.add(robot.getHeadingDegrees());
        }
        telemetry.addData("Saved itens", routine);


        // Telemetry
        telemetry.addData("State", pathState);
        telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("X LL", robot.getCamX());
        telemetry.addData("Y LL", robot.getCamY());
        telemetry.addData("X", robot.follower.getPose().getX());
        telemetry.addData("Y", robot.follower.getPose().getY());
        telemetry.addData("Heading", robot.getHeadingDegrees());
        telemetry.addData("Turret Angle", robot.turret.getCurrentAngle());
        telemetry.addData("error", robot.flywheel.getVelocityError());
        telemetry.addData("Distance to Goal", robot.getDistanceToGoal());
        telemetry.update();
    }

    public void statePathUpdate() {

    }

    public void setPathState(int newState) {
        pathState = newState;
        stateInit = false;
        arrived = false;
        pathTimer.resetTimer();
    }

    // liga (chama quando apertar botão)

    /**
     * Lógica inteligente de tiro: só liga o intake/transfer se a flywheel estiver pronta.
     */

    public void ShootLogic(int newState) {
        if (!robot.follower.isBusy() && !arrived) {
            arrived = true;
            pathTimer.resetTimer();
        }
        if (arrived) {
            Shoot();
        }
        if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.3) {
            endShoot();
            pathState = newState;
            stateInit = false;
            arrived = false;
            pathTimer.resetTimer();
        }
    }

    public void Shoot() {
        robot.intake.setPower(1);
        if (robot.flywheel.isAtTargetVelocity() && !robot.follower.isBusy()) {
            robot.transfer.runForward();
        } else {
            // Se a velocidade cair (ex: após o primeiro disco sair), ele para e espera recuperar
            robot.transfer.stop();
        }
    }

    public void endShoot() {
        robot.transfer.stop();
    }

    public void Intake() {
        robot.intake.setPower(1);
    }

    public void endIntake() {
        robot.intake.setPower(1);
    }

    /**
     * Inner class containing all autonomous paths, starting from the robot's initial pose.
     */
    public static class Paths {
        public PathChain MainChain;

        public Paths(Follower follower) {
            MainChain = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(40.346, 135.924),
                                    new Pose(52.162, 91.283)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135))
                    .addPath(
                            new BezierCurve(
                                    new Pose(52.162, 91.283),
                                    new Pose(47.122, 82.819),
                                    new Pose(17.360, 83.919)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(17.360, 83.919),
                                    new Pose(52.994, 90.173)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                    .addPath(
                            new BezierCurve(
                                    new Pose(52.994, 90.173),
                                    new Pose(51.508, 55.743),
                                    new Pose(10.610, 58.800)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .addPath(
                            new BezierCurve(
                                    new Pose(10.610, 58.800),
                                    new Pose(17.664, 61.741),
                                    new Pose(14.068, 67.332)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                    .addPath(
                            new BezierLine(
                                    new Pose(14.068, 67.332),
                                    new Pose(53.827, 89.064)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135))
                    .addPath(
                            new BezierCurve(
                                    new Pose(53.827, 89.064),
                                    new Pose(37.034, 72.206),
                                    new Pose(10.048, 34.500)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(10.048, 34.500),
                                    new Pose(54.659, 88.231)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .addPath(
                            new BezierLine(
                                    new Pose(54.659, 88.231),
                                    new Pose(22.751, 71.861)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();
        }
    }
}
