package org.firstinspires.ftc.teamcode.autonomous;
//6.92307692308 is 1 inch
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "drive")
public class Auto1 extends LinearOpMode {
    public static double DISTANCE = 24; // in

    public Slide slide;
    public Motors panningMotor;
    public Servos claw;
    public Servos panningServo;
    public Servos orientation;
    public Motors pulley;
    public HuskyLenses frontLens;


    public int runFrames;

    public HuskyLens.Block currentTarget;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        slide = new Slide(hardwareMap, telemetry, "slide");
        panningMotor = new Motors(hardwareMap, "panningmotor");
        claw = new Servos(hardwareMap, "claw");
        panningServo = new Servos(hardwareMap, "panning");
        orientation = new Servos(hardwareMap, "orientation");
        pulley = new Motors(hardwareMap, "pulley");
        frontLens = new HuskyLenses(hardwareMap, "frontlens", "color");


        runFrames = 0;
        claw.moveForwardMAX();
        panningServo.moveForwardMAX();
        orientation.moveForwardMAX();


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory lineTo1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(0, 24.3, Math.toRadians(0)))
                .build();
        Trajectory backward = drive.trajectoryBuilder(startPose)
                .back(10.5)
                .build();
        Trajectory backward1 = drive.trajectoryBuilder(startPose)
                .back(4.5)
                .build();
        Trajectory backward2 = drive.trajectoryBuilder(startPose)
                .back(9)
                .build();
        Trajectory lineTo2 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(16, 0, Math.toRadians(0)))
                .build();
        Trajectory lineTo3 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-4, -17, Math.toRadians(0)))
                .build();
        Trajectory lineTo6 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-16, 0, Math.toRadians(0)))
                .build();
        Trajectory lineTo7 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(0, -15, Math.toRadians(0)))
                .build();
        Trajectory lineTo4 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(0, 16, Math.toRadians(0)))
                .build();
        Trajectory lineTo5 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(19, 0, Math.toRadians(0)))
                .build();
        Trajectory backward17 = drive.trajectoryBuilder(startPose)
                .back(8)
                .build();
        Trajectory forward = drive.trajectoryBuilder(startPose)
                .forward(36)
                .build();


        waitForStart();

        if (isStopRequested()) return;

        //following is the BASE DEPOSIT CODE
        orientation.moveForwardMAX();
        panningMotor.rotateForward(0.8, 300);
        slide.MoveToLevelAsync(Slide.level.slide_highbucket);
        //remove from base deposit code
        drive.followTrajectory(backward);
        panningServo.moveBackwardMIN();
        sleep(500);
        claw.moveBackwardMIN();
        sleep(500);
        panningServo.moveForwardMAX();
        sleep(500);
        // I MADE THIS ASYNC
        slide.MoveToLevel(Slide.level.zero);
        //remove from base deposit code
        drive.turn(2.5);
        panningMotor.rotateForward(-1.0, 300);
        drive.followTrajectory(backward1);
        drive.followTrajectory(lineTo1);
        drive.followTrajectory(lineTo2);
        drive.turn(0.7);
        sleep(700);
        claw.moveForwardMAX();
        sleep(700);
        drive.turn(-0.7);
        panningMotor.rotateForward(0.8, 300);
        slide.MoveToLevelAsync(Slide.level.slide_highbucket);
        drive.followTrajectory(backward);
        drive.followTrajectory(lineTo3);
        drive.followTrajectory(backward2);
        //following is the BASE DEPOSIT CODE
        orientation.moveForwardMAX();
        panningServo.moveBackwardMIN();
        sleep(500);
        claw.moveBackwardMIN();
        sleep(500);
        panningServo.moveForwardMAX();
        sleep(500);
        slide.MoveToLevelAsync(Slide.level.zero);
        drive.followTrajectory(lineTo4);
        panningMotor.rotateForward(-1.0, 300);
        drive.followTrajectory(lineTo5);
        drive.turn(1.2);
        claw.moveForwardMAX();
        sleep(500);
        panningMotor.rotateForward(0.8, 300);
        slide.MoveToLevelAsync(Slide.level.slide_highbucket);
        drive.turn(-1.35);
        drive.followTrajectory(lineTo6);
        drive.followTrajectory(lineTo7);
        drive.followTrajectory(backward17);
        panningServo.moveBackwardMIN();
        sleep(500);
        claw.moveBackwardMIN();
        sleep(500);
        panningServo.moveForwardMAX();
        sleep(500);
        slide.MoveToLevelAsync(Slide.level.zero);
        drive.followTrajectory(forward);
        panningMotor.rotateForward(-1.0, 300);
    }

}
