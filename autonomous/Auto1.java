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

@Config
@Autonomous(group = "drive")
public class Auto1 extends LinearOpMode {
    public static double DISTANCE = 24; // in

    public Slide slide;
    public Slide panningMotor;
    public Servos claw;
    public Servos panningServo;


    public int runFrames;

    public HuskyLens.Block currentTarget;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        slide = new Slide(hardwareMap, telemetry, "slide");
        panningMotor = new Slide(hardwareMap, telemetry, "panningmotor");
        claw = new Servos(hardwareMap, "claw");
        panningServo = new Servos(hardwareMap, "panning");

        runFrames = 0;
        claw.moveForwardMAX();
        panningServo.moveForwardMAX();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory lineTo1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-24, 24.5, Math.toRadians(0)))
                .build();


        waitForStart();

        if (isStopRequested()) return;

        panningMotor.MoveToLevel(Slide.level.pan_highbucket);
        slide.MoveToLevel(Slide.level.slide_highbucket);
        sleep(1000);
        panningServo.moveForwardMAX();
        sleep(1000);
        claw.moveBackwardMIN();
        sleep(1000);
        panningServo.moveBackwardMIN();
        sleep(100);
        slide.MoveToLevel(Slide.level.zero);
        panningMotor.MoveToLevelAsync(Slide.level.zero);
        drive.followTrajectory(lineTo1);
        slide.MoveToLevel(Slide.level.pan_highbucket);
        claw.moveForwardMAX();
        sleep(1000);

        /*
        panningMotor.MoveToLevelAsync(Slide.level.pan_highbucket);
        drive.turn(Math.toRadians(160));
        drive.followTrajectory(forward1);
        slide.MoveToLevel(Slide.level.slide_highbucket);
        sleep(1000);
        panningServo.moveForwardMAX();
        sleep(1000);
        claw.moveBackwardMIN();
        sleep(1000);
        panningServo.moveBackwardMIN();
        sleep(100);
        slide.MoveToLevel(Slide.level.zero);
        panningMotor.MoveToLevel(Slide.level.zero);

        drive.followTrajectory(turn1);
        slide.MoveToLevel(Slide.level.pan_highbar);
        claw.moveForwardMAX();
        sleep(500);
        panningMotor.MoveToLevelAsync(Slide.level.pan_highbucket);
        sleep(1000);
        slide.MoveToLevel(Slide.level.slide_highbucket);
        sleep(500);
        panningServo.moveForwardMAX();
        sleep(1000);
        claw.moveBackwardMIN();
        sleep(1000);
        panningServo.moveBackwardMIN();
        sleep(1000);
        drive.followTrajectory(forward2);
        slide.MoveToLevel(Slide.level.pan_highbar);
        sleep(1000);
        panningMotor.MoveToLevelAsync(Slide.level.zero);
        slide.MoveToLevel(Slide.level.zero);
        sleep(1000);*/

    }

}
