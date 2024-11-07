package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.autonomous.Motors;
import org.firstinspires.ftc.teamcode.autonomous.Servos;
import org.firstinspires.ftc.teamcode.autonomous.Slides;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="DRIVER CONTROL", group="Linear Opmode") // @Autonomous(...) is the other common choice
public class DRIVER_CONTROL extends LinearOpMode /*implements Runnable*/ {
    public Slides slide;
    public Motors panningMotor;
    public Servos claw;
    public Servos orientation;
    public Servos panningServo;

    public DcMotor fl;
    public DcMotor bl;
    public DcMotor fr;
    public DcMotor br;

    boolean dpadRightPressed;
    boolean dpadLeftPressed;
    boolean depositDone = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        slide = new Slides(hardwareMap, "slide", 6000);
        panningMotor = new Motors(hardwareMap, "panningmotor");
        claw = new Servos(hardwareMap, "claw");
        orientation = new Servos(hardwareMap, "orientation");
        panningServo = new Servos(hardwareMap, "panning");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.right_stick_x,
                            -gamepad1.left_stick_x
                    )
            );

            if (gamepad2.a) {
                panningServo.moveBackwardMIN();
            } else if (gamepad2.y) {
                panningServo.moveForwardMAX();
            }

            if (gamepad2.x) {
                orientation.moveBackwardMIN();
            } else if (gamepad2.b) {
                orientation.moveForwardMAX();
            }

            // Slide movement
            if (gamepad1.right_bumper) {
                slide.runForward(1.0, 0);
            } else if (gamepad1.left_bumper) {
                slide.runBackward(1.0, 0);
            } else {
                slide.stopSlide();
            }

            if (gamepad2.right_trigger > 0.8) {
                panningMotor.rotateForward(1.0, 0);
            } else if (gamepad2.left_trigger > 0.8) {
                panningMotor.rotateBackward(1.0, 0);
            } else {
                panningMotor.stopRotation();
            }

            if (gamepad2.dpad_up) {
                claw.moveForwardMAX();
            } else if (gamepad2.dpad_down) {
                claw.moveBackwardMIN();
            }

            if (gamepad1.dpad_right && !dpadRightPressed && !depositDone) {
                // Pan up
                panningMotor.runToTargetPosition(3000, 1.0, "backward");
                // Move slide up
                slide.runToTargetPosition(3000, 1.0, "forward");
                // Move panning up
                panningServo.moveForwardMAX();
                Thread.sleep(250);
                // Deposit block
                claw.moveBackwardMIN();
                Thread.sleep(250);
                // Move panning back
//                panningServo.moveBackwardMIN();
                // Move Slide down
                slide.runToTargetPosition(0, 1.0, "forward");
//                panningServo.moveForwardMAX();
                // Pan down back to original position
                panningMotor.runToTargetPosition(0, 1.0, "forward");
                Thread.sleep(1000);
                depositDone = true;
            } else if (gamepad1.dpad_left && !dpadLeftPressed) {

            }

            dpadLeftPressed = gamepad1.dpad_left;
            dpadRightPressed = gamepad1.dpad_right;
            if (gamepad1.dpad_right) {
                depositDone = false;
            }
        }
    }
}
