package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teleop.teleop;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous (name="Autonomous")
public class Auto extends LinearOpMode {
    private int liftPos;
    private double bucketPos;
    private double distanceToHub;
    private double bucketDumpPos;

    //define dcmotors
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    DcMotor arm = null;
    DcMotor liftMotor1 = null;
    DcMotor liftMotor2 = null;

    public Servo claw; //Port?
    public ColorSensor colorSensor = null;

    //drivetrain variables
    int leftPos;
    int rightPos;

    //arm variables
    int armPos;


    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // TRAJECTORIES

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(-39.0, -63.0, Math.toRadians(180.0)))
                .strafeLeft(10.0)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(2.6)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeRight(10.0)
                .build();

        final int width = 320;
        final int height = 240;
        boolean test = false;


        SignalReader detector = new SignalReader(width);
        OpenCvCamera camera;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        camera.setPipeline(detector);
        //camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
        while (!isStarted()) {
            telemetry.addData("Avg Red in View", detector.getAvgRed());
            telemetry.addData("Avg Green in View", detector.getAvgGreen());
            telemetry.addData("Avg Blue in View", detector.getAvgBlue());
            telemetry.update();
        }

        /*signal 1 = red
        signal 2 = green
        signal 3 = blue
        */
        boolean signal1 = detector.getAvgRed() > detector.getAvgBlue() + 10 && detector.getAvgRed() > detector.getAvgGreen() + 10;
        boolean signal3 = detector.getAvgBlue() > detector.getAvgRed() + 10 && detector.getAvgBlue() > detector.getAvgGreen() + 10;
        boolean signal2 = false;

        /*if (signal1 = false && signal3 = false) {
            signal2 = true;
        }
         */

        int leftPos = 0;
        int rightPos = 0;

        int armPos = 0;

        int liftPos = 0;


        claw.setPosition(0);

        waitForStart();

        if (detector.getAvgRed() > detector.getAvgBlue() + 10 && detector.getAvgRed() > detector.getAvgGreen() + 10) {
            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);
        }
        if (detector.getAvgBlue() > detector.getAvgRed() + 10 && detector.getAvgBlue() > detector.getAvgGreen() + 10) {
            drive.followTrajectory(traj3);
            drive(1050, 1050, 0.5, 0, 0);
        }
        else {
            drive(900, 900, 0.5, 0, 0);
        }
        //lift.liftServo.setPosition(liftPos);
        //lift.bucketServo.setPosition(bucketPos);

        sleep(2000);

        if (isStopRequested()) return;

    }

    private void drive(int leftTarget, int rightTarget, double speed, int armTarget, int liftTarget) {
        leftPos += leftTarget;
        rightPos += rightTarget;
        armPos += armTarget;
        liftPos += liftTarget;

        FrontLeft.setTargetPosition(leftPos);
        BackLeft.setTargetPosition(leftPos);
        FrontRight.setTargetPosition(rightPos);
        BackRight.setTargetPosition(rightPos);
        arm.setTargetPosition(armPos);
        liftMotor1.setTargetPosition(liftPos);
        liftMotor2.setTargetPosition(liftPos);


        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //lift motors
        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontLeft.setPower(speed);
        BackLeft.setPower(speed);
        FrontRight.setPower(speed);
        BackRight.setPower(speed);
        arm.setPower(speed);
        liftMotor1.setPower(speed);
        liftMotor2.setPower(speed);
        sleep(1000);
    }
}

