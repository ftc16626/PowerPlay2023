package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "cameraCompute")

public class cameraCompute extends LinearOpMode {

    // position variables
    private int liftPos;
    private double bucketPos;
    private double distanceToHub;
    private double bucketDumpPos;

    //define dcmotors
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;
    DcMotor arm = null;
    DcMotor liftMotor1 = null;
    DcMotor liftMotor2 = null;

    public Servo claw;

    //drivetrain variables (would have been used in drive method at the bottom)
    int leftPos;
    int rightPos;

    //arm variable
    int armPos;




    final int width = 320;
    final int height = 240;
    static final Point perfectPoint = new Point(160,120);

    final double perfectPointX = 160;
    final double perfectPointY = 120;

    boolean calculation = false;

    public double rightJunction = 0;
    public double rightDistance = 0;
    public double moveRobot = 0;

    OpenCvWebcam webcam;
    blockedVision.BlockedVisionPipeline pipeline;
    blockedVision.BlockedVisionPipeline.junctionPosition snapshotAnalysis = blockedVision.BlockedVisionPipeline.junctionPosition.LEFT; // default

    triangulationTesting detector = new triangulationTesting(width);


    @Override
    public void runOpMode() {

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        claw = hardwareMap.servo.get("claw");
        arm = hardwareMap.dcMotor.get("arm");
        liftMotor1 = hardwareMap.dcMotor.get("liftMotor1");
        liftMotor2 = hardwareMap.dcMotor.get("liftMotor2");

        //specifies that the devices are using encoders which allow for encoder (ticks)-based autonomous rather than timebased
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Because the motors are inverted (facing the center) on the robot, you have to reverse one side in order to make all the wheels move the same direction
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //these lines reset the encoders to 0 so that every time you use a command that uses encoders, you start from 0 rather than updating a value
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //one of the lift motors is reversed so that the lift can move up and down
        liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        //this calls the sample mecanum drive from roadrunner (check the drive package to the left)
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new blockedVision.BlockedVisionPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }


            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        calculation = true;

        snapshotAnalysis = pipeline.getAnalysis();

        /*
         * Show that snapshot on the telemetry
         */
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        switch (snapshotAnalysis)
        {
            case LEFT:
            {
                /* Your autonomous code */
                break;
            }

            case RIGHT:
            {
                /* Your autonomous code */
                break;
            }

            case CENTER:
            {
                sleep(1000);
            }
        }

        if(calculation = true){
            rightJunction = 320 - detector.getCenterX();
            rightDistance = 320 - perfectPointX;
            moveRobot = detector.getCenterX() - perfectPointX;

            telemetry.addData("moveRobot", moveRobot);
            telemetry.addData("CenterX", detector.getCenterX());
            telemetry.update();
        }

        sleep(1000000000);









        /*while (opModeIsActive()) {
        }
         */
//        stopStreaming
    }
}