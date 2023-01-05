package org.firstinspires.ftc.teamcode.Auto;


import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

import java.util.Arrays;


@Autonomous (name="Autonomous")
public class Auto extends LinearOpMode {
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

    public Servo claw; //Port?

    //drivetrain variables
    int leftPos;
    int rightPos;

    //arm variables
    int armPos;


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

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // robot.FrontLeft.setTargetPosition(1000);
        //    robot.BackRight.setTargetPosition(-1000);

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
        liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // TRAJECTORIES
        // gobilda motor 131 encoder ticks per block strafing in inches



        Trajectory traj0 = drive.trajectoryBuilder(new Pose2d(-39.0, -63.0, Math.toRadians(180.0)))
                .forward(.05)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .strafeLeft(3.0)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(5.4)
                .build();

        /*Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeRight(6.0)
                .build();
                */
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeRight(3.2)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .back(3.4)
                .build();

        /*Trajectory traj5 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(-60)))
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .strafeLeft(2.0)
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .back(3.0)
                .build();

*/



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

        int leftPos = 0;
        int rightPos = 0;

        int armPos = 0;

        int liftPos = 0;


        //claw.setPosition(0);

        waitForStart();


        if (isStopRequested()) return;

        //RED
        if (detector.getAvgRed() > detector.getAvgBlue() && detector.getAvgRed() > detector.getAvgGreen()) {
            drive.followTrajectory(traj0);
            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj3);
            drive(0.5, -1200, 0);
            drive.turn(Math.toRadians(-60));
            sleep(1000);
            //drive(0,0, 0.5, 0, 700);
            //sleep(1000);
            //drive(40, -40, 0.5, 0, 0);
            /*claw.setPosition(1);
            sleep(1000);
            drive(40, -40, 0.5, 0, 0);
            sleep(1000);
            drive(0, 0, 0.5, 1200, -700);
            drive.followTrajectory(traj1);
            drive.followTrajectory(traj4);

             */




        }

        //BLUE
        /*if (detector.getAvgBlue() > detector.getAvgRed() && detector.getAvgBlue() > detector.getAvgGreen()) {
            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj3);
            drive(0,0, 0.5, 0, 500);
            drive(0,0,0.5,1, 0);
            drive.followTrajectory(traj5);
            drive.followTrajectory(traj6);
            drive(0, 0, 0.5, 0, -500);
        }

        //GREEN

        else {
            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj7);
            drive(0,0, 0.5, 0, 500);
            drive(0,0,0.5,1, 0);
            drive(0, 0, 0.5, 0, -500);
        }

         */




        //lift.liftServo.setPosition(liftPos);
        //lift.bucketServo.setPosition(bucketPos);

        sleep(2000);



    }


    //private void drive(int leftTarget, int rightTarget, double speed, int armTarget, int liftTarget){
    private void drive(double speed, int armTarget, int liftTarget){

        //leftPos += leftTarget;
        //rightPos += rightTarget;
        armPos += armTarget;
        liftPos += liftTarget;

        /*leftFront.setTargetPosition(leftPos);
        leftBack.setTargetPosition(leftPos);
        rightFront.setTargetPosition(rightPos);
        rightBack.setTargetPosition(rightPos);
         */
        arm.setTargetPosition(armPos);
        liftMotor1.setTargetPosition(liftPos);
        liftMotor2.setTargetPosition(liftPos);




        /*leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         */
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //lift motors
        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);
         */
        arm.setPower(speed);
        liftMotor1.setPower(speed);
        liftMotor2.setPower(speed);

        while(opModeIsActive() && leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()){
            idle();
        }
    }
}

