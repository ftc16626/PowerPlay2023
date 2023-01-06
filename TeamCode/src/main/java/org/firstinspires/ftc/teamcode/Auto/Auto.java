package org.firstinspires.ftc.teamcode.Auto;


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
import org.firstinspires.ftc.teamcode.teleop.RobotCentricSample;
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
        //  robot.BackRight.setTargetPosition(-1000);

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



        /* The generation of the trajectories start here. You can name them whatever you want, but for
        this autonomous I made trajectories from 0 on. */


        Trajectory traj0 = drive.trajectoryBuilder(new Pose2d(-39.0, -63.0, Math.toRadians(180.0)))
                .forward(5)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .strafeLeft(34)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(43)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeRight(35)
                .build();

        /*Trajectory traj5 = drive.trajectoryBuilder(traj3.end())
                .strafeRight(0)
                .addDisplacementMarker(() -> {
                    // This marker runs after the first splineTo()
                    move(0.5, -1200, 700);
                    // Run your action in here!
                })
                .build();
         */


        Trajectory traj4 = drive.trajectoryBuilder(traj3.end().plus(new Pose2d(0, 0, Math.toRadians(-70))))
                .forward(8)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .back(10)
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .forward(25)
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .back(30)
                .build();


        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .forward(10)
                .build();

        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .back(10)
                .build();

        /**everything after this is not part of the origin



        // Red

        Trajectory traj10 = drive.trajectoryBuilder(traj9.end())
                .forward(38)
                .build();
        Trajectory traj11 = drive.trajectoryBuilder(traj10.end())
                .strafeLeft(20)
                .build();

         // Green
         drive.turn(Math.toRadians(60));

         Trajectory traj12 = drive.trajectoryBuilder(traj11.end())
                .back(14)
                .build();

         // Blue
         drive.turn(Math.toRadians(60));

         Trajectory traj12 = drive.trajectoryBuilder(traj11.end())
            .strafeRight(30)
            .build();

         Trajectory traj13 = drive.trajectoryBuilder(traj11.end())
            .back(14)
            .build();

         **/



        final int width = 320;
        final int height = 240;
        boolean test = false;


        // This has to do with the computer vision. The other file in this package called SignalReader is the camera configuration portion
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
        // These statements below show up on the driver station and detect the average amounts of RGB in the frame created in SignalReader
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

        claw.setPosition(0);
        //claw.setPosition(0);

        waitForStart();


        if (isStopRequested()) return;


        //RED
        if (detector.getAvgRed() > detector.getAvgBlue() && detector.getAvgRed() > detector.getAvgGreen()) {
            drive.followTrajectory(traj0);
            move(0.5, -100, 0);
            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj3);
            drive.turn(Math.toRadians(-57));
            move(0.5, -1000, 800); // -1200 900 lift before and worked
            sleep(1000);
            //move arms and lift
            //moves forward to pole
            drive.followTrajectory(traj4);
            move(0.5, 300, 800);
            claw.setPosition(0.3);

            // raises up arm above pole
            move(0.5, -200, 0);
            //move back
            drive.followTrajectory(traj5);

            //lift goes down
            move(0.5, 300, -750);
            //turn
            drive.turn(Math.toRadians(205));

            //drops lift and arm
            move(0.5, -100, -900);
            sleep(1000);
            //moves forward
            drive.followTrajectory(traj6);
            //moves into arm
            move(0.5, 400, 0);
            claw.setPosition(0);
            sleep(500);
            // moves above arm
            move(0.5, -500, 0);
            //moves backward
            drive.followTrajectory(traj7);
            move(0.5, 0, 900);
            sleep(1000);
            //turns right
            drive.turn(Math.toRadians(-150));
            //moves forward
            drive.followTrajectory(traj8);
            //release cone
            claw.setPosition(0.3);
            //moves back
            drive.followTrajectory(traj9);
            //turns
            drive.turn(Math.toRadians(150));
            claw.setPosition(0);
            move(0.5, 1000, -900);
            sleep(1000);
            //RED PORTION
            //drive.followTrajectory(traj10);
            //drive.followTrajectory(traj11);


            /**this stuff should be added after test
             drive.followTrajectory(traj5);
             drive.turn(Math.toRadians(150));
             //if an issue arises then add move(0.5, 0, -900);
             drive.followTrajectory(traj6);
             move(0.5, 500, 0);
             claw.setPosition(0);
             move(0.5, -500, 0);
             drive.followTrajectory(traj7);
             move(0.5, 0, 900);
             sleep(1000);
             drive.turn(Math.toRadians(-150));
             drive.followTrajectory(traj8);
             claw.setPosition(0.3);
             drive.followTrajectory(traj9);
             drive.turn(Math.toRadians(150));
             move(0.5, 0, 0);
             sleep(1000);
             //RED PORTION
             drive.followTrajectory(traj10);
             drive.followTrajectory(traj11);

            **/
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
    private void move(double speed, int armTarget, int liftTarget){

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
    }
}

