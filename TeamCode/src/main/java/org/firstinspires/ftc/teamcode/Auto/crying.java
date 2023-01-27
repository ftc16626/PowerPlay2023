package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.teleop.teleop;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.opencv.core.Point;
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.triangulationTesting;
import org.firstinspires.ftc.teamcode.Auto.testAuto;


@Autonomous(name = "Autonomous")

public class crying extends LinearOpMode {

    final int width = 320;
    final int height = 240;
    final double perfectPointX = 0;
    final double perfectPointY = 0;

    boolean calculation = false;

    public double rightJunction = 0;
    public double rightDistance = 0;
    public double moveRobot = 0;



    triangulationTesting detector = new triangulationTesting(width);


    @Override
    public void runOpMode() {
//        initialize camera and pipeline
        testAuto cv = new testAuto(this);
//      call the function to startStreaming
        cv.observeStick();

        waitForStart();

        calculation = true;
        if(calculation = true){
            Point perfectPoint = new Point(
                    perfectPointX.x,
                    perfectPointY.y);


            rightJunction = 320 - detector.getCenterX();
            rightDistance = 320 - perfectPointX;
            moveRobot = rightDistance - rightJunction;



        }





        if (isStopRequested()) {
            cv.stopCamera();
            return;
        }

        /*while (opModeIsActive()) {
        }
         */
//        stopStreaming
    }
}