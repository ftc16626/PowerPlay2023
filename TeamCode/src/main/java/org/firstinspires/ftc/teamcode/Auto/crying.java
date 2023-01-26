package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.triangulationTesting;
import org.firstinspires.ftc.teamcode.Auto.testAuto;
@Autonomous(name = "crying")

public class crying extends LinearOpMode {

    @Override
    public void runOpMode() {
//        initialize camera and pipeline
        testAuto cv = new testAuto(this);
//      call the function to startStreaming
        cv.observeStick();
        waitForStart();
        while (opModeIsActive()) {
        }
//        stopStreaming
        cv.stopCamera();
    }
}