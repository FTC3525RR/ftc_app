package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.CommonLibrary;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;

@Autonomous (name = "Auto Driver test")

public class AutonomousDriverTest extends LinearOpMode {

    @Override
    public void runOpMode(){

        RobotHardware rh = new RobotHardware();
        CommonLibrary cl = new CommonLibrary();
        AutonomousLibrary al = new AutonomousLibrary();
        rh.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()){

            al.encoderTicksToInchesTest(telemetry, this);

        }

    }
}
