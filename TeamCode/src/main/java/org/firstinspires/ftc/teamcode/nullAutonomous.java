package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.oldSensors;
import org.firstinspires.ftc.teamcode.utils.newMove;

@Autonomous(name="AB AB 0", group="Autonomous Finals")
//@Disabled
public class nullAutonomous extends LinearOpMode {
    @Override
    public void runOpMode(){

        new newMove(this);


        waitForStart();
        oldSensors.gyroDriftRead();

        while (opModeIsActive())
        {
            telemetry.addData("Heading", oldSensors.gyroHeading());
            telemetry.addData("Gyro Heading", oldSensors.gyro.getHeading());
            telemetry.addData("Offset rate", oldSensors.gyrochange);
            telemetry.update();
        }
    }
}
