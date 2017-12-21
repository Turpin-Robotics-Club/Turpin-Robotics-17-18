package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Sensors;
import org.firstinspires.ftc.teamcode.utils.move;

@TeleOp(name = "Gyro Test Sensors", group = "Sensors")
public class gyroTestSensors extends LinearOpMode {


    public void runOpMode()
    {
        move Move = new move(this);
        while (opModeIsActive()) {
            telemetry.addData("Current Pos", Sensors.readGyro());
            telemetry.addData("offset", Sensors.gyroInitial);
            telemetry.addData("actual", Sensors.angles.firstAngle);
            telemetry.update();
        }
    }
}
