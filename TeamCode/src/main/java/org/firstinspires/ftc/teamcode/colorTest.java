package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.oldSensors;
import org.firstinspires.ftc.teamcode.utils.newMove;

@TeleOp(name="Color Test", group="Utilities")
//@Disabled
public class colorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        new newMove(this);
        while (opModeIsActive())
        {
            telemetry.addData("color", oldSensors.checkColor());
            telemetry.update();
        }
    }
}
