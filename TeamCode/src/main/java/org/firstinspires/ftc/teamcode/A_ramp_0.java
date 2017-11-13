package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.oldSensors;
import org.firstinspires.ftc.teamcode.utils.newMove;


@Autonomous(name="A ramp 0", group="Autonomous Finals")
//@Disabled
public class A_ramp_0 extends LinearOpMode {



    public void runOpMode() throws InterruptedException{



        newMove drive = new newMove(this);
        waitForStart();
        oldSensors.gyroDriftRead();

        drive.left(-17, 0.75);
        sleep(50);
        drive.forward(40, 0.75);
        sleep(10000);

    }
}
