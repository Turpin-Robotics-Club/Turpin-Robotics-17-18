package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.move;


@Autonomous(name="1010", group="Autonomous Finals")
public class testDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        move drive = new move(this);
        drive.forward(100, 0.75);

    }
}
