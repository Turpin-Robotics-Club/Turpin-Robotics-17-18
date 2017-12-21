package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Lift Test", group="teletests")
//@Disabled
public class liftTest extends OpMode{

    private DcMotor arm;
    private int currentpos = 0;
    public void init()
    {
        arm = hardwareMap.dcMotor.get("relic1");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void loop()
    {
        if(gamepad1.a) {
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setPower(-0.4);
            currentpos = this.arm.getCurrentPosition();
        }
        else if(gamepad1.b) {
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setPower(0.25);
            currentpos = this.arm.getCurrentPosition();
        }
        else
        {
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setTargetPosition(currentpos);

        }
        telemetry.addData("position", arm.getCurrentPosition());
    }
}
