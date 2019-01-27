package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Combined Drive", group = "3766")
@Disabled


public class TeleOp_3766_v2 extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    Hardware_3766_v1 robot = new Hardware_3766_v1();


    static double STRAFE_POWER = 0.9;


    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot.init(hardwareMap);

        stopAllMotors();

        telemetry.addData("Mode", "initializing...");
        telemetry.update();

    }

    // Code to run REPEATEDLY after INIT, prior to START
    @Override
    public void init_loop() {
        stopAllMotors();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {

        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;

        if (gamepad1.left_bumper) {
            // strafe left
            robot.leftFrontDrive.setPower(-STRAFE_POWER);
            robot.leftRearDrive.setPower(+STRAFE_POWER);
            robot.rightFrontDrive.setPower(+STRAFE_POWER);
            robot.rightRearDrive.setPower(-STRAFE_POWER);


            telemetry.addLine("Strafe LEFT");

        } else if (gamepad1.right_bumper) {
            //strafe right
            robot.leftFrontDrive.setPower(+STRAFE_POWER);
            robot.leftRearDrive.setPower(-STRAFE_POWER);
            robot.rightFrontDrive.setPower(-STRAFE_POWER);
            robot.rightRearDrive.setPower(+STRAFE_POWER);

            telemetry.addLine("Strafe RIGHT");
        } else {
            //normal tank drive
            robot.leftFrontDrive.setPower(left);
            robot.leftRearDrive.setPower(left);
            robot.rightRearDrive.setPower(right);
            robot.rightFrontDrive.setPower(right);

            telemetry.addData("left", "%.2f", left);
            telemetry.addData("right", "%.2f", right);

        }


        // Send telemetry message to signify robot running;

        telemetry.update();

    }


    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
        stopAllMotors();

    }


    // OpMode methods
    private void stopAllMotors() {
        robot.leftFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);

    }


}
