
package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Crater Worlds", group = "Exercises")
//@Disabled

public class CraterWorlds_v2 extends LinearOpMode {

    static final double DRIVEPOWER   =  0.5;
    static final double TURNPOWER    =  0.9;
    static final double ENCODE_PPR   = 1020; //NeverRest 40 280 pulse per revolution (ppr)
    static final double WHEEL_CIRCUM = 31.91; // 10cm diameter ~ 31.42 cm/rev

    HardwareOmni robot = new HardwareOmni();

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;
    PIDController  pidRotate, pidDrive, pidEncoder;

    private ElapsedTime runtime = new ElapsedTime();
    private GoldAlignDetector detector;
    double GoldPos;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        pidRotate = new PIDController(.005, 0, 0);


        // Set PID values to reduce poser 60%
        pidEncoder = new PIDController(.005, 0, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);


        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();
        detector.alignSize = 100;
        detector.alignPosOffset = 0;
        detector.downscale = 0.4;
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        detector.maxAreaScorer.weight = 0.005;
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;
        detector.enable();

        telemetry.addData("Status", "Ready to run");
        telemetry.addData("IsAligned", detector.getAligned());
        telemetry.addData("X Pos", detector.getXPosition());
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        runtime.reset();

        // drive until end of period.

        while (opModeIsActive()) {

            if(detector.getXPosition() < 200){
                GoldPos = 1;      //ROLLED LEFT, RIGHT OF BOT
            }else if (detector.getXPosition() > 400){
                GoldPos = 3;      //ROLLED RIGHT, LEFT OF BOT
            }else if (detector.getXPosition() > 200 && detector.getXPosition() < 400){
                GoldPos = 2;      //ROLLED CENTER, CENTER OF BOT
            }
            telemetry.addData("Gold Position: ", GoldPos);
            telemetry.update();

//            robot.startauto(1.0);
                robot.sleep(0.5);
            robot.backward(0.2);
                robot.sleep(1.0);
//            straight(.2,.2);
//            robot.sleep(1.0);
            strafe(0.2 * WHEEL_CIRCUM, -DRIVEPOWER); // left
                robot.sleep(1.0);
            straight(+1.2 , DRIVEPOWER);
                robot.sleep(1.0);

            if (GoldPos == 3) { //ROLLED RIGHT, LEFT OF BOT
                rotate(60,TURNPOWER);
                    robot.sleep(0.100);
                straight(35, DRIVEPOWER);
                    robot.sleep(0.100);
                straightback(-34, -DRIVEPOWER);
                    robot.sleep(0.100);
                rotate(70,TURNPOWER);
                    robot.sleep(0.100);
                straight(65, DRIVEPOWER);
                    robot.sleep(0.100);
                rotate(60,TURNPOWER);
                    robot.sleep(0.100);
                straight(4 * WHEEL_CIRCUM, DRIVEPOWER);
                    robot.sleep(0.100);
                robot.drop(1.0);
                    robot.sleep(0.250);
                straightback(-5 * WHEEL_CIRCUM, -DRIVEPOWER);
                    robot.sleep(0.250);

            } else if (GoldPos == 2){ //ROLLED CENTER, CENTER OF BOT
                rotate(90, TURNPOWER);
                    robot.sleep(0.100);
                straight(1 * WHEEL_CIRCUM, DRIVEPOWER);
                    robot.sleep(0.100);
                straightback(-1 * WHEEL_CIRCUM, -DRIVEPOWER);
                    robot.sleep(0.100);
                rotate(50,TURNPOWER);
                    robot.sleep(0.100);
                straight(2.1 * WHEEL_CIRCUM, DRIVEPOWER);
                    robot.sleep(0.100);
                rotate(40,TURNPOWER);
                    robot.sleep(0.100);
                straight(4 * WHEEL_CIRCUM, DRIVEPOWER);
                    robot.sleep(0.250);
                robot.drop(1.0);
                    robot.sleep(0.100);
                straightback(-5 * WHEEL_CIRCUM, -DRIVEPOWER);
                    robot.sleep(0.100);

            } else if (GoldPos == 1) { //ROLLED LEFT, RIGHT OF BOT
                rotate(125, TURNPOWER);
                    robot.sleep(0.100);
                straight(2 * WHEEL_CIRCUM, DRIVEPOWER);
                    robot.sleep(0.100);
                rotate(80,TURNPOWER);
                    robot.sleep(0.100);
                straight(4 * WHEEL_CIRCUM, DRIVEPOWER);
                    robot.sleep(0.100);
                robot.drop(1.0);
                    robot.sleep(0.100);
                straightback(-5 * WHEEL_CIRCUM, -DRIVEPOWER);
                    robot.sleep(0.100);
            }

            // turn the motors off.
            while (opModeIsActive()) {
                telemetry.addData("Done: ", runtime);
                telemetry.update();

                robot.leftFront.setPower(0);
                robot.rightFront.setPower(0);
                robot.leftRear.setPower(0);
                robot.rightRear.setPower(0);
            }
        }
    }

    /**
     * Calculate number of pulse ticks for a given distance
     */
    private int calcPPR(double dist) {
        return (int) ((dist / WHEEL_CIRCUM) * ENCODE_PPR);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .02;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Drive straight for dist centimeters.
     * @param dist Centimeters to drive
     */
    private void straight(double dist, double power) {
        resetAngle();
        robot.sleep(0.1);

        // int currentLeftEnc  = leftFront.getCurrentPosition();    ////
        int currentBackLeftEnc = robot.leftRear.getCurrentPosition();

        // int targetLeftEnc  = currentLeftEnc + calcPPR(dist);  ////
        int targetBackLeftEnc = currentBackLeftEnc + calcPPR(dist);

        do {
            correction = checkDirection();
            robot.leftFront.setPower(power  + correction);
            robot.rightFront.setPower(-power + correction);
            robot.leftRear.setPower(power  + correction);
            robot.rightRear.setPower(-power + correction);
            currentBackLeftEnc = robot.leftRear.getCurrentPosition();
            telemetry.addData("1 power", power);
            telemetry.addData("2 correction", correction);
            telemetry.addData("3 target", targetBackLeftEnc);
            telemetry.addData("4 left rear enc", currentBackLeftEnc);
            telemetry.addData("5 straight", dist);
            telemetry.update();

        } while (currentBackLeftEnc < targetBackLeftEnc && opModeIsActive());

        robot.sleep(0.1);
    }


    private void straightback(double dist, double power) {
        resetAngle();
        robot.sleep(0.1);

        // int currentLeftEnc  = leftFront.getCurrentPosition();    ////
        int currentBackLeftEnc = robot.leftRear.getCurrentPosition();

        // int targetLeftEnc  = currentLeftEnc + calcPPR(dist);  ////
        int targetBackLeftEnc = currentBackLeftEnc + calcPPR(dist);

        do {
            correction = checkDirection();
            robot.leftFront.setPower(power  + correction);
            robot.rightFront.setPower(-power + correction);
            robot.leftRear.setPower(power  + correction);
            robot.rightRear.setPower(-power + correction);
            currentBackLeftEnc = robot.leftRear.getCurrentPosition();
            telemetry.addData("1 power", power);
            telemetry.addData("2 correction", correction);
            telemetry.addData("3 target", targetBackLeftEnc);
            telemetry.addData("4 left rear enc", currentBackLeftEnc);
            telemetry.addData("5 st_back", dist);
            telemetry.update();
        } while (currentBackLeftEnc > targetBackLeftEnc && opModeIsActive());

        // power down and pause
        robot.sleep(0.1);
    }


    private void strafe(double dist, double power) {// negative is left and positive is right
        resetAngle();
        robot.sleep(0.1);

        int sign = +1;
        if (power < 0){
            sign = -1;
        }

        //   int currentLeftEnc  = leftFront.getCurrentPosition();
        int currentBackLeftEnc = robot.leftRear.getCurrentPosition();

        // int targetLeftEnc  = currentLeftEnc + calcPPR(dist);
        int targetBackLeftEnc = currentBackLeftEnc + calcPPR(dist);

        pidEncoder.reset();
        pidEncoder.setSetpoint(targetBackLeftEnc);
        pidEncoder.setInputRange(currentBackLeftEnc, targetBackLeftEnc);
        pidEncoder.setOutputRange(.50, power);
        pidEncoder.setTolerance(2);
        pidEncoder.enable();

        do {
            power = pidEncoder.performPID(currentBackLeftEnc);
            correction = checkDirection();
            robot.leftFront.setPower(-power);
            robot.rightFront.setPower(-power);
            robot.leftRear.setPower(power + (correction * sign));
            robot.rightRear.setPower(power - (correction *sign));
            currentBackLeftEnc = robot.leftRear.getCurrentPosition();
            telemetry.addData("1 power", power);
            telemetry.addData("2 correction", correction);
            telemetry.addData("3 target", targetBackLeftEnc);
            telemetry.addData("4 left rear enc", currentBackLeftEnc);
            telemetry.addData("5 strafe", dist);
            telemetry.update();
        } while (opModeIsActive() && !pidEncoder.onTarget());

        // power down and pause
        robot.sleep(0.1);
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) { // negative power on right and positive power on left
        // restart imu angle tracking.
        resetAngle();

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle with a minimum of 20%.
        // This is to prevent the robots momentum from overshooting the turn after we turn off the
        // power. The PID controller reports onTarget() = true when the difference between turn
        // angle and target angle is within 2% of target (tolerance). This helps prevent overshoot.
        // The minimum power is determined by testing and must enough to prevent motor stall and
        // complete the turn. Note: if the gap between the starting power and the stall (minimum)
        // power is small, overshoot may still occur. Overshoot is dependant on the motor and
        // gearing configuration, starting power, weight of the robot and the on target tolerance.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, Math.abs(degrees));
        pidRotate.setOutputRange(0.35, power);
        pidRotate.setTolerance(2);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                robot.leftFront.setPower(power);
                robot.rightFront.setPower(power);
                robot.leftRear.setPower(power);
                robot.rightRear.setPower(power);
                sleep(1000);
            }

            do
            {
                power = pidRotate.performPID(getAngle());
                robot.leftFront.setPower(power);
                robot.rightFront.setPower(power);
                robot.leftRear.setPower(power);
                robot.rightRear.setPower(power);
                telemetry.addData("1 PID     power:", power);
                telemetry.addData("2 current angle:", getAngle());
                telemetry.addData("3 Rotate target:", degrees);
                telemetry.update();

            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle());
                robot.leftFront.setPower(-power);
                robot.rightFront.setPower(-power);
                robot.leftRear.setPower(-power);
                robot.rightRear.setPower(-power);
                telemetry.addData("1 PID     power:", power);
                telemetry.addData("2 current angle:", getAngle());
                telemetry.addData("3 Rotate target:", degrees);
                telemetry.update();
            } while (opModeIsActive() && !pidRotate.onTarget());

        // power down & wait for rotation to stop.
        robot.sleep(0.5);

        // reset angle tracking on new heading.
        resetAngle();
    }


    // PID controller courtesy of Peter Tischler, with modifications.
    public class PIDController
    {
        private double m_P;                     // factor for "proportional" control
        private double m_I;                     // factor for "integral" control
        private double m_D;                     // factor for "derivative" control
        private double m_input;                 // sensor input for pid controller
        private double m_maximumOutput = 1.0;   // |maximum output|
        private double m_minimumOutput = -1.0;  // |minimum output|
        private double m_maximumInput = 0.0;    // maximum input - limit setpoint to this
        private double m_minimumInput = 0.0;    // minimum input - limit setpoint to this
        private boolean m_continuous = false;   // do the endpoints wrap around? eg. Absolute encoder
        private boolean m_enabled = false;      // is the pid controller enabled
        private double m_prevError = 0.0;       // the prior sensor input (used to compute velocity)
        private double m_totalError = 0.0;      // the sum of the errors for use in the integral calc
        private double m_tolerance = 0.05;      // the percentage error that is considered on target
        private double m_setpoint = 0.0;
        private double m_error = 0.0;
        private double m_result = 0.0;

        /**
         * Allocate a PID object with the given constants for P, I, D
         * @param Kp the proportional coefficient
         * @param Ki the integral coefficient
         * @param Kd the derivative coefficient
         */
        public PIDController(double Kp, double Ki, double Kd)
        {
            m_P = Kp;
            m_I = Ki;
            m_D = Kd;
        }

        /**
         * Read the input, calculate the output accordingly, and write to the output.
         * This should only be called by the PIDTask
         * and is created during initialization.
         */
        private void calculate()
        {
            int     sign = 1;

            // If enabled then proceed into controller calculations
            if (m_enabled)
            {
                // Calculate the error signal
                m_error = m_setpoint - m_input;

                // If continuous is set to true allow wrap around
                if (m_continuous)
                {
                    if (Math.abs(m_error) > (m_maximumInput - m_minimumInput) / 2)
                    {
                        if (m_error > 0)
                            m_error = m_error - m_maximumInput + m_minimumInput;
                        else
                            m_error = m_error + m_maximumInput - m_minimumInput;
                    }
                }

                // Integrate the errors as long as the upcoming integrator does
                // not exceed the minimum and maximum output thresholds.

                if ((Math.abs(m_totalError + m_error) * m_I < m_maximumOutput) &&
                        (Math.abs(m_totalError + m_error) * m_I > m_minimumOutput))
                    m_totalError += m_error;

                // Perform the primary PID calculation
                m_result = m_P * m_error + m_I * m_totalError + m_D * (m_error - m_prevError);

                // Set the current error to the previous error for the next cycle.
                m_prevError = m_error;

                if (m_result < 0) sign = -1;    // Record sign of result.

                // Make sure the final result is within bounds. If we constrain the result, we make
                // sure the sign of the constrained result matches the original result sign.
                if (Math.abs(m_result) > m_maximumOutput)
                    m_result = m_maximumOutput * sign;
                else if (Math.abs(m_result) < m_minimumOutput)
                    m_result = m_minimumOutput * sign;
            }
        }

        /**
         * Set the PID Controller gain parameters.
         * Set the proportional, integral, and differential coefficients.
         * @param p Proportional coefficient
         * @param i Integral coefficient
         * @param d Differential coefficient
         */
        public void setPID(double p, double i, double d)
        {
            m_P = p;
            m_I = i;
            m_D = d;
        }

        /**
         * Get the Proportional coefficient
         * @return proportional coefficient
         */
        public double getP() {
            return m_P;
        }

        /**
         * Get the Integral coefficient
         * @return integral coefficient
         */
        public double getI() {
            return m_I;
        }

        /**
         * Get the Differential coefficient
         * @return differential coefficient
         */
        public double getD() {
            return m_D;
        }

        /**
         * Return the current PID result for the last input set with setInput().
         * This is always centered on zero and constrained the the max and min outs
         * @return the latest calculated output
         */
        public double performPID()
        {
            calculate();
            return m_result;
        }

        /**
         * Return the current PID result for the specified input.
         * @param input The input value to be used to calculate the PID result.
         * This is always centered on zero and constrained the the max and min outs
         * @return the latest calculated output
         */
        public double performPID(double input)
        {
            setInput(input);
            return performPID();
        }

        /**
         *  Set the PID controller to consider the input to be continuous,
         *  Rather then using the max and min in as constraints, it considers them to
         *  be the same point and automatically calculates the shortest route to
         *  the setpoint.
         * @param continuous Set to true turns on continuous, false turns off continuous
         */
        public void setContinuous(boolean continuous) {
            m_continuous = continuous;
        }

        /**
         *  Set the PID controller to consider the input to be continuous,
         *  Rather then using the max and min in as constraints, it considers them to
         *  be the same point and automatically calculates the shortest route to
         *  the setpoint.
         */
        public void setContinuous() {
            this.setContinuous(true);
        }

        /**
         * Sets the maximum and minimum values expected from the input.
         * @param minimumInput the minimum value expected from the input, always positive
         * @param maximumInput the maximum value expected from the output, always positive
         */
        public void setInputRange(double minimumInput, double maximumInput)
        {
            m_minimumInput = Math.abs(minimumInput);
            m_maximumInput = Math.abs(maximumInput);
            setSetpoint(m_setpoint);
        }

        /**
         * Sets the minimum and maximum values to write.
         * @param minimumOutput the minimum value to write to the output, always positive
         * @param maximumOutput the maximum value to write to the output, always positive
         */
        public void setOutputRange(double minimumOutput, double maximumOutput)
        {
            m_minimumOutput = Math.abs(minimumOutput);
            m_maximumOutput = Math.abs(maximumOutput);
        }

        /**
         * Set the setpoint for the PIDController
         * @param setpoint the desired setpoint
         */
        public void setSetpoint(double setpoint)
        {
            int     sign = 1;

            if (m_maximumInput > m_minimumInput)
            {
                if (setpoint < 0) sign = -1;

                if (Math.abs(setpoint) > m_maximumInput)
                    m_setpoint = m_maximumInput * sign;
                else if (Math.abs(setpoint) < m_minimumInput)
                    m_setpoint = m_minimumInput * sign;
                else
                    m_setpoint = setpoint;
            }
            else
                m_setpoint = setpoint;
        }

        /**
         * Returns the current setpoint of the PIDController
         * @return the current setpoint
         */
        public double getSetpoint() {
            return m_setpoint;
        }

        /**
         * Retruns the current difference of the input from the setpoint
         * @return the current error
         */
        public synchronized double getError() {
            return m_error;
        }

        /**
         * Set the percentage error which is considered tolerable for use with
         * OnTarget. (Input of 15.0 = 15 percent)
         * @param percent error which is tolerable
         */
        public void setTolerance(double percent) {
            m_tolerance = percent;
        }

        /**
         * Return true if the error is within the percentage of the total input range,
         * determined by setTolerance. This assumes that the maximum and minimum input
         * were set using setInputRange.
         * @return true if the error is less than the tolerance
         */
        public boolean onTarget()
        {
            return (Math.abs(m_error) < Math.abs(m_tolerance / 100 * (m_maximumInput - m_minimumInput)));
        }

        /**
         * Begin running the PIDController
         */
        public void enable() {
            m_enabled = true;
        }

        /**
         * Stop running the PIDController.
         */
        public void disable() {
            m_enabled = false;
        }

        /**
         * Reset the previous error,, the integral term, and disable the controller.
         */
        public void reset()
        {
            disable();
            m_prevError = 0;
            m_totalError = 0;
            m_result = 0;
        }

        /**
         * Set the input value to be used by the next call to performPID().
         * @param input Input value to the PID calculation.
         */
        public void setInput(double input)
        {
            int     sign = 1;

            if (m_maximumInput > m_minimumInput)
            {
                if (input < 0) sign = -1;

                if (Math.abs(input) > m_maximumInput)
                    m_input = m_maximumInput * sign;
                else if (Math.abs(input) < m_minimumInput)
                    m_input = m_minimumInput * sign;
                else
                    m_input = input;
            }
            else
                m_input = input;
        }
    }
}
