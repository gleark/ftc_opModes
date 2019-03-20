
// Simple autonomous program that drives bot forward for a given amount
// of time then does right-hand 90-deg trun; or does turn when gamepad A,B
// button is pushed.
//
// Also uses IMU to drive in a straight line when not avoiding an obstacle.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Encode Imu PID", group = "Exercises")
//@Disabled

public class Drive_Encode_Imu_PID extends LinearOpMode {

    static final double DRIVEPOWER   =  0.5;
    static final double TURNPOWER    =  0.4;
    static final int    TURNCORR     =   +9; // degrees corr, pos corrects for oversteer, neg for under
    static final double ENCODE_PPR   = 1020; // NeverRest 40 280 pulse per revolution (ppr)
    static final double WHEEL_CIRCUM = 31.4; // 10cm diameter ~ 31.42 cm/rev

    DcMotor leftMotor;
    DcMotor rightMotor;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;
    PIDController  pidRotate, pidDrive, pidEncoder;

    private ElapsedTime runtime = new ElapsedTime();

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor = hardwareMap.dcMotor.get("motorZero");

        rightMotor = hardwareMap.dcMotor.get("motorOne");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
//        pidDrive = new PIDController(.05, 0, 0);

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

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        runtime.reset();

        // drive until end of period.

        while (opModeIsActive()) {

            // Use gyro to drive in a straight line for dist
            straight(8.0 * WHEEL_CIRCUM, DRIVEPOWER);
            sleep(250);
            rotate(90,  TURNPOWER);
            straight(5.0 * WHEEL_CIRCUM, DRIVEPOWER);
            sleep(250);
            rotate(90,  TURNPOWER);
            straight(5.0 * WHEEL_CIRCUM, DRIVEPOWER);
            sleep(250);
            rotate(+90, TURNPOWER);
            straight(5.0 * WHEEL_CIRCUM, DRIVEPOWER);
            sleep(250);
            rotate(+90, TURNPOWER);

        }

        // turn the motors off.
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    } // end of runOpMode

    // ============================================================================

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
     *
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
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .025;

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
     *
     * @param dist Centimeters to drive
     */

    private void straight(double dist, double power) {
        int currentLeftEnc  = leftMotor.getCurrentPosition();
        int currentRightEnc = rightMotor.getCurrentPosition();

        int targetRightEnc = currentRightEnc + calcPPR(dist);

        pidEncoder.reset();
        pidEncoder.setSetpoint(targetRightEnc);
        pidEncoder.setInputRange(0, targetRightEnc+100);
        pidEncoder.setOutputRange(.20, power);
        pidEncoder.setTolerance(2);
        pidEncoder.enable();

        do
        {
            power = pidEncoder.performPID(currentRightEnc); // power will be + on left turn.
            correction = checkDirection();
            //correction = pidDrive.performPID(getAngle());
            leftMotor.setPower(-power + correction);
            rightMotor.setPower(-power - correction);
            //currentLeftEnc  = leftMotor.getCurrentPosition();
            currentRightEnc = rightMotor.getCurrentPosition();
            telemetry.addData("1 power", power);
            telemetry.addData("2 correction", correction);
            telemetry.addData("3 target", targetRightEnc);
            telemetry.addData("4 right enc", currentRightEnc);
            telemetry.addData("5 error", pidEncoder.m_error);
            telemetry.update();
        } while (opModeIsActive() && !pidEncoder.onTarget());

        sleep(100);
        leftMotor.setPower(0);
        rightMotor.setPower(0);

    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) {
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
        pidRotate.setInputRange(0, 90);
        pidRotate.setOutputRange(.20, power);
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
                leftMotor.setPower(-power);
                rightMotor.setPower(power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                leftMotor.setPower(power);
                rightMotor.setPower(-power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                leftMotor.setPower(power);
                rightMotor.setPower(-power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        rightMotor.setPower(0);
        leftMotor.setPower(0);

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

     /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate_old(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower  = -power;
            rightPower =  power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower  =  power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees + TURNCORR) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees - TURNCORR) {}

        // turn the motors off.
        rightMotor.setPower(0);
        leftMotor.setPower(0);

        // wait for rotation to stop.
        sleep(500);

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
         *
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
         *
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

