package org.firstinspires.ftc.teamcode;

/**
 * @author Zack Horton
 * @version 1.0
 * @since 1.0
 */
public class PIDFunctions{
    private double last = 0;
    private double errorSum = 0;
    private double current = 0;
    private double target = 0;
    private double kp = 0;
    private double ki = 0;
    private double kd = 0;
    private double range = 0;
    private double timeIn = 0;
    private double timeInInput = 0;

    public PIDFunctions(double c, double t, double p, double i, double d, double r, double in){
        current = c;
        target = t;
        kp = p;
        ki = i;
        kd = d;
        range = r;
        timeIn = 0;
        timeInInput = in;
    }

    public void setCurrent (double c){
        current = c;
    }

    public double getError()
    {
        return current - target;
    }

    public double output() {
        if(timeIn > timeInInput)
        {
            return 0;
        }

        double error = current - target;

        if(Math.abs(error) < range) {
            errorSum = 0;
            timeIn++;
        }
        else {
            errorSum += error;
            timeIn = 0;
        }


        double proportionalTerm = kp * error;
        double integralTerm = ki * errorSum;
        double derivativeTerm = kd * (error - last);

        last = current;

        double result = proportionalTerm + integralTerm + derivativeTerm;

        if (result == 0)
        {
            result += .0000001;
        }

        return result;
    }
}
