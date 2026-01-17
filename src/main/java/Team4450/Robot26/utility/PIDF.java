package Team4450.Robot26.utility;

public class PIDF {
    private double p;
    private double i;
    private double d;
    private double f;
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double maxValue;
    private double minValue;
    private double previousTime;
    private double deltaTime;
    private double deltaError;

    public PIDF(double kP, double kI, double kD, double kF, double maxValue, double minValue) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.maxValue = maxValue;
        this.minValue = minValue;
    }

    public double calculate(double error, double currentTime) 
    {
        deltaTime = currentTime - previousTime;
        deltaError = error - (previousTime == 0 ? 0 : error);

        p = kP * error;
        i += kI * error * deltaTime;
        d = (deltaTime != 0) ? kD * (deltaError / deltaTime) : 0;
        f = kF * Math.signum(error);

        previousTime = currentTime;

        return Math.max(minValue, Math.min(maxValue, p + i + d + f));
    }
}
