package frc.robot;

/**
 * Helper class for mapping the range [a,b] onto the range [c,d]
 */
public class RangeTransformer {

    public final double a;
    public final double b;
    public final double c;
    public final double d;
    
    public RangeTransformer(double a, double b, double c, double d) {
        if (a == b) throw new IllegalArgumentException("A != B");
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }

    /**
     * Takes a number in the range [a,b] and puts it in the range [c,d]
     * @param x number in the range [a,b]
     * @return number in the range [c,d]
     */
    public double calculate(double x) {
        return (c + ((d - c)/(b - a)) * (x - a));
    }

}
