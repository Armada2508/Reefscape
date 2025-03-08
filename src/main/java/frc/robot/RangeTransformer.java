package frc.robot;

import edu.wpi.first.math.MathUtil;

/**
 * Helper class for mapping the range [a,b] onto the range [c,d]
 */
public class RangeTransformer {

    public final double a;
    public final double b;
    public final double c;
    public final double d;
    public final boolean clamp;
        
        public RangeTransformer(double a, double b, double c, double d, boolean clamp) {
        if (a == b) throw new IllegalArgumentException("A != B");
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
        this.clamp = clamp;
    }

    /**
     * Takes a number in the range [a,b] and puts it in the range [c,d]
     * @param x number in the range [a,b]
     * @return number in the range [c,d]
     */
    public double calculate(double x) {
        double result = (c + ((d - c)/(b - a)) * (x - a));
        if (clamp) return MathUtil.clamp(result, Math.min(c, d), Math.max(c, d));
        return result;
    }

}
