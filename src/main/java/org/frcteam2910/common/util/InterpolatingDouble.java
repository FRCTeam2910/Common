package org.frcteam2910.common.util;

import java.io.Serializable;

public class InterpolatingDouble implements Interpolable<InterpolatingDouble>, InverseInterpolable<InterpolatingDouble>, Comparable<InterpolatingDouble>, Serializable {
    private static final long serialVersionUID = -4236134181182630613L;

    public Double value;

    public InterpolatingDouble(Double value) {
        this.value = value;
    }

    @Override
    public int compareTo(InterpolatingDouble other) {
        return Double.compare(value, other.value);
    }

    @Override
    public InterpolatingDouble interpolate(InterpolatingDouble other, double t) {
        double delta = other.value - value;
        return new InterpolatingDouble(value + delta * t);
    }

    @Override
    public double inverseInterpolate(InterpolatingDouble upper, InterpolatingDouble query) {
        double delta = upper.value - value;
        double deltaQuery = query.value - value;

        return deltaQuery / delta;
    }
}
