package org.frcteam2910.common.control;

public final class PidConstants {
    public final double p;
    public final double i;
    public final double d;

    public PidConstants(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }
}
