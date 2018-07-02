package org.frcteam2910.common.util;

public final class PIDConstants {
	public final double p, i, d, f;

	public PIDConstants(double p, double i, double d) {
		this(p, i, d, 0);
	}

	public PIDConstants(double p, double i, double d, double f) {
		this.p = p;
		this.i = i;
		this.d = d;
		this.f = f;
	}
}
