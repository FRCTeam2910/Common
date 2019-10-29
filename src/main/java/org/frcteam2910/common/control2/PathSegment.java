package org.frcteam2910.common.control2;

public abstract class PathSegment {
    public abstract Path.State calculate(double distance);

    public abstract double getLength();
}
