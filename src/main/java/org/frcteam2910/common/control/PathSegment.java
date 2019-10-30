package org.frcteam2910.common.control;

public abstract class PathSegment {
    public abstract Path.State calculate(double distance);

    public abstract double getLength();
}
