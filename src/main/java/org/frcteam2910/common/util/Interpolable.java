package org.frcteam2910.common.util;

public interface Interpolable<T> {
    T interpolate(T other, double t);
}
