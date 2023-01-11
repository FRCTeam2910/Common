package org.frcteam2910.common.util;

public interface InverseInterpolatable<T> {
    double inverseInterpolate(T upper, T query);
}
