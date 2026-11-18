function m = stop_primitive_j0(v0, a0)
    assert(v0 > 0 && a0 < 0, "Cannot find the stop primitive when v0 is non positive, or a0 is positive");

    T = finalOptTimeStopj0(v0, a0);
    s_max = finalOptPosj0(v0, a0, T);
    m = evalPrimitiveCoeffs(v0, a0, s_max, 0, 0, T);
end

