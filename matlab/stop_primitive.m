
function [m, sf_max, tf] = stop_primitive(v0, a0, sf)
    assert(v0 > 0 && sf > 0, "Cannot find the stop primitive when sf or v0 is non positive");

    sf_max = max(sf, -4 * v0^2 / (5 * a0));
    tf = finalOptTimeStop(v0, a0, sf_max);
    m = evalPrimitiveCoeffs(v0, a0, sf_max, 0, 0, tf);
end
