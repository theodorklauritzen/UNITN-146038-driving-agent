
function [m, sf_max, tf] = stop_primitive(v0, a0, sf)
    assert(v0 > 0 && sf > 0, "Cannot find the stop primitive when sf or v0 is non positive");

    if 4 * v0^2 + 5 * a0 * sf < 0
        sf_max = -4 * v0^2 / (5 * a0);
        tf = 5 * sf / (v0);
    else
        sf_max = sf;
        tf = finalOptTimeStop(v0, a0, sf_max);
    end
    m = evalPrimitiveCoeffs(v0, a0, sf_max, 0, 0, tf);
end
