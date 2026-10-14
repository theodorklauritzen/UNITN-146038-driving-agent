
function [sf_max, tf] = stop_primitive(v0, a0, sf)
    if v0 <= 0 || sf <= 0
        error("Cannot find the stop primitive when sf or v0 is non positive");
    end

    sf_max = max(sf, -4 * v0^2 / (5 * a0));
    tf = finalOptTimeStop(v0, a0, sf_max);
end
