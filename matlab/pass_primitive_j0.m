
function [m, T] = pass_primitive_j0(a0, v0, sf, v_min, v_max)
    t = finalOptTimej0(v0, a0, sf);
    v = finalOptVelj0(v0, a0, sf, t);

    % T may be negative and/or complex, this may be quite bad

    if (v_min < v(1) && v(1) < v_max)
        m = evalPrimitiveCoeffs(v0, a0, sf, v(1), 0, t(1));
        T = t(1);
    elseif (v_min < v(2) && v(2) < v_max)
        m = evalPrimitiveCoeffs(v0, a0, sf, v(2), 0, t(2));
        T = t(2);
    else
        error('Could not find a valid pass primitive solution with j0 = 0.');
    end
end

