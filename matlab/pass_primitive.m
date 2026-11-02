
function [m1, v1, T1, m2, v2, T2] = pass_primitive(a0, v0, sf, v_min, v_max, T_min, T_max)
    assert(sf > 0, "sf needs to be positive");
    assert(v_min <= v_max, "v_min needs to be less that or equal to v_max");
    assert(T_min <= T_max, "T_min needs to be less that or equal to T_max");

    T_v_min = finalOptTime(v0, a0, sf, v_min);
    T_v_max = finalOptTime(v0, a0, sf, v_max);

    if a0 < 0
        v_star = v_star_calculation(a0, sf, v0);

        if v_star > v_max
            T_v_min = 0;
            T_v_max = 0;
        elseif v_star > v_min % Within the interval
            T_v_min = T_star_calculation(a0, sf);
        end
    end

    T1 = max(T_v_max, T_min);
    T2 = min(T_v_min, T_max);
    % Need to verify that the interval is valid
    assert(0 < T1 && T1 <= T2, "The resulting interval needs to be valid");

    v1 = finalOptVel(v0, a0, sf, T1);
    v2 = finalOptVel(v0, a0, sf, T2);

    m1 = evalPrimitiveCoeffs(v0, a0, sf, v1, 0, T1);
    m2 = evalPrimitiveCoeffs(v0, a0, sf, v2, 0, T2);
end


