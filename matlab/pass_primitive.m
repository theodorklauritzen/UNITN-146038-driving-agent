
function [m1, v1, T1, m2, v2, T2] = pass_primitive(a0, v0, sf, v_min, v_max, T_min, T_max)
    assert(sf > 0, "sf needs to be positive");
    assert(v_min <= v_max, "v_min needs to be less that or equal to v_max");
    assert(T_min <= T_max, "T_min needs to be less that or equal to T_max");
    m1 = nan(1,5); v1 = nan; T1 = nan;
    m2 = nan(1,5); v2 = nan; T2 = nan;
    if sf < 0
        return;
    end
    if v_min > v_max
        return;
    end
    if T_min > T_max
        return;
    end

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

    t1 = max(T_v_max, T_min);
    t2 = min(T_v_min, T_max);
    % Need to verify that the interval is valid
    assert(0 < t1 && t1 <= t2, "The resulting interval needs to be valid");
    if t1 <= 0 || t1 > t2
        return;
    end

    T1 = t1;
    T2 = t2;

    v1 = finalOptVel(v0, a0, sf, t1);
    v2 = finalOptVel(v0, a0, sf, t2);

    m1 = evalPrimitiveCoeffs(v0, a0, sf, v1, 0, t1);
    m2 = evalPrimitiveCoeffs(v0, a0, sf, v2, 0, t2);
end


