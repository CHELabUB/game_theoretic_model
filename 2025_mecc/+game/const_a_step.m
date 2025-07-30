function new_state = const_a_step(s, v, a, dt, v_max, v_min)
% a generic calculation of constant acceleration but with speed limit in mind
% note s does not really matter. only v and a does
% The current calculation is assuming v >=0, always travel in the direction
% where s is increasing.
% tested with testConstAStep

new_v = v + a * dt;
if new_v > v_max
    new_v = v_max;
    t1 = (new_v - v) / a;
    delta_s = (new_v^2 - v^2) / a / 2 + new_v * (dt - t1);
else
    if new_v < v_min
        new_v = v_min;
        t1 = (new_v - v) / a;
        delta_s = (new_v^2 - v^2) / a / 2 + new_v * (dt - t1);
    else
        delta_s = v * dt + a * 0.5 * dt^2;
    end
end
new_state = [s + delta_s;
                    new_v];
end