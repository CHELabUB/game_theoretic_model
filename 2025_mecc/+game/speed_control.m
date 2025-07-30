function a = speed_control(v, v_target, param)
a = param.alpha_v * (v_target - v);
end