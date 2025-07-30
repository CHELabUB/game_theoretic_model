clc;clear; close all;
Util.plot_setup();
%% Compare ovm and speed regulation for stop
% Note that for intersection case where s_stop is usually far
% it is OK to use OVM. It would be problematic if it is far away.
v_max = 20;
v_min = 0;
ovm_params.alpha = 0.4;
ovm_params.beta = 0.5;
% to make the two equivalent
ovm_params.alpha_v = ovm_params.alpha + ovm_params.beta;
ovm_params.kappa = 0.6;
ovm_params.hst = 5;
ovm_params.vmax = v_max;
ovm_params.hgo = ovm_params.hst + ovm_params.vmax / ovm_params.kappa;
dt = 0.1;
full_sample_len = 100;
t = (0:1:full_sample_len) * dt;

s_trace = zeros(2, full_sample_len + 1);
u_trace = zeros(1, full_sample_len);
u_trace2 = u_trace;
v0 = 4;
v_target = 0;
s_stop = 16;
s_trace(:, 1) = [0; v0];
s_trace2 = s_trace;
for i = 1:full_sample_len
    u_trace(:, i) = game.speed_control(s_trace(2, i), v_target, ovm_params);
    s_trace(:, i + 1) = game.const_a_step(s_trace(1, i), s_trace(2, i), u_trace(:, i), dt, v_max, v_min);
    if v_target > 0
        h = v_target / ovm_params.kappa + ovm_params.hst;
    else
        h = s_stop + ovm_params.hst - s_trace2(1, i);
    end
    u_trace2(:, i) = game.OVM(h, s_trace2(2, i), v_target, ovm_params);
    s_trace2(:, i + 1) = game.const_a_step(s_trace2(1, i), s_trace2(2, i), u_trace2(:, i), dt, v_max, v_min);
end

figure(1)
subplot(3,1,1);hold on;
plot(t, s_trace(1, :), "b")
plot(t, s_trace2(1, :), "r--")
plot(t, s_stop * ones(size(t)), "k--")
hold off;
subplot(3,1,2);hold on;
plot(t, s_trace(2, :), "b")
plot(t, s_trace2(2, :), "r--")
hold off;
subplot(3,1,3);hold on;
plot(t(1:end-1), u_trace, "b")
plot(t(1:end-1), u_trace2, "r--")
hold off;
