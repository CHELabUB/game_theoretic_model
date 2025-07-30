function a = OVM(h, v, v1, params)
% if h is negative, it will be kept at h_st
h = max(params.hst, min(h, params.hgo));
v1 = max(0.0, min(v1, params.vmax));
a = params.alpha * (params.kappa * (h - params.hst) - v) + params.beta * (v1 - v);
end

