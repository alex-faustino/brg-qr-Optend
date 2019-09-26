function [full, noPara, noProf, noInd, noClimb] = estimatePower(v_inf, vz, r, p, params)
% Returns instantaneous power estimate based on airspeed and orientation
% assumes level flight and planar wind

% short hand for trig operations
sp = sin(p);
cp = cos(p);
sr = sin(r);
cr = cos(r);

% Find induced velocity
v_i_poly_coeff = [1
                  2*v_inf*cr*cp 
                  v_inf^2 
                  0
                  -params.v_h^4];
v_i_soln = roots(v_i_poly_coeff);
if real(v_i_soln(4)) > real(v_i_soln(1))
    v_i = real(v_i_soln(4));
else
    v_i = real(v_i_soln(1));
end

% Find total thrust assuming level flight
T = params.m*params.g/cr/cp;

% Find drag force using isoropic drag model
f_D = (params.mu1*v_inf + params.mu2*v_inf^2 + params.mu3*v_inf^3);

parasitic = f_D*v_inf;
induced = T*(v_i + v_inf*sr*sp);
profile = params.k*T^(3/2);
climb = -vz*params.m*params.g;

full = 1/params.eta*(induced + parasitic + profile + climb);
noPara = 1/params.eta*(induced + profile + climb);
noProf = 1/params.eta*(induced + parasitic + climb);
noInd = 1/params.eta*(parasitic + profile + climb);
noClimb = 1/params.eta*(induced + parasitic + profile);
end

