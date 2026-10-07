function du = baseline_pilot(xd, P)
%BASELINE_PILOT  Simple small-angle PD stabiliser used ONLY while collecting data.
%   A free quadrotor is unstable, so flight data is always recorded with some
%   stabilising controller in the loop (here: the role ArduPilot plays).
%   Random excitation is added on top of it in Step 1 so that DMDc can still
%   tell the dynamics (A) apart from the controller.
%
%   INPUT : xd [12x1] deviation state [dp; v; theta; w],  P params
%   OUTPUT: du [4x1] deviation input [dT (N); tau (N m)] relative to hover
a_cmd = -1.0*xd(1:3) - 1.5*xd(4:6);                       % desired acceleration
th_d  = [-a_cmd(2)/P.g0; a_cmd(1)/P.g0; 0];               % tilt that produces it
du    = [P.m*a_cmd(3);
         P.J*(-36*(xd(7:9) - th_d) - 10*xd(10:12))];
end
