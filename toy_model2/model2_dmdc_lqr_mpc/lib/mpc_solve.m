function [u0, U, info] = mpc_solve(mpc, x0)
%MPC_SOLVE  Solve the stacked MPC QP for the current state and return the first input.
%
%   min_U 1/2 U'HU + (F x0)'U   s.t.  G U <= h0 + E x0
%
%   If the state constraints cannot be met from x0 (e.g. after a gust),
%   they are dropped for this sample and only the input limits are kept.
%
%   INPUT : mpc struct from MPC_BUILD, x0 [nx1] current (deviation) state
%   OUTPUT: u0 [mx1] first optimal input u_0*,  U [mNx1] whole optimal sequence,
%           info from QP_SOLVE plus info.relaxed (true if state limits were dropped)
m = size(mpc.B, 2);
f = mpc.F*x0;
[U, info] = qp_solve(mpc.H, f, mpc.G, mpc.h0 + mpc.E*x0);
info.relaxed = false;
if ~info.converged
    rows = 1:mpc.nInputRows;
    [U, info] = qp_solve(mpc.H, f, mpc.G(rows,:), mpc.h0(rows));
    info.relaxed = true;
end
u0 = U(1:m);
end
