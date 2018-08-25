% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(u_1, eye(3)) + quad_form(u_2, eye(3)) + quad_form(u_3, eye(3)) + quad_form(u_4, eye(3)) + quad_form(u_5, eye(3)) + quad_form(u_6, eye(3)) + quad_form(u_7, eye(3)) + quad_form(u_8, eye(3)) + quad_form(u_9, eye(3)) + quad_form(u_10, eye(3)) + quad_form(u_11, eye(3)) + quad_form(u_12, eye(3)) + quad_form(u_13, eye(3)) + quad_form(u_14, eye(3)) + quad_form(x_15 - xf, Q_final))
%   subject to
%     x_1 == A*x_0 + B*u_0
%     x_2 == A*x_1 + B*u_1
%     x_3 == A*x_2 + B*u_2
%     x_4 == A*x_3 + B*u_3
%     x_5 == A*x_4 + B*u_4
%     x_6 == A*x_5 + B*u_5
%     x_7 == A*x_6 + B*u_6
%     x_8 == A*x_7 + B*u_7
%     x_9 == A*x_8 + B*u_8
%     x_10 == A*x_9 + B*u_9
%     x_11 == A*x_10 + B*u_10
%     x_12 == A*x_11 + B*u_11
%     x_13 == A*x_12 + B*u_12
%     x_14 == A*x_13 + B*u_13
%     x_15 == A*x_14 + B*u_14
%     norm(u_0, inf) <= v_max
%     norm(u_1, inf) <= v_max
%     norm(u_2, inf) <= v_max
%     norm(u_3, inf) <= v_max
%     norm(u_4, inf) <= v_max
%     norm(u_5, inf) <= v_max
%     norm(u_6, inf) <= v_max
%     norm(u_7, inf) <= v_max
%     norm(u_8, inf) <= v_max
%     norm(u_9, inf) <= v_max
%     norm(u_10, inf) <= v_max
%     norm(u_11, inf) <= v_max
%     norm(u_12, inf) <= v_max
%     norm(u_13, inf) <= v_max
%     norm(u_14, inf) <= v_max
%     norm(u_15, inf) <= v_max
%
% with variables
%      u_0   3 x 1
%      u_1   3 x 1
%      u_2   3 x 1
%      u_3   3 x 1
%      u_4   3 x 1
%      u_5   3 x 1
%      u_6   3 x 1
%      u_7   3 x 1
%      u_8   3 x 1
%      u_9   3 x 1
%     u_10   3 x 1
%     u_11   3 x 1
%     u_12   3 x 1
%     u_13   3 x 1
%     u_14   3 x 1
%     u_15   3 x 1
%      x_1   3 x 1
%      x_2   3 x 1
%      x_3   3 x 1
%      x_4   3 x 1
%      x_5   3 x 1
%      x_6   3 x 1
%      x_7   3 x 1
%      x_8   3 x 1
%      x_9   3 x 1
%     x_10   3 x 1
%     x_11   3 x 1
%     x_12   3 x 1
%     x_13   3 x 1
%     x_14   3 x 1
%     x_15   3 x 1
%
% and parameters
%        A   3 x 3
%        B   3 x 3
%  Q_final   3 x 3    PSD
%    v_max   1 x 1    positive
%      x_0   3 x 1
%       xf   3 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.A, ..., params.xf, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2018-08-24 19:17:55 -0400.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
