% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(u_0, eye(3)) + quad_form(u_1, eye(3)) + quad_form(u_2, eye(3)) + quad_form(u_3, eye(3)) + quad_form(u_4, eye(3)) + quad_form(u_5, eye(3)) + quad_form(u_6, eye(3)) + quad_form(u_7, eye(3)) + quad_form(u_8, eye(3)) + quad_form(u_9, eye(3)) + quad_form(u_10, eye(3)) + quad_form(u_11, eye(3)) + quad_form(u_12, eye(3)) + quad_form(u_13, eye(3)) + quad_form(u_14, eye(3)) + quad_form(x_15 - xf, Q_final))
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
%     norm(u_0, inf) <= a_max
%     norm(u_1, inf) <= a_max
%     norm(u_2, inf) <= a_max
%     norm(u_3, inf) <= a_max
%     norm(u_4, inf) <= a_max
%     norm(u_5, inf) <= a_max
%     norm(u_6, inf) <= a_max
%     norm(u_7, inf) <= a_max
%     norm(u_8, inf) <= a_max
%     norm(u_9, inf) <= a_max
%     norm(u_10, inf) <= a_max
%     norm(u_11, inf) <= a_max
%     norm(u_12, inf) <= a_max
%     norm(u_13, inf) <= a_max
%     norm(u_14, inf) <= a_max
%     norm(u_15, inf) <= a_max
%     max(abs(x_1(4)),abs(x_1(5)),abs(x_1(6))) <= v_max
%     max(abs(x_2(4)),abs(x_2(5)),abs(x_2(6))) <= v_max
%     max(abs(x_3(4)),abs(x_3(5)),abs(x_3(6))) <= v_max
%     max(abs(x_4(4)),abs(x_4(5)),abs(x_4(6))) <= v_max
%     max(abs(x_5(4)),abs(x_5(5)),abs(x_5(6))) <= v_max
%     max(abs(x_6(4)),abs(x_6(5)),abs(x_6(6))) <= v_max
%     max(abs(x_7(4)),abs(x_7(5)),abs(x_7(6))) <= v_max
%     max(abs(x_8(4)),abs(x_8(5)),abs(x_8(6))) <= v_max
%     max(abs(x_9(4)),abs(x_9(5)),abs(x_9(6))) <= v_max
%     max(abs(x_10(4)),abs(x_10(5)),abs(x_10(6))) <= v_max
%     max(abs(x_11(4)),abs(x_11(5)),abs(x_11(6))) <= v_max
%     max(abs(x_12(4)),abs(x_12(5)),abs(x_12(6))) <= v_max
%     max(abs(x_13(4)),abs(x_13(5)),abs(x_13(6))) <= v_max
%     max(abs(x_14(4)),abs(x_14(5)),abs(x_14(6))) <= v_max
%     max(abs(x_15(4)),abs(x_15(5)),abs(x_15(6))) <= v_max
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
%      x_1   6 x 1
%      x_2   6 x 1
%      x_3   6 x 1
%      x_4   6 x 1
%      x_5   6 x 1
%      x_6   6 x 1
%      x_7   6 x 1
%      x_8   6 x 1
%      x_9   6 x 1
%     x_10   6 x 1
%     x_11   6 x 1
%     x_12   6 x 1
%     x_13   6 x 1
%     x_14   6 x 1
%     x_15   6 x 1
%
% and parameters
%        A   6 x 6
%        B   6 x 3
%  Q_final   6 x 6    PSD
%    a_max   1 x 1    positive
%    v_max   1 x 1    positive
%      x_0   6 x 1
%       xf   6 x 1
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
% Produced by CVXGEN, 2018-08-30 09:34:28 -0400.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
