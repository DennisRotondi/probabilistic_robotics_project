addpath 'tools/g2o_wrapper'
addpath 'tools/utilities'
addpath 'tools/visualization'

[~, poses, transitions, observations] = loadG2o("datasets/slam2d_range_only_initial_guess.g2o");
[landmarksg, posesg, transitionsg, observationsg] = loadG2o("datasets/slam2d_range_only_ground_truth.g2o");


% for obs in observations:

# in our problem we do not have landmarks positions, we need to solve 
# a data reconciliation problem so to have a good initial guess. I'll
# solve a linear system exploiting the range of the measurements we have.

# https://en.wikipedia.org/wiki/True-range_multilateration
# first we need to find at least three observations i,j,k for each landmark (x y)'
# di^2 = (xi-x)^2+(yi-y)^2
# dj^2 = (xj-x)^2+(yj-y)^2
# dk^2 = (xk-x)^2+(yk-y)^2
# so:
# di^2 - dj^2 = -2xi*x + xi^2 - 2yi*y + yi^2 + 2xj*x - xj^2 + 2yj*y - yj^2
# => di^2 - dj^2 - xi^2 - yi^2 + xj^2 + yj^2 = -2(xi-xj)x - 2(yi-xj)*y
# => di^2 - dk^2 - xi^2 - yi^2 + xk^2 + yk^2 = -2(xi-xk)x - 2(yi-xk)*y
# this allows a closed form solution, but we can obtain a more robust result
# if we have n>3 points: we can substract each i = 1,2,...,n-1 equation with
# j = i+1,i+2,..,n and stack them in matrix A so to have a multiple regression. 
# Note that in this case we have n*(n-1)/2 equations --> overdetermined system 
# that admit a pseudoinverse solution i.e. (x y)' = pinv(A)*d

# then after init guess use trasition as pose-pose constraint and observations as pose-landmarks positions.


disp(observations(105))
% disp(transitionsg(105))