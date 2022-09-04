addpath 'g2o_wrapper' # <- edited to have range only measurements
addpath 'slam_code' # <- most of my code here
warning('off','all');
[~, poses, transitions, observations] = loadG2o("datasets/slam2d_range_only_initial_guess.g2o");
[landmarksg, posesg, ~, observationsg] = loadG2o("datasets/slam2d_range_only_ground_truth.g2o");
# we flat poses and landmarks
poses = reshape(cell2mat(struct2cell(poses)(2:end, :, :)), 3, []);
posesg = reshape(cell2mat(struct2cell(posesg)(2:end, :, :)), 3, []);
landmarksg = reshape(cell2mat(struct2cell(landmarksg)(2:end, :, :)), 2, []);
# then after init guess use trasition as pose-pose constraint and observations as pose-landmarks positions.
[landmarks, id_to_landmark, landmark_to_id] = compute_initial_guess(poses, observations);
# now that we have the landmarks and the relative associations we can solve the least square problem
niterations = 28;
[poses_est, landmarks_est] = least_square(landmarks, poses, transitions, observations, id_to_landmark, niterations);
# then we finally plot the results
plot_trajectories(poses,poses_est,posesg)
plot_maps(landmarks,landmarks_est,landmarksg)
disp("press a key to end the program")
pause(20)