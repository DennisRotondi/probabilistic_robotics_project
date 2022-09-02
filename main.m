addpath 'tools/g2o_wrapper'
addpath 'tools/visualization'
[~, poses, transitions, observations] = loadG2o("datasets/slam2d_range_only_initial_guess.g2o");
[landmarksg, posesg, transitionsg, observationsg] = loadG2o("datasets/slam2d_range_only_ground_truth.g2o");

disp(poses);