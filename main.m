addpath 'g2o_wrapper' # <- edited to have range only measurements
addpath 'slam_code' # <- most of my code here (IGL + LS)
warning('off', 'all');

function plot_maps(landmarks, landmarksest, landmarksg)
    figure();
    hold on;
    title("Maps comparison");
    scatter(reshape(landmarks'(:, 1), 1, []), reshape(landmarks'(:, 2), 1, []), 140);   
    scatter(reshape(landmarksest'(:, 1), 1, []), reshape(landmarksest'(:, 2), 1, []), 140, 'p');   
    scatter(reshape(landmarksg'(:, 1), 1, []), reshape(landmarksg'(:, 2), 1, []), 140, 'd');   
    legend("initial guess", "after ls", "ground truth");
endfunction

function plot_trajectories(poses, posesest, posesg)
    figure();
    hold on;
    title("Trajectories comparison");
    plot(reshape(poses'(:, 1), 1, []), reshape(poses'(:, 2), 1, []), 'r-', 'linewidth', 3);   
    plot(reshape(posesest'(:, 1), 1, []), reshape(posesest'(:, 2), 1, []), 'b-', 'linewidth', 3);   
    plot(reshape(posesg'(:, 1), 1, []), reshape(posesg'(:, 2), 1, []), 'g-', 'linewidth', 3);   
    legend("initial guess", "after ls", "ground truth");
endfunction

# here we load the IG and GD using the G2o wrapper:
[~, poses, transitions, observations] = loadG2o("datasets/slam2d_range_only_initial_guess.g2o");
[landmarksg, posesg, ~, observationsg] = loadG2o("datasets/slam2d_range_only_ground_truth.g2o");
# we flat poses and landmarks to properly suit the state removing ids
poses = reshape(cell2mat(struct2cell(poses)(2:end, :, :)), 3, []);
posesg = reshape(cell2mat(struct2cell(posesg)(2:end, :, :)), 3, []);
landmarksg = reshape(cell2mat(struct2cell(landmarksg)(2:end, :, :)), 2, []);
# we compute the initial guess for landmarks position using the observations from a given pose
[landmarks, id_to_landmark, landmark_to_id] = compute_initial_guess(poses, observations);
# now that we have the landmarks and the relative associations we can solve the least square problem
niterations = 26;
[poses_est, landmarks_est] = least_square(landmarks, poses, transitions, observations, id_to_landmark, niterations);
# then we repeat this process but with a partial knowledge of the state (only the first 3 positions)
[poses_est_pk, landmarks_est_pk] = least_square_partial_knowledge(landmarks, poses, transitions, observations, id_to_landmark, niterations, posesg(:, 1:3));
# eventually we plot the results
plot_trajectories(poses, poses_est, posesg)
plot_maps(landmarks, landmarks_est, landmarksg)
plot_trajectories(poses, poses_est_pk, posesg)
plot_maps(landmarks, landmarks_est_pk, landmarksg)
disp("figures 3,4 are with partial knowledge, press a key to end the program")
pause