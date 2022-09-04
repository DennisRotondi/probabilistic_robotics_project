function plot_trajectories(poses,posesest, posesg)
    poses = poses';
    posesg = posesg';
    posesest = posesest';
    figure();
    hold on;
    title("Trajectories comparison");
    plot(reshape(poses(:,1),1,[]), reshape(poses(:,2),1,[]), 'r-', 'linewidth', 3);   
    plot(reshape(posesest(:,1),1,[]), reshape(posesest(:,2),1,[]), 'b-', 'linewidth', 3);   
    plot(reshape(posesg(:,1),1,[]), reshape(posesg(:,2),1,[]), 'g-', 'linewidth', 3);   
    legend("initial guess","after ls","ground truth");
endfunction
