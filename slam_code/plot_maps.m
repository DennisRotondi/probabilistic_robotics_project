function plot_maps(landmarks,landmarksest,landmarksg)
    landmarks = landmarks';
    landmarksg = landmarksg';
    landmarksest = landmarksest';
    figure();
    hold on;
    title("Maps comparison");
    scatter(reshape(landmarks(:,1),1,[]), reshape(landmarks(:,2),1,[]), 140);   
    scatter(reshape(landmarksest(:,1),1,[]), reshape(landmarksest(:,2),1,[]), 140, 'p');   
    scatter(reshape(landmarksg(:,1),1,[]), reshape(landmarksg(:,2),1,[]), 140, 'd');   
    legend("initial guess","after ls","ground truth");
endfunction