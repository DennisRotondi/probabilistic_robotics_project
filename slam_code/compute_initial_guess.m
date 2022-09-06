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
# => di^2 - dj^2 - xi^2 - yi^2 + xj^2 + yj^2 = -2(xi-xj)x - 2(yi-yj)*y
# => di^2 - dk^2 - xi^2 - yi^2 + xk^2 + yk^2 = -2(xi-xk)x - 2(yi-yk)*y
# this allows a closed form solution, but we can obtain a more robust result
# if we have n>3 points: we can substract each i = 1,2,...,n-1 equation with
# j = i+1,i+2,..,n and stack them in matrix A so to have a multiple regression. 
# Note that in this case we have n*(n-1)/2 equations --> overdetermined system 
# that admits a pseudoinverse solution i.e. (x y)' = pinv(A)*d

function [landmarks, id_to_landmark, landmark_to_id] = compute_initial_guess(poses, observations)
    landmarks = [];
    # since we do not know how many landmarks we will observe, we allocate a large enough buffer (they are less than 100)
    id_to_landmark = ones(100, 1) * -1;
    landmark_to_id = ones(100, 1) * -1;
    obs_foreach_land = cell;
    num_land = 0;
    # these ids have less than 3 observations, we won't initialize them, just for efficiency reasons 
    # I put them in a constant array, to find them just check the columns of obs_foreach
    id_toskip = [62]; 
    # here we build the vector xi yi di for each pose that observe the landmark to use the regressor
    for i = 1:length(observations)
        obs = observations(i);
        pose_id = obs.pose_id - 1099; # this is a simple conversion I've found checking the datas
        for m = 1:length(obs.observation)
            sob = obs.observation(m);
            id = sob.id;
            d = sob.bearing;
            if id_to_landmark(id) < 0
                if ismember(id, id_toskip) > 0
                    continue
                endif
                new_id = ++num_land;
                id_to_landmark(id) = new_id;
                landmark_to_id(new_id) = id;
                obs_foreach_land{new_id} = [];
            endif
            land_id = id_to_landmark(id);
            obs_foreach_land{land_id}(:, end+1) = [poses(1:2, pose_id); d];
        endfor
    endfor
    # now we solve the regression problem for each landmark
    % disp(size(obs_foreach_land))
    for n = 1:length(obs_foreach_land)
        lobs = obs_foreach_land{n};
        if columns(lobs) < 3
            disp("less than 3 obs for landmark")
            disp(landmark_to_id(n))
            # landmark with id 62 is the only one
        endif
        % disp(size(lobs))
        % disp(n)
        A = [];
        b = [];
        for i = 1:columns(lobs)-1
            [xi yi di] = deal(num2cell(lobs(:,i)){:});
            for j = i+1:columns(lobs)
                [xj yj dj] = deal(num2cell(lobs(:,j)){:});
                b_ = di^2 - dj^2 - xi^2 - yi^2 + xj^2 + yj^2;
                A_ = [2*(xj-xi) 2*(yj-yi)];
                b(end+1) = b_;
                A(end+1, :) = A_;
            endfor
        endfor    
        landmarks(:, end+1) = pinv(A) * b';
    endfor
endfunction