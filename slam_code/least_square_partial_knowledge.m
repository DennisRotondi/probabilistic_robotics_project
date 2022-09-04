function [poses_est, landmarks_est] = least_square_partial_knowledge(landmarks, poses, transitions, observations, id_to_landmark, niterations, posesg)
    allp= prod(size((poses)));
    dim = allp+prod(size((landmarks)));
    poses(:,1:3)=posesg(:,1:3); #NOW WE FIX 3 POINTS SINCE THE PROBLEM AS IT IS HAS 6DOF
    for iteration=1:niterations
        H=zeros(dim,dim);  b=zeros(dim,1); %accumulators for H and b
        chi_tot=0; %cumulative chi2
        for i=1:length(observations)
            obs = observations(i);
            pose_id = obs.pose_id-1099; # this is a simple conversion I've found checking the datas
            for j =1:length(obs.observation)
                z = obs.observation(j);
                id = z.id;
                d = z.bearing;
                idl = id_to_landmark(id);
                if idl<0 
                    % printf("found a not initialized landmark %d\n", id)
                    continue
                endif
                [e, Jr, Jl] = errorAndJacobianLP(landmarks(:,idl), poses(:,pose_id), d);
                chi=e'*e;
                k_treshold = 1.4;
                if chi > k_treshold
                    e *= sqrt(k_treshold/chi);
                    chi = k_treshold;
                endif
                chi_tot+=chi;

                rhidx = 1+(pose_id-1)*3;
                lhidx = 1+allp+(idl-1)*2;
                
                H(rhidx:rhidx+2, rhidx:rhidx+2) += Jr'*Jr;
                H(lhidx:lhidx+1, lhidx:lhidx+1) += Jl'*Jl;
                Jrl=Jr'*Jl;
                H(rhidx:rhidx+2, lhidx:lhidx+1) += Jrl;
                H(lhidx:lhidx+1, rhidx:rhidx+2) += Jrl';

                b(rhidx:rhidx+2) += Jr'*e;
                b(lhidx:lhidx+1) += Jl'*e;              
            endfor
        endfor 
        for i=1:length(transitions)
            trans = transitions(i);
            pi_id = trans.id_from-1099;
            pj_id = trans.id_to-1099;
            z = trans.v;
            [e, Ji, Jj] = errorAndJacobianPP(poses(:,pi_id), poses(:,pj_id), z);
            k_treshold = 2;
                if chi > k_treshold
                    e *= sqrt(k_treshold/chi);
                    chi = k_treshold;
                endif
                chi_tot+=chi;
            iidx = 1+(pi_id-1)*3;
            jidx = 1+(pj_id-1)*3;
            H(iidx:iidx+2, iidx:iidx+2) += Ji'*Ji;
            H(jidx:jidx+2, jidx:jidx+2) += Jj'*Jj;
            jij=Ji'*Jj;
            H(iidx:iidx+2, jidx:jidx+2) += jij;
            H(jidx:jidx+2, iidx:iidx+2) += jij';
            b(iidx:iidx+2) += Ji'*e;
            b(jidx:jidx+2) += Jj'*e;   
        endfor
        # now we need to fix the first 3 poses that we "locked"
        dx = zeros(dim,1);
        H=H(4:end,4:end);
        b=b(4:end);
        tdx = -H\b;
        dx(4:end)=tdx;
        [poses, landmarks] = boxplus(poses,landmarks,dx, allp);
        printf("chi after iteration %d: %f\n",iteration, chi_tot )
    endfor        
    poses_est = poses;
    landmarks_est = landmarks;
endfunction

function [e, Jr, Jl] = errorAndJacobianLP(land, pos, z)
    # standard jacobian for range obs as seen
    Xr = v2t(pos);
    R = Xr(1:2,1:2);
    t = Xr(1:2,3);
    ph = R' * (land-t);
    zh = norm(ph);
    e = zh - z;    

    Ji = zeros(2,3);
    Ji(1:2,1:2) = -R';
    Ji(1:2,3) = R' * [0 1;-1 0] * land;

    Jr = zeros(1,3);
    Jr = (1/zh) * ph' * Ji;
    Jl = (1/zh) * ph' * R';
endfunction

function [e, Ji, Jj] = errorAndJacobianPP(pi, pj, z)
# here we apply the flattening technique seen for pose-pose contraints
    Xi=v2t(pi);
    Xj=v2t(pj);
    Z=v2t(z);
    df = Xi^-1*Xj;
    e = reshape(df(1:2,:),[],1) - reshape(Z(1:2,:),[],1);
    dRx0 = [0 -1; 1 0];
    Ri = Xi(1:2,1:2);
    Rj = Xj(1:2,1:2);
    tj = Xj(1:2,3);

    rx = reshape(Ri'*dRx0*Rj, [], 1);
    stj = -Ri'*dRx0*tj;
    Jj = zeros(6,3);
    Jj(:,3) = [rx; stj];
    Jj(5:6,1:2) = Ri';
    Ji = -Jj;
endfunction

function [np, nl] = boxplus(poses,landmarks,dx,allp);
    updatep = dx(1:allp);
    updatel = dx(allp+1:end);
    #update poses
    upvp = reshape(updatep,3,[]);
    np = poses;
    for i=1:columns(upvp)
        dxi = upvp(:,i);
        np(:,i) = t2v(v2t(dxi)*v2t(poses(:,i)));
    endfor
    #update landmarks
    nl = landmarks + reshape(updatel, 2, []);
endfunction
% computes the pose 2d pose vector v from an homogeneous transform A
% A:[ R t ] 3x3 homogeneous transformation matrix, r translation vector
% v: [x,y,theta]  2D pose vector
function v=t2v(A)
	v(1:2, 1)=A(1:2,3);
	v(3,1)=atan2(A(2,1),A(1,1));
end

% computes the homogeneous transform matrix A of the pose vector v
% A:[ R t ] 3x3 homogeneous transformation matrix, r translation vector
% v: [x,y,theta]  2D pose vector
function A=v2t(v)
  	c=cos(v(3));
  	s=sin(v(3));
	A=[c, -s, v(1) ;
	s,  c, v(2) ;
	0   0  1  ];
end