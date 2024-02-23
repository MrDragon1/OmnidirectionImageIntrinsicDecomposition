function [I,R]=intrinsicScene_color_L1(src,depth_data, normal_data, shadowmask, param)
    addpath('./utils/matlab');
    addpath('./utils/npy-matlab/npy-matlab');
    nn_num_s = 20;
    if param.NonLocal 
        lambda_Ns = 0.005;
    else
        lambda_Ns = 0;
    end

    if param.Intensity
        lambda_a=0.02;
    else
        lambda_a = 0;
    end

    if param.Data
        lambda = 0.3;
    else
        lambda = 1;
    end
    lambda_r=0.005;
    
    vareps=0.01;
    K=20;

    [h, w,~] = size(src);
    point = get_pointcloud(depth_data);
    lambda(lambda<0)=0;
  
    S_hat = shadowmask;

    if param.HalfSest
        S_hat = ones(h,w,3).*0.5;
    end

    if param.ISest
        S_hat = repmat(im2gray(src),[1 1 3]);
    end
    
    I=ones(size(src));
    R=src./I;

    Ns = NonLocalConstraint_KNN(src,normal_data,point,nn_num_s,shadowmask);
    for iter=1:K
        preI=I;
        preR=R;
        %% update reflectance
        R=src./(I+1e-3);

        R=L0Smoothing(R,lambda_r);
        R(R>1)=1; R(R<0)=0;
        eplisonR=norm(R-preR,'fro')/norm(preR,'fro');

        %% update illuination
        Ir=solveLinearSystem(src(:,:,1),R(:,:,1),Ns,S_hat(:,:,1),lambda_Ns,lambda_a,lambda);
        Ig=solveLinearSystem(src(:,:,2),R(:,:,2),Ns,S_hat(:,:,2),lambda_Ns,lambda_a,lambda);
        Ib=solveLinearSystem(src(:,:,3),R(:,:,3),Ns,S_hat(:,:,3),lambda_Ns,lambda_a,lambda);
        I = cat(3,Ir,Ig,Ib);
        I = max(I,src); I(I>1)=1;
        eplisonI=norm(I-preI,'fro')/norm(preI,'fro');
        if(iter > 2 && (eplisonI<vareps || eplisonR<vareps))
            break;
        end

    end
end

function dst = solveLinearSystem(s, r, Ns, s_hat,lambda_Ns,lambda_a,lambda)
    [h, w] = size(s);
    hw = h * w;
    DEN = lambda_Ns * Ns + (lambda(:) + lambda_a(:)) .* speye(hw,hw);
    NUM = lambda .* s./(r+1e-3) + lambda_a .* s_hat;
    L = ichol(DEN,struct('michol','on'));
    [dst,~] = pcg(DEN, NUM(:), 0.01, 40, L, L');
    dst = reshape(dst, h, w);
end

%% this function is from
% Xu, L., Lu, C., Xu, Y., & Jia, J., Image smoothing via $l_0$ gradient
% minimization, Transactions on Graphics, 30(6), 174 (2011).
function S = L0Smoothing(Im, lambda, kappa)
    if ~exist('kappa','var')
        kappa = 2.0;
    end
    if ~exist('lambda','var')
        lambda = 2e-2;
    end
    S = im2double(Im);
    betamax = 1e5;
    fx = [1, -1];
    fy = [1; -1];
    [N,M,D] = size(Im);
    sizeI2D = [N,M];
    otfFx = psf2otf(fx,sizeI2D);
    otfFy = psf2otf(fy,sizeI2D);
    Normin1 = fft2(S);
    Denormin2 = abs(otfFx).^2 + abs(otfFy ).^2;
    if D>1
        Denormin2 = repmat(Denormin2,[1,1,D]);
    end
    beta = 2*lambda;
    while beta < betamax
        Denormin  = 1 + beta.*Denormin2;
        % h-v subproblem
        h = [diff(S,1,2), S(:,1,:) - S(:,end,:)];
        v = [diff(S,1,1); S(1,:,:) - S(end,:,:)];
        if D==1
            t = (h.^2+v.^2)<lambda./beta;
        else
            t = sum((h.^2+v.^2),3)<lambda./beta; %sum((h.^2+v.^2),3)
            t = repmat(t,[1,1,D]);
        end
        h(t)=0; v(t)=0;
        % S subproblem
        Normin2 = [h(:,end,:) - h(:, 1,:), -diff(h,1,2)];
        Normin2 = Normin2 + [v(end,:,:) - v(1, :,:); -diff(v,1,1)];
        FS = (Normin1 + beta.*fft2(Normin2))./Denormin;
        S = real(ifft2(FS));
        beta = beta.*kappa;
    end
end

function [camera_points] = get_pointcloud(depth_data)
    [h,w,~] = size(depth_data);
    [pixel_x, pixel_y] = meshgrid(0:w-1,0:h-1);
    theta = pixel_x * 2.0 * pi / w - pi;
    phi = (h - pixel_y-0.5) * pi / h - pi / 2;
    
    camera_points_x = depth_data .* cos(phi) .* sin(theta);
    camera_points_y = depth_data .* sin(phi);
    camera_points_z = depth_data .* cos(phi) .* cos(theta);
    
    camera_points = cat(3,camera_points_x,camera_points_y,camera_points_z);
end
