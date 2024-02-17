function [Sest,lightsourcemask,lightsourcecolor]=getShadowMask(im,depth_data,normal_data,segment)
    addpath('utils/matlab');
    addpath('utils/npy-matlab/npy-matlab');
    %% z->front y->up x->right
    %% Real/Synthetic x->front y->up z->left
%     path = 'dataset\synthetic\livingroom';
%     im=im2double(imread([path '\up.png']));
%     normal_data = im2double(exrread([path '\normal.exr']));
%     normal_data_x = normal_data(:,:,1);normal_data_y = normal_data(:,:,2);normal_data_z = normal_data(:,:,3);
%     normal_data = cat(3, -normal_data_z, normal_data_y, normal_data_x);
%     depth_data = exrread([path '\depth.exr']);
%     depth_data = depth_data(:,:,1);
%     
    %%  y->front z->up x->right
    % im=im2double(imread(['dataset\' path '\data\' path '_color_0_Left_Down_0.0.png']));
    % normal_data = im2double(exrread(['dataset\' path '\data\' path '_normal_0_Left_Down_0.0.exr']));
    % normal_data_x = normal_data(:,:,1);normal_data_y = normal_data(:,:,2);normal_data_z = normal_data(:,:,3);
    % normal_data = cat(3, normal_data_x, normal_data_z, normal_data_y);
    % depth_data = im2double(exrread(['dataset\' path '\data\' path '_depth_0_Left_Down_0.0.exr']));
    % depth_data = depth_data(:,:,1);
    
    %% Structure3D
    % y->up x->front z-> left
%     path = 'E:\Python\LRG_360Panoramic-main\data\dataset\72\raw';
%     im=im2double(imread([path '\input.png']));
%     normal_data = double(imread([path '\output_normal.png'])) ./128 - 1;
%     normal_data_x = normal_data(:,:,1);normal_data_y = normal_data(:,:,2);normal_data_z = normal_data(:,:,3);
%     normal_data = cat(3, -normal_data_z, normal_data_y, normal_data_x);
% %     depth_data = imread([path '\depth.png']);
%     depth_data = readNPY([path '\depth.npy']);
%     depth_data = double(im2gray(depth_data));

    normal_data_x = normal_data(:,:,1);normal_data_y = normal_data(:,:,2);normal_data_z = normal_data(:,:,3);
    normal_data = cat(3, -normal_data_z, normal_data_y, normal_data_x);
    depth_data = double(im2gray(depth_data));
    
    [h,w,~] = size(depth_data);
    gray = im2gray(im);

%    bright = gray;bright(bright<0.6)=0;
    bright = (im - Shen2009(im));
    %bright = im2gray(bright);
    ori_bright = bright;
    bright = im2gray((bright - min(min(bright)))./(max(max(bright)) - min(min(bright))));
 
    stmp = sort(bright(:),'descend');
    threshold = stmp(round(h*w*0.02));
    tmp=bright;tmp(tmp>threshold)=1;%0.8
    mask = tmp;

%     stmp = sort(gray(:),'descend');
%     threshold = stmp(round(h*w*0.02));
%     tmp=gray;tmp(tmp>threshold)=1;%0.8
%     mask = tmp;


    [points, color_points] = get_pointcloud(im, depth_data);
    normal_data = reshape(pagetranspose(normal_data),h*w,3);
    normal_data = normalize(normal_data,2,'norm');
    normal_data(isnan(normal_data)) = 0;
    
    [x,y] = find(mask==1);
    light = double(points(x*w + y - w,:)) * 0.95;
    normal = double(normal_data(x*w + y - w,:));
    ls = size(light,1);

    tmpp = sort(points(:,2),'descend');
    

    %% Check Mask
    count_light = zeros(ls,1);
    count_image = zeros(h*w,1);
    if ls > 2000
        loopind = floor(linspace(1,ls,2000));
    else
        loopind = 1:ls;
    end
    for l=loopind
        lightpoint = light(l,:);
        view = normalize(-light(l,:),'norm');
        lightnormal = normal(l,:);
        if (norm(lightnormal) <= 1e-6)
            continue 
        end
        ref = 2 * (view * lightnormal') * lightnormal - view;
        
        tmpvec = normalize(light - lightpoint,2,'norm');
        ind = tmpvec*ref' > 0.99;
        count_light(ind) = count_light(ind) + 1;

        tmpvec = normalize(points - lightpoint,2,'norm');
        ind = tmpvec*ref' > 0.95;
        count_image(ind) = count_image(ind) + 1;

%         if lightnormal(2) < -0.98 || lightpoint(2) > tmpp(round(h*w*0.2))
%            count(l) = 9999; 
%            %count(x(l) * w + y(l) - w) = 9999;
%         end
%         
%         if lightnormal(2) > 0.98
%            count(l) = -999; 
%            %count(x(l) * w + y(l) - w) = -999;
%         end
    end
    ind = x*w + y - w;
    tmp_light = zeros(h*w,1);
    tmp_light(ind(count_light > 999)) = 1;
    count(count_light >999)=0;
    tmp_light(ind(count_light>mean(count_light))) = 1;
    % imshow(reshape(tmp,w,h)')

    tmp_image = sort(count_image,'descend');
    count_image(count_image < tmp_image(round(h*w*0.1)) ) = 0;
    count_image = count_image ./ tmp_image(round(h*w*0.0025));
    tmp_image = count_image;
    

    [M,~] = max(tmp_image,[],'all');
    tmp = reshape(tmp_image,w,h)';
    tmp(tmp<M*0.8)=0;
    tmp_image = tmp.*reshape(tmp_image,w,h)';

    tmp = (reshape(tmp_light,w,h)' + tmp_image)/2;

    mask = tmp;mask(mask>0)=1;

    mask(normal_data_y>0.5) = 0;
    mask = AreaGrowth(im, mask);
    mask(normal_data_y>0.8) = 0;
    
    %mask = Dilate(im, mask);

    mask = mask.*im;

    % for hdr
%     tmp = zeros(size(im));tmp(im>max(im,[],'all') * 0.3)=1;
%     mask = tmp.*im;
%     tmp = mean(tmp,3);

    lightsourcecolor = sum(mask,[1  2])./sum(mask>0,[1 2]);

    upind = points(:,2) > tmpp(round(h*w*0.2));
    normal_data(upind,:) = repmat([0 -1 0],[sum(upind>0),1]);

    % mask(floor(h/2):h,:)=0;
    % mask(bright==0)=0;
    lightsourcemask = mask;
    mask = mean(mask,3);
    %lightsourcemask(mask<0.5) = 0;
    [x,y] = find(mask>0);
    light = double(points(x*w + y - w,:))* 0.95;
    normal = double(normal_data(x*w + y - w,:));

    rowbright = reshape(pagetranspose(mask),h*w,1);
    intensity = double(rowbright(x*w+y-w,:));
    intensity(intensity>2) = 2;

    ls = size(light,1);
    

    shadowmap_h = h;
    shadowmap_w = w;

    shadow = zeros(h*w,1);
    count = 0;
    
    if ls > 20
        loopind = floor(linspace(1,ls,20));
    else
        loopind = 1:ls;
    end
    for l=loopind
        lightpoint = light(l,:);
        lightnormal = normal(l,:);
    
        dis = sqrt(sum((points - lightpoint).^2,2));
        theta_phi = world_2_angle_coordinate(points - lightpoint);
        i_x_y = angle_2_image_coordinate(theta_phi, shadowmap_w, shadowmap_h);
        [len,~] = size(points);
        new_image = zeros(h*w,1);
        norm_dir = normalize(lightpoint - points,2,'norm');
        norm_view = normalize(points,2,'norm'); 
        norm_dis = (dis - mean(dis))/(mean(dis) - min(dis));
        %Gen shadow map 
        envmap = zeros(shadowmap_h,shadowmap_w,3);
        tmp =zeros(shadowmap_h,shadowmap_w);
        %% for point cloud method
        shadowmap = ones(shadowmap_h,shadowmap_w) * 999999999;
        for i = 1:len
            i_x = i_x_y(i,1) + 1;
            i_y = i_x_y(i,2) + 1;
    
            if dis(i,1) < shadowmap(i_y,i_x)
                shadowmap(i_y, i_x) = dis(i,1);
                tmp(i_y,i_x) = i;
                envmap(i_y, i_x,:) = color_points(i,:);
            end
        end
        %% Compare depth
        for i = 1:len
            i_x = i_x_y(i,1) + 1;
            i_y = i_x_y(i,2) + 1;
    
            %0.1 control the shadow area
            if dis(i,1) > shadowmap(i_y,i_x) + 0.2 && shadowmap(i_y,i_x) > 0.1% && ~(normal_data(i,2) < -0.99) 
                new_image(i) = 0;
            else
                vec = norm_dir(i,:);
                N = normal_data(i,:);
                view_dir = norm_view(i,:);
                halfv = (view_dir + vec);
                %intensity(l,1)
                new_image(i) =  intensity(l,1) / ((dis(i,1) * 0.2 + 0.5).^2) * (max(0,vec*N') + max(0,N * halfv').^5) ;
            end
        end
        shadow = shadow + new_image;
        count = count + 1;
    end
    res = reshape(shadow,w,h)'./count;
    ori_res = res;
    gray = im2gray(im);
    [h,w,~] = size(im);

%     weight = res ./ mean(res,'all').*0.5;
%     res = (weight.*res + (1 - weight).*gray);

    weight = zeros([h,w]);
    top = floor(h*100/512); bottom = floor(h*410/512); 

    weight(top:bottom,:) = 1;
    weight(1:top,:) = repmat(linspace(0,1,top)',[1,w]);
    weight(bottom:end,:) = repmat(linspace(1,0,h - bottom + 1)',[1,w]);

    res = res .* weight + gray.*(1-weight);

    % 缩小图像范围
    res = res - mean(res,'all');
    res = res*0.5;
    res = res + mean(gray,'all');

    res = repmat(res,[1 1 3]).*lightsourcecolor;

    Sest = res + ori_bright;
end
% imwrite(res,['shadow_' path '_' method  '_auto.png']);
% exrwrite(res,['shadow_' path '_' method  '_auto.exr']);

function s = CheckVisibility(p)
    global mask;
    global points;
    global light;

    [h,w,~] = size(mask);
    s = 0;
    count = 0;
    ls = size(light,2);
    a = sum((points - p).^2,3);
    for l=1:10:ls
        q = light(1,l,:);    
        
        b = sum((points - q).^2,3);
        c = sqrt(sum((p - q).^2,3));

        dis = sqrt(a.*b - ((a + b - c.^2)/2).^2)./c;
        dis = real(dis);
        if sum(sum(dis < 0.005)) <= 2
            s = s + 1;
        end
        count = count + 1;
    end
   
    s = s / count;
end

function [camera_points, color_points] = get_pointcloud(color_data, depth_data)
    [h,w,~] = size(color_data);
    [pixel_x, pixel_y] = meshgrid(0:w-1,0:h-1);
    theta = pixel_x * 2.0 * pi / w - pi;
    phi = (h - pixel_y-0.5) * pi / h - pi / 2;
    
    camera_points_x = depth_data .* cos(phi) .* sin(theta);
    camera_points_y = depth_data .* sin(phi);
    camera_points_z = depth_data .* cos(phi) .* cos(theta);
    
    camera_points = cat(3,camera_points_x,camera_points_y,camera_points_z);
    camera_points = reshape(pagetranspose(camera_points),h*w,3);
    color_points = reshape(pagetranspose(color_data),h*w,3);
%     global remove_up_down
%     if remove_up_down
%         depth_data(1:72, :) = 0;
%         depth_data(441:end, :) = 0;
%     end
%     valid_depth_ind = depth_data > 0;
%     camera_points = camera_points(valid_depth_ind, :);
%     color_points = color_points(valid_depth_ind, :);
end

function [angle] = world_2_angle_coordinate(world_p)
    target_p_x = world_p(:,1);
    target_p_y = world_p(:,2); 
    target_p_z = world_p(:,3);
    r = sqrt(sum(world_p .^ 2, 2));

    phi = asin(target_p_y ./ r);
    theta = asin(target_p_x ./ sqrt(target_p_x .^ 2 + target_p_z .^ 2+1e-10));

    idx = (target_p_x < 0) & (target_p_z < 0);
    theta(idx) = - theta(idx) - pi;

    idx = (target_p_x > 0) & (target_p_z < 0);
    theta(idx) = - theta(idx) + pi;
    angle = cat(3,theta,phi);
end

function [coord] = angle_2_image_coordinate(theta_phi, width, height)
    theta = theta_phi(:, 1);
    phi = theta_phi(:, 2);

    x = (theta + pi) * width / 2 / pi;
    y = height - (phi + pi / 2) * height / pi - 0.5;
    i_x = mod(int32(round(x)),width);
    i_y = mod(int32(round(y)),height);
    coord = [i_x i_y];
end

function [inter] = img_inter_func(valid_index, values, method)
    [h,w,~] = size(values);
    inter = zeros(size(values));
    [X, Y] = meshgrid(1:h, 1:w);
    [vx,vy] = find(valid_index);
    for i = 1:3
        chn = values(:, :, i);
        Ti = griddata(vx, vy, chn(valid_index), X, Y, method);
        inter(:, :, i) = Ti';
    end
end

function [C, label, J] = kmeans(I, k) 
    [m, n, p] = size(I);%图片的大小m*n，p代表RGB三层
    X = reshape(double(I), m*n, p);
    rng('default');
    C = X(randperm(m*n, k), :);%随机选三个聚类中心
    J_prev = inf; iter = 0; J = []; tol = 1e-11;%容忍度tol，inf为无穷大

    while true,
        iter = iter + 1;
        dist = sum(X.^2, 2)*ones(1, k) + (sum(C.^2, 2)*ones(1, m*n))' - 2*X*C';%计算图片中各个点到K个聚类中心的距离
        [~,label] = min(dist, [], 2) ;  %label记录最小值的行数
        for i = 1:k,
           C(i, :) = mean(X(label == i , :)); %取新的k个聚类中心
        end
        J_cur = sum(sum((X - C(label, :)).^2, 2));%距离之和
        J = [J, J_cur];
        display(sprintf('#iteration: %03d, objective fcn: %f', iter, J_cur));
        if norm(J_cur-J_prev, 'fro') < tol,% A和A‘的积的对角线和的平方根，即sqrt(sum(diag(A'*A)))，本次与上次距离之差
            break;
        end
        if (iter==10),% A和A‘的积的对角线和的平方根，即sqrt(sum(diag(A'*A)))，本次与上次距离之差
            break;
        end
        J_prev = J_cur;
    end
end

function im = Dilate(image, mask)
[M,I] = max(image.*mask,[],'all');
[h,w] = size(image);
% mask = zeros(h,w);
% mask(mod(I,h)+1,floor(I/h)+1)=1;
mask(mask>0)=1;
flag = true;

% se = strel("disk",5);
% mask = imerode(mask,se);

se = strel("disk",1);
count = 20;
while flag
    mask = imdilate(mask,se);
    tmpr = image(:,:,1).*mask;
    tmpr(mask<1)=[];
    tmpg = image(:,:,2).*mask;
    tmpg(mask<1)=[];
    tmpb = image(:,:,3).*mask;
    tmpb(mask<1)=[];
    m = (tmpr+tmpg+tmpb) / 3;
    if max(mink(m,ceil(size(m,2)*0.25))) < M*0.8 || count==0
        flag =false;
    end
    count = count - 1;
end
if count==0
    disp('Early stop dilate.')
end

im = mask;
% tmp = image;
% tmp(tmp<M*2/3)=0;
% im = tmp.*image;
end

function im = AreaGrowth(image, mask)
[M,~] = max(image.*mask,[],'all');
se = strel("disk",1);

I = im2gray(image); % 输入图像
Ir = image(:,:,1);
Ig = image(:,:,2);
Ib = image(:,:,3);
[Gr,~] = imgradient(Ir);
[Gg,~] = imgradient(Ig);
[Gb,~] = imgradient(Ib);
G = max(cat(3,Gr,Gg,Gb),[],3);

[Mr,~] = max(Ir.*mask,[],'all');
[Mg,~] = max(Ig.*mask,[],'all');
[Mb,~] = max(Ib.*mask,[],'all');

Mask = imerode(mask,se); % 原始mask
S = find(Mask); % 种子点集
% S = S(1:20:size(S));
D = S; % 扩散点集

res = false(size(I)); 
res(S) = true;
step = 0;
while ~isempty(S) && step < 50
    step = step + 1;
    while ~isempty(S)
      p = S(1);
      S(1) = [];
      [row,col] = ind2sub(size(I),p);
      rowMin = max(row-1,1);
      rowMax = min(row+1,size(I,1));
      
      colMin = max(col-1,1); 
      colMax = min(col+1,size(I,2));
      
      [neighsx,neighsy] = meshgrid(rowMin:rowMax, colMin:colMax);
      neighsx = neighsx(:);
      neighsy = neighsy(:);           
      for n=1:size(neighsx,1)
        q = sub2ind(size(I),neighsx(n),neighsy(n));
        if res(q)==false && G(q) < 0.05 && Ir(q) > Mr * 0.8 && Ig(q) > Mg * 0.8 && Ib(q) > Mb * 0.8
          D = [D; q];
          res(q) = true;
        end
      end
    end
    S = D;
    D = [];
end
im = res;
end
