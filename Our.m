function [Ref,Shading,shadowmask,lightsourcemask,lightcolor] = Our(path, param)
    im=im2double(imread([path '\input.png']));
    im = imresize(im,[512,1024]);

    depth_data = readNPY([path '\depth.npy']);
    depth_data = double(im2gray(depth_data));
    depth_data = imresize(depth_data,[512,1024]);

    segment = imread([path '\segment.png']);
    segment = imresize(segment,[512,1024]);

    [points] = get_pointcloud(depth_data);
    [nx,ny,nz] = surfnorm(points(:,:,1),points(:,:,2),points(:,:,3));
    normal_data = cat(3,nz,ny,-nx);

    normal_data = imresize(normal_data,[512,1024]);
    depth_data = imresize(depth_data,[512,1024]);

    im(im>1) = 1;im(im<0) = 0;

    gray = im2gray(im);
    [h,w,~] = size(im);

    [shadowmask,lightsourcemask,lightcolor]= getShadowMask(im,depth_data,normal_data,segment);

    ceiling_color = [120, 120, 80];
    ceiling_mask = (segment(:,:,1) == ceiling_color(1)) & ...
           (segment(:,:,2) == ceiling_color(2)) & ...
           (segment(:,:,3) == ceiling_color(3));
    ceiling = zeros(size(segment, 1), size(segment, 2));
    ceiling(ceiling_mask) = 1;

    floor_color = [80, 50, 50];
    floor_mask = (segment(:,:,1) == floor_color(1)) & ...
           (segment(:,:,2) == floor_color(2)) & ...
           (segment(:,:,3) == floor_color(3));

    rug_color = [255, 9, 92];
    rug_mask = (segment(:,:,1) == rug_color(1)) & ...
           (segment(:,:,2) == rug_color(2)) & ...
           (segment(:,:,3) == rug_color(3));

    floor1 = zeros(size(segment, 1), size(segment, 2));
    floor1(floor_mask | rug_mask) = 1;
    other = 1 - (ceiling + floor1);

    Mceiling = exp((sum(gray.*ceiling,'all') / sum(ceiling,'all')) - 1).^2;
    Mfloor = exp(sum(gray.*floor1,'all') / sum(floor1,'all')).^2;
    Mother = (sum(gray.*other,'all') / sum(other,'all'));
    weight = imgaussfilt(ceiling * Mceiling + Mfloor * floor1 + other, 20, 'FilterSize', 33);
    weight = imresize(weight,size(shadowmask,[1 2]));
    weight = repmat(weight,[1 1 3]);

    if param.Refine
        shadowmask = shadowmask.* weight;
        shadowmask = imguidedfilter(shadowmask,gray,'NeighborhoodSize', 5);
    end

    if(size(shadowmask,3) == 1)
        shadowmask = cat(3,shadowmask,shadowmask,shadowmask);
    end
    [I,R]=intrinsicScene_color_L1(im,depth_data,normal_data,shadowmask,param);

    res = I.*R;
    res = res./max(max(max(res)));
    Ref = R;
    Shading = I;
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

