function [As] = NonLocalConstraint_KNN(I,normal,point, nn_num_s ,shadow)
    run('utils\vlfeat-0.9.21/toolbox/vl_setup.m');
    [h,w,~] = size(I);
    I = reshape(I,h*w,3);
    normal = reshape(normal,h*w,3);
    points_whitened=reshape(whiten(reshape(point,[],3)),size(I));%whiten the 3D points
    points_whitened = points_whitened./max(max(points_whitened));
    n = h*w;
   
    data=[reshape(points_whitened,[],3) normal]';
    [ind,dis]=(vl_kdtreequery(vl_kdtreebuild(data),data,data,'numneighbors',nn_num_s));
    ind = double(ind);
    dis = dis(:);
    tmp=[reshape(repmat(1:n,[nn_num_s 1]),[],1) reshape(ind,[],1)];
    tmp = unique([min(tmp(:,1),tmp(:,2)) max(tmp(:,1),tmp(:,2))],'rows');
    diff = shadow;
    point_tmp = diff(:);
    edge1 = point_tmp(tmp(:,1));
    edge2 = point_tmp(tmp(:,2));
    diff = (edge1 - edge2);
    diff = sum(diff.^2, 2);

    h_pos = h / 2 - mod(tmp,h);
    h_height = abs(sin(h_pos(:,1)/h * pi)) + abs(sin(h_pos(:,2)/h*pi));

    weightAs = ones(size(diff)).*(abs(diff) < 0.05) .* exp(0.5 *h_height);
    edgeS = tmp;

    weightAs = weightAs(:);
    As = sparse(edgeS(:,1), edgeS(:,1), weightAs, h*w, h*w)...
       - sparse(edgeS(:,1), edgeS(:,2), weightAs, h*w, h*w)...
       + sparse(edgeS(:,2), edgeS(:,2), weightAs, h*w, h*w)...
       - sparse(edgeS(:,2), edgeS(:,1), weightAs, h*w, h*w);
end
