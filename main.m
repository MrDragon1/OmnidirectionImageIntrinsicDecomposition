clc;clear;close all;
addpath('./utils/SIHR');
addpath('./utils/matlab');
addpath('./utils/npy-matlab/npy-matlab');

SIHR();
param = struct('HalfSest',          false, ...
            'ISest',                false, ...
            'NonLocal',             true, ...
            'Intensity',            true, ...
            'Data',                 true, ...
            'Refine',               true);
path = './img';
savepath = [path '/results'];

if ~exist(savepath)
    mkdir(savepath)
end

[R,S,shadowmask,lightsourcemask,lightcolor] = Our(path,param);
imwrite(R,[savepath '\Ours_R.png']);
imwrite(im2gray(S),[savepath '\Ours_S.png']);
imwrite(shadowmask./max(shadowmask,[],'all'),[savepath '\Sest.png']);
imwrite(lightsourcemask,[savepath '\LightSource.png']);
% imwrite(ones(32,32,3).*lightcolor,[savepath '\lightcolor.png']);