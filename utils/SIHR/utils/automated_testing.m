is_octave = exist('OCTAVE_VERSION', 'builtin'); % https://wiki.octave.org/Compatibility

method = { ...
    @Tan2005, @Yoon2006, @Shen2008, @Shen2009, ...
    @Yang2010, @Shen2013, @Akashi2016};

name = {'animals', 'cups', 'fruit', 'masks'};
gt = '_gt';
ext = '.bmp';

image = cell(4, 1);
truth = cell(4, 1);

for i = 1:length(name)
    image{i} = im2double(imread([name{i}, ext]));
    truth{i} = im2double(imread([name{i}, gt, ext]));
end

my_toc = zeros(length(method), length(name));
my_psnr = zeros(length(method), length(name));
if ~is_octave
    my_ssim = zeros(length(method), length(name));
end

for m = 1:length(method)
    disp(['Method ', func2str(method{m})])
    for i = 1:length(name)
        disp(['  Image ', name{i}])
        tic
        I_d = feval(method{m}, image{i});
        my_toc(m, i) = toc;
        disp(['    Elapsed time ', num2str(my_toc(m, i)), ' s'])
        my_psnr(m, i) = psnr(I_d, truth{i});
        if ~is_octave
            my_ssim(m, i) = ssim(I_d, truth{i});
        end
    end
end

disp('my_psnr =')
disp(round(10*my_psnr)/10)

if ~is_octave
    disp('my_ssim =')
    disp(round(1000*my_ssim)/1000)
end
