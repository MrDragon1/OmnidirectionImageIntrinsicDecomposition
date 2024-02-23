function I_d = Yoon2006(I)
%Yoon2006 I_d = Yoon2006(I)
%  This method operates by iteratively reducing or increasing a
%  neighboring pixel intensity.
%  
%  It is ambiguous how they define the neighborhood in which they operate, so I
%  have assumed they mean the next pixel when the image is represented in a
%  lexicographic order.
%  
%  This method is specially slow in Octave because of the repeated iterated
%  accesses. Should take 1 s, but takes 10+ on Octave/Linux and 30+ on
%  Octave/Windows. Ideally it should be made into a vectorized
%  implementation.
%  
%  If you spot any obvious mistakes, please inform me of them, as I have
%  made some special considerations in the implementation in order to avoid
%  propagating unreliable pixels.
%  
%  See also SIHR, Tan2005.

assert(isa(I, 'float'), 'SIHR:I:notTypeSingleNorDouble', ...
    'Input I is not type single nor double.')
assert(min(I(:)) >= 0 && max(I(:)) <= 1, 'SIHR:I:notWithinRange', ...
    'Input I is not within [0, 1] range.')
[n_row, n_col, n_ch] = size(I);
assert(n_row > 1 && n_col > 1, 'SIHR:I:singletonDimension', ...
    'Input I has a singleton dimension.')
assert(n_ch == 3, 'SIHR:I:notRGB', ...
    'Input I is not a RGB image.')

I = reshape(I, [n_row * n_col, n_ch]);
I_min = min(I, [], 2);
I_sf = I - I_min;

sum_I_sf_ = sum(I_sf, 2);

cr = I_sf(:, 1) ./ sum_I_sf_;
cg = I_sf(:, 2) ./ sum_I_sf_;

cr(isnan(cr)) = 0;
cg(isnan(cg)) = 0;

skip = false([n_row * n_col - 1, 1]);

% chroma threshold values (color discontinuity)
th_r = 0.05;
th_g = 0.05;

% iterate until only diffuse pixels are left
count = uint32(0);
iter = uint16(0);

idx = (1:(n_row * n_col - 1))';
skip(sum_I_sf_(idx) == 0 ... % check if sum along rows ~= 0
    | sum_I_sf_(idx+1) == 0) = true;
skip(abs(cr(idx)-cr(idx+1)) > th_r ... % check discontinuities
    | abs(cg(idx)-cg(idx+1)) > th_g) = true;
skip((1:n_row-1)*n_col) = true;
skip(I_min(idx) < 12/255) = true;

rd = ones([n_row * n_col - 1, 1]);
rd(idx(~skip)) = sum(I_sf(idx(~skip), :), 2) ...
    ./ sum(I_sf(idx(~skip)+1, :), 2);

rds = ones([n_row * n_col - 1, 1]);

while true
    rds(idx(~skip)) = sum(I((idx(~skip)), :), 2) ...
        ./ sum(I((idx(~skip))+1, :), 2);
    for x1 = 1:n_row * n_col - 1
        x2 = x1 + 1;
        if skip(x1)
            continue
        elseif sum(I(x1,:), 2) == 0 || ...
                sum(I(x2,:), 2) == 0 % || ...
            %        (abs(cr_col(x1)-cr_col(x2)) > th_r && ...
            %        abs(cg_col(x1)-cg_col(x2)) > th_g)
            skip(x1) = true;
            continue
        end
        % compare ratios and decrease intensity
        if rds(x1) > rd(x1) % && rds ~= 1
            m = sum(I(x1, :), 2) - rd(x1) * sum(I(x2, :), 2);
            if m < 1e-3
                continue
            end
            I(x1, :) = I(x1, :)-m/3;
            %skip(x1) = true;
            count = count + 1;
        elseif rds(x1) < rd(x1) % && rd(x1) ~= 1
            m = sum(I(x2, :), 2) - sum(I(x1, :), 2) / rd(x1);
            if m < 1e-3
                continue
            end
            I(x2, :) = I(x2, :)-m/3;
            %skip(x2) = true;
            count = count + 1;
        end
    end
    if count == 0 || iter == 1000
        break
    end
    count = 0;
    iter = iter + 1;
end

I_d = reshape(I, [n_row, n_col, n_ch]);

end
