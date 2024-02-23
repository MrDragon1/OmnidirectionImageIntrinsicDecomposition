function [src,sfi] = zSpecularFreeImage(src)
Lambda = 0.6;
camDark = 20;
r = src.rgb(:,:,1); g = src.rgb(:,:,2); b = src.rgb(:,:,3);
src.i(intersect(intersect(find(r<camDark),find(g<camDark)),...
    find(b<camDark))) = z.CAMERA_DARK;
c = z.MaxChroma(src.rgb);
dI = (z.Max(src.rgb).*(3*c-1))./(c*(3*Lambda-1));
sI = (z.Total(src.rgb)-dI)/3;
drgb = min(255,max(0,src.rgb-sI));
sfi.rgb = drgb;
end

