function [r,g,b] = getRGB(val,val_max,color_scale)
% this function gives RGB color mapping for visualization 
% val = data value 
% val_max = expected maximum of data value 
% color_scale = maximum of color value (0-255? I don't know ^^)

val_norm = val/val_max;
r = val_norm * color_scale;
b = color_scale -val_norm * color_scale;
if val_norm <= 0.5
    g = val_norm * color_scale * 2;
else
    g = 2 * color_scale  -val_norm * color_scale * 2;
end




end