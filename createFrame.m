function [gst_out] = createFrame(coords)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
coords = coords * .001;
gst_out = [1,0,0,coords(1);0,1,0,coords(2);0,0,1,0;0,0,0,1];
end