function [vel_left, vel_right] = Kinematic_Conv(v,w,r,l)
l = l/2;
vel_left = (v - w*l)/(r);
vel_right = (v + w*l)/(r);
