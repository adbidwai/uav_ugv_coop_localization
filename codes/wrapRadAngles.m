%wrapRadAngles.m
%%Enforces [-pi,pi] angular constraints on an array of angles in radians
function anglesOut = wrapRadAngles(anglesIn)
anglesOut = anglesIn;
llogind = anglesIn < -pi; 
ulogind = anglesIn > pi; 
anglesOut(llogind) = anglesOut(llogind)+2*pi;
anglesOut(ulogind) = anglesOut(ulogind)-2*pi;