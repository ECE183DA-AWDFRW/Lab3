[sf,sr] = estimate(1.9, 1,50, 2, 2);

function [sf, sr] = estimate(x,y,theta, dimx, dimy)
% check if angle is n*90 (n=1,2,3...)
if theta == 0
    sf = dimx - x;
    sr = y;
    return
elseif theta == 90
    sf = dimy - y;
    sr = dimx - x;
    return
elseif theta == 180
    sf = x;
    sr = dimy - y;
    return
elseif theta == 270
    sf = y;
    sr = dimx - x;
    return
end
%determine the angles from car to each corner
Q1 = [x,y; dimx,dimy];
dh = pdist(Q1, 'euclidean');
dx = dimx - x;
if dx == 0
    thetaQ1 = 90;
else
    thetaQ1 = acosd(dx/dh);
end

Q4 = [x,y; dimx,0];
dh = pdist(Q4, 'euclidean');
if dx == 0
    thetaQ4 = 270;
else
    thetaQ4 = 360 - acosd(dx/dh);
end

Q2 = [x,y; 0,dimy];
dh = pdist(Q2, 'euclidean');
if x == 0
    thetaQ2 = 90;
else
    thetaQ2 = 180 - acosd(x/dh);
end

Q3 = [x,y; 0,0];
dh = pdist(Q3, 'euclidean');
if x == 0
    thetaQ3 = 270;
else
    thetaQ3 = 180 + acosd(x/dh);
end
%determine sensor measurements based on which quadrant of box it is facing,
%also check if both sensors are pointing towards same wall or not
if theta <= thetaQ1 || theta >= thetaQ4
    if thetaQ1 + 360 - thetaQ4 > 90
        if theta > thetaQ1 - (thetaQ1 + 360 - thetaQ4 - 90) && theta < thetaQ1
            sf = (dimx - x) * cosd(theta);
            sr = (dimx - x) * cosd(90 - theta);
            return
        end
    end
    sf = (dimx - x) * cosd(theta);
    sr = y * cosd(theta);
elseif theta > thetaQ1 && theta < thetaQ2
    if thetaQ2 - thetaQ1 > 90
        if theta > thetaQ2 - (thetaQ2 - thetaQ1 - 90) && theta < thetaQ2
            sf = (dimy - y) * cosd(theta);
            sr = (dimy - y) * cosd(90 - theta);
            return
        end
    end
    sf = (dimy - y) * cosd(theta);
    sr = (dimx - x) * cosd(theta);
elseif theta >= thetaQ2 && theta <= thetaQ3
    if thetaQ3 - thetaQ2 > 90
        if theta > thetaQ3 - (thetaQ3 - thetaQ2 - 90) && theta < thetaQ3
            sf = x * cosd(theta);
            sr = x * cosd(90 - theta);
            return
        end
    end
    sf = x * cosd(theta);
    sr = (dimy - y) * cosd(theta);
elseif theta > thetaQ3 && theta < thetaQ4
    if thetaQ4 - thetaQ3 > 90
        if theta > thetaQ4 - (thetaQ4 - thetaQ3 - 90) && theta < thetaQ4
            sf = y * cosd(theta);
            sr = y * cosd(90 - theta);
            return
        end
    end
    sf = y * cosd(theta);
    sr = (dimx - x) * cosd(theta);
end
end