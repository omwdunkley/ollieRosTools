function [ sphere ] = unrotate_unit_sphere( fs, R )

R = R(1:3,1:3);
sphere = R*fs(1:3,:);

end

