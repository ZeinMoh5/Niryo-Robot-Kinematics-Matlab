function [Alpha, Beta, Gamma] = RPY(Orientation)

r31 = max(min(Orientation(3, 1), 1), -1);
Beta = asin(-r31);

if cos(Beta) == 0
    disp('RPY CANNOT BE IMPLEMMTED');
    Gamma = 0;
    Alpha = atan2(-Orientation(1, 2), Orientation(2, 2));
else
    Alpha = atan2(Orientation(2, 1), Orientation(1, 1));
    Gamma = atan2(Orientation(3, 2), Orientation(3, 3));
end

end
