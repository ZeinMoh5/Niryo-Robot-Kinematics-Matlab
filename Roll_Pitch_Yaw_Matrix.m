function [ Orientation ] = Roll_Pitch_Yaw_Matrix( Alpha,Beta,Gamma )

Orientation(1,1)=cos(Alpha)*cos(Beta);
Orientation(1,2)=-sin(Alpha)*cos(Gamma)+cos(Alpha)*sin(Beta)*sin(Gamma);
Orientation(1,3)=sin(Alpha)*sin(Gamma)+cos(Alpha)*sin(Beta)*cos(Gamma);

Orientation(2,1)=sin(Alpha)*cos(Beta);
Orientation(2,2)=cos(Alpha)*cos(Gamma)+sin(Alpha)*sin(Beta)*sin(Gamma);
Orientation(2,3)=-cos(Alpha)*sin(Gamma)+sin(Alpha)*sin(Beta)*cos(Gamma);

Orientation(3,1)=-sin(Beta);
Orientation(3,2)=cos(Beta)*sin(Gamma);
Orientation(3,3)=cos(Beta)*cos(Gamma);

end

