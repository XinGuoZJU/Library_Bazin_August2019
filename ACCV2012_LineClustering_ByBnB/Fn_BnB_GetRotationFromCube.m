function  Rbar=Fn_BnB_GetRotationFromCube(RotComponents)

% goal: given a cube, compute the center rotation, Rbar, 1x3

if ~(length(RotComponents)==3 && size(RotComponents,1)==1), RotComponents, error('wrong size'), end
% associated function: Fn_BnB_GetCubeFromRotation

MyNorm=norm(RotComponents);
        RotVector=RotComponents/MyNorm; % unit vector of the rotation
        RotAngle=MyNorm; % amplitude of the rotation
        Rbar=Fn_Rotation_Matrix_matrix_from_VectorAndAngle(RotVector, RotAngle, 0,1);
        