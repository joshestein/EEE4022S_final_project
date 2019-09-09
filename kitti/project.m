function p_out = project(p_in,T)
  % expecting p_in to be x, y, z of velodyne
  % due to the already 'transformed' nature of the velodyne points (from frame integration), expect p_in to be a n-by-4 matrix
  % T is the transform matrix. Expecting a 3-by-4 matrix
  % % % % % %
  % output is an n-by-2 matrix, transformed to camera frame

% dimension of data and projection matrix
dim_norm = size(T,1);
dim_proj = size(T,2);

% do transformation in homogeneous coordinates
% this just adds a column of 1s for the matrix multiplication to work
p2_in = p_in;
if size(p2_in,2)<dim_proj
  p2_in(:,dim_proj) = 1;
end
p2_out = (T*p2_in')';

% normalize homogeneous coordinates:
p_out = p2_out(:,1:dim_norm-1)./(p2_out(:,dim_norm)*ones(1,dim_norm-1));

% keep depth information!
p_out(:, 3) = p2_out(:,3);