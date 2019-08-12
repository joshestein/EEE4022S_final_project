function status = outside_image(img, velo_img, pos)
% determines wether a given position is inside image bounds
% 
% Parameters:
% img        | image you want to be inside
% velo_image | x,y positions of projected LiDAR pointcloud
% pos        | current position index
status = (velo_img(pos,1) > size(img,2) || round(velo_img(pos, 1)) <= 0 || velo_img(pos,2) > size(img,1) || round(velo_img(pos, 2)) <= 0);