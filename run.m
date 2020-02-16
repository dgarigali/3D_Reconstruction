%% Script to run two functions:
%rigid_body3D.m -> returns the 3D Rigid Body transformation between each image and the reference image (i.e., first image)
%plot3D.m -> builds the 3D point cloud of the dataset scene based on the transformations obtained 

%Clear screen and command window
clear;
close all;
clc;

%Get depth and rgb path list
folder_name = 'datasets/table/';
d_depth =dir(strcat(folder_name, '*.mat'));
d_rgb_jpg=dir(strcat(folder_name, '*.jpg'));
d_rgb_png=dir(strcat(folder_name, '*.png'));
if(length(d_rgb_jpg) < length(d_rgb_png))
    d_rgb = d_rgb_png;
else
    d_rgb = d_rgb_jpg;
end
rgb_list=cell(length(d_rgb),1);
depth_list = cell(length(d_depth), 1);
for i=1:length(d_rgb)
    rgb_list{i}=strcat(folder_name, d_rgb(i).name);
    depth_list{i}=strcat(folder_name, d_depth(i).name);
end

%Load camera parameters
load('cam_params.mat');

%Obtain 3D Rigid Body transformations
tic
[transforms] = rigid_body3D(depth_list, rgb_list, cam_params, 0);
toc

%Obtain 3D Point Cloud
[pc] = plot3D(rgb_list, depth_list, cam_params, transforms);
showPointCloud(pc);