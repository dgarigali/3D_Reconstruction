%Function that returns the 3D Point Cloud based on sequence of depth and
%RGB images and the corresponding Rigid Body transformations
function [pc] = plot3D(rgb_list, depth_list, cam_params, transforms)
    
    %Read number of images
    num_images = numel(rgb_list);
    
    %Prepare local variables
    xyz_full = [];
    rgb_full = [];
    
    %Loop for iterating over each image
    for i = 1:num_images
    
        %Load rgb and depth image
        im = imread(rgb_list{i});
        depth_data = load(depth_list{i});
        d_img = double(depth_data.depth_array)./1000; %to meters
        
        %Image dimensions
        [v, u] = size(d_img);
        cols = (1:v)'*ones(1, u);
        rows = ones(v, 1)*(1:u);
        
        %Camera extrinsic parameteres
        M = [cam_params.R cam_params.T];
        
        %Get xyz from depth image
        d = d_img(:)';
        d_w = [d.*rows(:)'; d.*cols(:)'; d];
        d_pos = cam_params.Kdepth\d_w;
        
        %Get xyz in the first image reference frame
        H_model = [transforms{i}.R transforms{i}.T; [0 0 0 1]];
        xyz_i1 = H_model*[d_pos; ones(1, size(d_pos, 2))];
        xyz_ref = xyz_i1(1:3, :);
        
        %Get rgb from xyz
        rgb_pos = M*[d_pos; ones(1, size(d_pos, 2))];
        
        %Get position of rgb values
        om_img = cam_params.Krgb*rgb_pos;
        img_pixels = [om_img(1, :)./om_img(3, :); om_img(2, :)./om_img(3, :)];
        img_pixels(isnan(img_pixels)) = 1;
        img_pixels = round(img_pixels);
        
        %Limit range of values
        row1 = img_pixels(1, :); row1(row1 < 1) = 1; row1(row1 > u) = u;
        row2 = img_pixels(2, :); row2(row2 < 1) = 1; row2(row2 > v) = v;
        img_pixels = [row1 ; row2];

        %Get rgb values in the order of the depth pixels
        rgbi_r = im(sub2ind(size(im),img_pixels(2, :), img_pixels(1, :), ones(1, size(img_pixels, 2)))); 
        rgbi_g = im(sub2ind(size(im),img_pixels(2, :), img_pixels(1, :), 2*ones(1, size(img_pixels, 2)))); 
        rgbi_b = im(sub2ind(size(im),img_pixels(2, :), img_pixels(1, :), 3*ones(1, size(img_pixels, 2)))); 
        rgb = [rgbi_r; rgbi_g; rgbi_b];
        
        %Append to full list of xyz and rgb
        xyz_full = [xyz_full xyz_ref];
        rgb_full = [rgb_full rgb];
    end

    %Point cloud of all points in 3D
    pc = pointCloud(xyz_full','color',rgb_full');
end