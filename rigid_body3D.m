%Function that returns the 3D Rigid Body transformation between each image and the reference image (i.e., first image)
function [transforms] = rigid_body3D(imglistdepth, imglistrgb, cam_params, ICP)

    %Load RGB and depth images
    function [rgb_im, d_im] = load_images(img_num)
        rgb_im = imread(imglistrgb{img_num});
        depth_data = load(imglistdepth{img_num});
        d_im = double(depth_data.depth_array)./1000; %to meters
    end

    %Project depth in RGB image plane
    function [d_pos, img_pixels] = from_depth_to_RGB(d_img)
        
        %Generate xyz from depth
        d = d_img(:)';
        d_pos = (cam_params.Kdepth)\[d.*rows(:)'; d.*cols(:)'; d];

        %Rigid transformation - RGB camera
        M = [cam_params.R cam_params.T];
        rgb_pos = M*[d_pos; ones(1, size(d_pos, 2))];

        %Project in the RGB image plane
        om_img = cam_params.Krgb*rgb_pos;
        img_pixels = [om_img(1, :)./om_img(3, :); om_img(2, :)./om_img(3, :)]; 
    end

    %Match surf 2D points to respective 3D points
    function [xyz] = match_2D_to_3D(img_pixels, p, d_pos)
        kdts = KDTreeSearcher(img_pixels','BucketSize',size(img_pixels,2));
        xyz = d_pos(:, knnsearch(kdts, p));
    end

    %Perform procrustes
    function [R, T] = procrustes(pts1, pts2)
                
        %Get average positions for each coordinate
        avg_1 = mean(pts1, 2);
        B = pts1 - avg_1;  % normalize inliers 1
        avg_2 = mean(pts2, 2);
        A = pts2 - avg_2; % normalize inliers 2

        %Perform SVD
        C = A*B';
        [U, ~, V] = svd(C);

        %Obtain rotation matrix and translation vetor
        R = U*[1 0 0; 0 1 0; 0 0 det(U*V')]*V'; %R = U*V';
        T = avg_2 - R*avg_1;
    end
        
    %Load first RGB and depth images
    num_images = numel(imglistrgb);
    [im1, d_img1] = load_images(1);

    %Prepare output variables
    transforms = cell(1, num_images);

    %Auxiliar transform matrix
    aux_H = cell(num_images);
    aux_H{1,1} = eye(4);

    %Setup RANSAC (determine number of iterations) 
    s = RandStream('mt19937ar', 'Seed', 0); % Seed random number generator
    threshold = 0.01; %set threshold (|r_i|<epsilon) - inlier criteria
    bigP = 0.99; %probability of success after k tries
    smallP = 0.45; %fraction of matches that are inliers - pessimistic assumption
    K = ceil(log(1-bigP)/log(1-smallP^(4))); %number of iterations: formula from slides
    
    %Setup ICP parameters
    inliers_percentage = 0.01;
    icp_num_iter = 5;
            
    %Generate accesses
    [v, u] = size(d_img1);
    cols = (1:v)'*ones(1, u);
    rows = ones(v, 1)*(1:u);
    
    %Project depth in RGB image 1 plane
    [d_pos1, img1_pixels] = from_depth_to_RGB(d_img1);
    
    %Get features of first image
    SURF_threshold = 100;
    [f1, vp1] = extractFeatures(rgb2gray(im1), detectSURFFeatures(rgb2gray(im1), 'MetricThreshold', SURF_threshold));

    %---------------------%
    %       Transforms    %
    %---------------------%

    %loop for each sequential pair of images
    for i = 2:num_images

        %load next RGB and depth images
        [im2, d_img2] = load_images(i);
        
        %--------%
        %  SURF  %
        %--------%
        [f2, vp2] = extractFeatures(rgb2gray(im2), detectSURFFeatures(rgb2gray(im2), 'MetricThreshold', SURF_threshold));
        [idxPairs, ~] = matchFeatures(f1, f2);
        p1 = double(vp1(idxPairs(:,1),:).Location); % [u1 v1]
        p2 = double(vp2(idxPairs(:,2),:).Location); % [u2 v2]

        %-----------------------%
        %  XYZ matching points  %
        %-----------------------%
        
        % Project depth in RGB image 2 plane
        [d_pos2, img2_pixels] = from_depth_to_RGB(d_img2);
        
        % Find correspondent 3D points using NN for each image
        [xyz_1] = match_2D_to_3D(img1_pixels, p1, d_pos1);
        [xyz_2] = match_2D_to_3D(img2_pixels, p2, d_pos2);
        
        %------------------%
        %  RANSAC (Affine) %
        %------------------%

        % Prepare variables
        num_SURF_matches = length(xyz_1);
        max_inliers = 0;
        SURF_pts = [xyz_1; xyz_2];                   
        
        %Loop for RANSAC
        for k = 1:K

            %Randomly pick 4 points
            comb = randperm(s, num_SURF_matches, 4); 
            pickedPts = SURF_pts(:, comb(:))';
            p_im1 = pickedPts(:, 1:3);
            p_im2 = pickedPts(:, 4:6);

            %Prepare matrixes for Affine transformation
            homog = [p_im1 ones(4, 1)];
            A = [homog zeros(4) zeros(4); zeros(4) homog zeros(4); zeros(4) zeros(4) homog];
            
            %Check if picked points are all non-collinear
            if rank(A) < 12
               k = k - 1;
               continue; 
            end

            %Apply Affine transformation
            H_model = p_im2'/(homog');

            %Pass all points through the H_model transform
            pts1 = [xyz_1; ones(1, size(xyz_1, 2))];
            pts2_ = H_model*pts1;

            %Get difference from image2 actual points
            diff = pts2_ - xyz_2;   
            mod_diff = diag(diff'*diff); %diagonal values are norm^2

            %Count inliers and update best model accordingly (model with more inliers)
            num_inliers = sum(mod_diff < threshold);
            if num_inliers > max_inliers
                max_inliers = num_inliers;

                %Save inliers, exclude pickedPts as well
                ransac_inliers = SURF_pts.*((mod_diff < threshold)'); %get only coordinates from inliers
                ransac_inliers(:, all(~ransac_inliers, 1)) = []; %remove empty cols
            end
        end

        %---------------------------%
        %  Procrustes (Rigid Body)  %
        %---------------------------%
        inl_1 = ransac_inliers(1:3, :);
        inl_2 = ransac_inliers(4:6, :);
        [R, T] = procrustes(inl_1, inl_2);
        H_total = inv([R T;0 0 0 1]);
        
        if(ICP == 1)

            %--------------------------------%
            %       Calculate mean error     %
            %--------------------------------% 
            I1 = [inl_1; ones(1, size(inl_1, 2))];
            inl_2_ = [R T; 0 0 0 1]*I1;
            inl_2_ = inl_2_(1:3, :);
            e = (inl_2_ - inl_2).*(inl_2_ - inl_2);
            e = sum(sqrt(sum(e)))/max_inliers;

            %---------------------------%
            %            ICP            %
            %---------------------------%

            %Randomly pick some xyz points from second image
            d_pos2_red = d_pos2(:, randperm(s, size(d_pos2, 2), round(inliers_percentage*size(d_pos2, 2))));

            %Iteration of ICP
            for j = 1 : icp_num_iter

                %Transform points from reduced image 2 to image 1 reference frame 
                xyz_pos2_in_1 = H_total*[d_pos2_red; ones(1, size(d_pos2_red, 2))];
                xyz_pos2_in_1(4, :) = [];

                %Find closest points of reduced image 2 in image 1 reference frame
                xyz_pos1 = match_2D_to_3D(d_pos1, xyz_pos2_in_1', d_pos1);

                %Computer error of each point
                e_icp = (xyz_pos1 - xyz_pos2_in_1).*(xyz_pos1 - xyz_pos2_in_1);
                e_icp = sqrt(sum(e_icp));

                %Remove nearest neighbours that are at a distance bigger than average error for Ransac inliers
                xyz_pos1(:, e_icp > e ) = [];
                xyz_pos2_in_1(:, e_icp > e ) = [];

                %Calculate new transformation
                [R_icp, T_icp] = procrustes(xyz_pos1, xyz_pos2_in_1);
                H_total = [R_icp T_icp; 0 0 0 1]\H_total; 
            end
        end

        %---------------%
        %  H transform  %
        %---------------%
        aux_H{i,i} = eye(4);
        for k = 1:i-1
            aux_H{k,i} = H_total\aux_H{k,i-1};
            aux_H{i,k} = inv(aux_H{k,i});
        end 
        
        %Update variables
        img1_pixels = img2_pixels;
        d_pos1 = d_pos2;
        f1 = f2;
        vp1 = vp2;        
    end

    %Obtain final transforms cell vector
    for i = 1:num_images
        transforms{i}.R = aux_H{i, 1}(1:3, 1:3);
        transforms{i}.T = aux_H{i, 1}(1:3, 4);
    end  
end