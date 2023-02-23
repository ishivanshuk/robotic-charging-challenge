%a=imread("images_test\6.jpg");
port=imread("images_test\sample_port.jpg");
port_grey = rgb2gray(port);
ref_pts = detectSURFFeatures(port_grey);
[ref_features,  ref_validPts] = extractFeatures(port_grey,  ref_pts);
figure; imshow(port);
hold on; plot(ref_pts.selectStrongest(50));
%%
figure;
subplot(5,5,3); title('First 25 Features');
for i=1:25
    scale = ref_pts(i).Scale;
    image = imcrop(port,[ref_pts(i).Location-10*scale 20*scale 20*scale]);
    subplot(5,5,i);
    imshow(image);
    hold on;
    rectangle('Position',[5*scale 5*scale 10*scale 10*scale],'Curvature',1,'EdgeColor','g');
end
%%
image = imread('images_test\test_img.jpg');
I = rgb2gray(image);

I_pts = detectSURFFeatures(I);
[I_features, I_validPts] = extractFeatures(I, I_pts);

figure;
imshow(image);
hold on; plot(I_pts.selectStrongest(50));

%%
index_pairs = matchFeatures(ref_features, I_features);

ref_matched_pts = ref_validPts(index_pairs(:,1)).Location;
I_matched_pts = I_validPts(index_pairs(:,2)).Location;

figure, showMatchedFeatures(image, port, I_matched_pts, ref_matched_pts, 'montage');
title('Showing all matches');

%%

[tform_matrix, inlierIdx] = estimateGeometricTransform2D(ref_matched_pts, I_matched_pts, "similarity");

ref_inlier_pts = ref_matched_pts(inlierIdx,:);
I_inlier_pts = I_matched_pts(inlierIdx,:);

% Draw the lines to matched points
figure;showMatchedFeatures(image, port, I_inlier_pts, ref_inlier_pts, 'montage');
title('Showing match only with Inliers');
%%

tform = maketform(affine2d , double(tform_matrix));
[width, height,~] = size(ref_img);
corners = [0,0;height,0;height,width;0,width];
new_corners = tformfwd(tform, corners(:,1),corners(:,2));
figure;imshow(image);
patch(new_corners(:,1),new_corners(:,2),[0 1 0],'FaceAlpha',0.5);
%%