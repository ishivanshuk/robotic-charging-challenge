a=imread("images_test\5.jpeg");
%imtool(a);

J = medfilt3(a,[3 3 3]); %median filter
%imtool(J);

I = double(J) / 255;
    HSV = rgb2hsv(J);
    H = HSV(:,:,1);
    S = HSV(:,:,2);
    I = sum(I, 3)./3;
 
% Creating the HSL Image
  HSI = zeros(size(J));
  HSI(:,:,1) = H;
  HSI(:,:,2) = S;
  HSI(:,:,3) = I;
  %imtool(HSI);

G = rgb2gray(HSI);
Hist = imhist(G);
[val, idx] = max(Hist);
idx = 1.05*idx;
seg_img = zeros(size(J));
[x , y, z ]=size(J);

for j = 1 : x
        for k = 1 : y
            for l = 1:z
                    if J(j, k, l) >= idx
                        seg_img(j, k, l)=255;
                    else 
                        seg_img(j,k,l)=0;
                    end
            end
        end
end

for j = 1 : x
        for k = 1 : y
            count=0;
            for l = 1:z
                if seg_img(j, k ,l)>0
                    count=count+1;
                end
            end
            
        end
end
%figure;
%imshow(seg_img);
%title('segmented image');

se = strel('line',11,90);

%obtain annular region
dil = imdilate(seg_img, se);
erod = imerode( seg_img, se);
annular = dil-erod;
annular  = imgaussfilt3(annular);
gray = rgb2gray(annular);
%figure;
%imshow(annular);
%title('annular')
%%

[centers, radii ] = imfindcircles(HSI, [6, 30], Sensitivity=0.9, ObjectPolarity="dark");
location= [0,0];
for i=1:size(centers, 1)
    location(1) = location(1) + centers(i, 1);
end
location(1) = location(1)/size(centers, 1);
for i=1:size(centers, 1)
    location(2) = location(2) + centers(i, 2);
end
    location(2) = location(2)/size(centers, 1);
%%
figure;
imshow(a);
h = viscircles(centers,radii);
img = insertShape(a,'circle',[location(1) location(2) 10]);
figure;
imshow(img);
hold on;
plot(location(1),location(2),'*r')
