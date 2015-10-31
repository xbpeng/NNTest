r_data = load('r_data.txt');
g_data = load('g_data.txt');
b_data = load('b_data.txt');
text_data = load('tex_data.txt');

img = r_data;
img(:,:,2) = g_data;
img(:,:,3) = b_data;

gray_data = (r_data + g_data + b_data) / 3;
img_gray = text_data;
img_gray(:,:,2) = text_data;
img_gray(:,:,3) = text_data;

image(img_gray);
%image(img);