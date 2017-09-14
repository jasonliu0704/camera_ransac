% Read in the input images
left = 'blocks1.gif';
right = 'blocks2.gif';
l = imread(left);
r = imread(right);

% Convert to double
l = double(l);
r = double(r);
% Pick the corresponding corners
xl = clickPoints(l, 10, 1, 1);
xr = clickPoints(r, 10, 1, 1);
xl = [xl; ones(size(xl(1,:)))];
xr = [xr; ones(size(xr(1,:)))];



a = [1 2 3;4 5 6];
%a(3,:) = ones(size(a,1),1);
a = [a; ones(size(a(1,:)))];
disp(a);

% Compute the fundamental matrix
F = computeF(xr, xl);
disp(xl(:,9)'*F*xr(:,9));
% Compute the epipolar lines in the left image
leftlines = F*xr;
% Compute the epipolar lines in the right image
rightlines = F'*xl;
% Draw epipolar lines on the left image
figure; imshow(l,[]); axis image; hold on;
drawLine(leftlines(:,1), l, 'yellow');
drawLine(leftlines(:,3), l, 'cyan');
drawLine(leftlines(:,5), l, 'green');
drawLine(leftlines(:,7), l, 'magenta');
drawLine(leftlines(:,9), l, 'red');
hold off;
% Draw epipolar lines on the right image
figure; imshow(r,[]); axis image; hold on;
drawLine(rightlines(:,1), r, 'yellow');
drawLine(rightlines(:,3), r, 'cyan');
drawLine(rightlines(:,5), r, 'green');
drawLine(rightlines(:,7), r, 'magenta');
drawLine(rightlines(:,9), r, 'red');
hold off;


% epipole
disp(zeros(1,3)*inv(F));
disp(zeros(1,3)*inv(F'));


function [F] = computeF(xl, xr)
%A = zeros(10,9);
%for i = 1:10
%    u = xl(1,i);
%    v = xl(2,i);
%    up = xr(1,i);
%    vp = xr(2,i);
%    A(i,:) = [u*up v*up up u*vp v*vp vp u v 1];
%end

A=[xr(1,:)'.*xl(1,:)'	xr(1,:)'.*xl(2,:)' xr(1,:)' ...
xr(2,:)'.*xl(1,:)' xr(2,:)'.*xl(2,:)' xr(2,:)'	...
xl(1,:)' xl(2,:)' ones(10,1)	];
disp(A);
[U,D,V] = svd(A);
F = reshape(V(:,9),3,3)';
[U,D,V] = svd(F);
F = U*diag([D(1,1) D(2,2) 0])*V';
end
