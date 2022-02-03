close all

% TO CHANGE

T = 10; %Total duration
t = 5; % Gripped duration
d1 = 3; % getting to postion
d2 = 2; % return to position
N = 1000; % number of 'points'
A = 2; % distanced covered
filename = 'z_Profile';

% END TO CHANGE

% Calculate each section number of points
N1 = int16(N/T*d1);
N2 = int16(N/T*t);
N3 = N-N1-N2;

% Determine end of phase 1 and 2 time
l1 = 0+d1;
l2 = l1+t;

% linearly spread space for all 3 phases
n1 = linspace(0,d1,N1);
n2 = linspace(l1,l2,N2);
n3 = linspace(l2,T,N3);

% Phase 1, reshape to fit 1xn
y1 = -A*cos(0.5/d1*pi*n1) + A;
y1 = reshape(y1, [N1,1]);

% Phase 2, only get 1st column
y2 = A*ones(N2);
y2 = y2(:,1);

% Phase 3, reshape to fit 1xn
y3 = A*cos(0.5/d2*pi*(n3-l2));
y3 = reshape(y3, [N3,1]);

% join x and y axis 
y = [y1; y2; y3];
n = [reshape(n1, [N1,1]); reshape(n2, [N2,1]); reshape(n3, [N3,1])];
z_profile = [n y];

% Export
save(filename,'z_profile');

% Visualise plots
scatter(n1,y1);
hold on;
scatter(n2,y2);
scatter(n3,y3);