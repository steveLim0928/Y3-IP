close all

% TO CHANGE

T = 2; % Total duration
t = 1; % Gripped duration
d1 = 0.5; % getting to postion
d2 = 0.5; % return to position
N = 200; % number of 'points'
A = 0.011535; % distanced covered
filename = 'grip_Profile';

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
grip_profile = [n y];

% Export
save(filename,'grip_profile');

% Visualise plots
scatter(n1,y1);
hold on;
scatter(n2,y2);
scatter(n3,y3);