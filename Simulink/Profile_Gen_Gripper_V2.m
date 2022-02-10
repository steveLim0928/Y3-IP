close all

% TO CHANGE

T = 4; % Total duration
t = 2; % Gripped duration
d1 = 0.2; % getting to postion
d2 = 0.2; % return to position
N = 400; % number of 'points'
A = 0.0117; % distanced covered
filename = 'Profile_grip';

% END TO CHANGE



% Calculate each section number of points
N0 = int16(N/T*0.8);

N000 = int16(N/T*0.8);
N1 = int16(N/T*d1);
N2 = int16(N/T*t);
N3 = N-N1-N2-N0-N000;

% Determine end of phase 1 and 2 time
l0 = 0+0.8;
l1 = l0+d1;
l2 = l1+t;
l000 = l2+d2;

% linearly spread space for all 3 phases
n0 = linspace(0,l0,N0);
n1 = linspace(l0,l1,N1);
n2 = linspace(l1,l2,N2);
n3 = linspace(l2,l000,N3);
n000 = linspace(l000,T,N000);

% Phase 1, reshape to fit 1xn
y1 = (-A*cos(1/d1*pi*n1) + A)/2;
y1 = reshape(y1, [N1,1]);

% Phase 2, only get 1st column
y2 = A*ones(N2);
y2 = y2(:,1);

% Phase 3, reshape to fit 1xn
y3 = (A*cos(1/d2*pi*(n3-l2)) + A)/2;
y3 = reshape(y3, [N3,1]);

y0 = zeros(N0);
y0 = y0(:,1);

y000 = zeros(N000);
y000 = y000(:,1);

% join x and y axis 
y = [y0; y1; y2; y3; y000];
n = [reshape(n0, [N0,1]); reshape(n1, [N1,1]); reshape(n2, [N2,1]); reshape(n3, [N3,1]); reshape(n000, [N000,1])];
grip_profile_V2 = [n y];

% Export
save(filename,'grip_profile_V2');

% Visualise plots
scatter(n1,y1);
hold on;
scatter(n2,y2);
scatter(n3,y3);
scatter(n,y);