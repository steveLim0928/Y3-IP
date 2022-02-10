close all

% TO CHANGE

T = 4; % Total duration
D1 = 0.2; % Grip deadtime
D2 = 0.2; % Release deatime
d1 = 0.1; % TO duration
d2 = 0.1; % TRANSITION 1 duration
d3 = 0.1; % TRANSITION 2 duration
d4 = 0.1; % RETURN duration
d5 = 0.1; % Raise slightly
A1 = 0.038; % distance covered
A2 = A1*0.6;
A3 = 0.028;

C1 = A1-A2;
E1 = A1+A2;
C2 = A2-A3;
E2 = A2+A3;

N = 400; % Number of 'POINTS'

filename = 'Profile_z';

% END TO CHANGE



% Calculate each section number of points
N0 = int16(N/T*0.8);
% N00 = int16(N/T*(1.6-d5));
N000 = int16(N/T*0.8);
N1 = int16(N/T*d1);
N2 = int16(N/T*D1);
N3 = int16(N/T*d2);
N7 = int16(N/T*d5);
N4 = int16(N/T*(d3+1.5));
N5 = int16(N/T*D2);
N6 = N-N1-N2-N3-N4-N5-N0-N000-N7;

% Determine end of phase 1 and 2 time
l0 = 0+0.8;
l1 = l0+d1;
l2 = l1+D1;
l3 = l2+d2;
l7 = l3+d5;
% l00 = l7+1.6-d5;
l4 = l7+d3+1.5;
l5 = l4+D2;
l000 = l5+d4;

% linearly spread space for all 3 phases
n0 = linspace(0,l0,N0);
n1 = linspace(l0,l1,N1);
n2 = linspace(l1,l2,N2);
n3 = linspace(l2,l3,N3);
n7 = linspace(l3,l7,N7);
% n00 = linspace(l7,l00,N00);
n4 = linspace(l7,l4,N4);
n5 = linspace(l4,l5,N5);
n6 = linspace(l5,l000,N6);
n000 = linspace(l000,T,N000);

% Phase 1, reshape to fit 1xn
y1 = (-A1*cos(1/d1*pi*n1) + A1)/2;
y1 = reshape(y1, [N1,1]);

% Phase 2, only get 1st column
y2 = A1*ones(N2);
y2 = y2(:,1);

% Phase 3, reshape to fit 1xn
y3 = (C1*cos(1/d2*pi*(n3-l2)) + E1)/2;
y3 = reshape(y3, [N3,1]);

y7 = A2*ones(N7);
y7 = y7(:,1);

% Phase 4, reshape to fit 1xn
y4 = (C2*cos(1/(d3+1.5)*pi*(n4-l7)) + E2)/2;
y4 = reshape(y4, [N4,1]);

% Phase 5, only get 1st column
y5 = A3*ones(N5);
y5 = y5(:,1);

% Phase 6, reshape to fit 1xn
y6 = (A3*cos(1/d4*pi*(n6-l5)) + A3)/2;
y6 = reshape(y6, [N6,1]);

y0 = zeros(N0);
y0 = y0(:,1);
% y00 = zeros(N00);
% y00 = y00(:,1);
y000 = zeros(N000);
y000 = y000(:,1);

% join x and y axis 
y = [y0; y1; y2; y3; y7; y4; y5; y6; y000];
n = [reshape(n0, [N0,1]); reshape(n1, [N1,1]); reshape(n2, [N2,1]); reshape(n3, [N3,1]); reshape(n7, [N7,1]); reshape(n4, [N4,1]); reshape(n5, [N5,1]); reshape(n6, [N6,1]); reshape(n000, [N000,1])];
z_profile_V2 = [n y];

% Export
save(filename,'z_profile_V2');

% Visualise plots
scatter(n1,y1);
hold on;
scatter(n2,y2);
scatter(n3,y3);
scatter(n4,y4);
scatter(n5,y5);
scatter(n6,y6);
scatter(n7,y7);
scatter(n0,y0);
scatter(n000,y000);
scatter(n,y);

scatter(grip_profile_V2(:,1),grip_profile_V2(:,2))


