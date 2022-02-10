close all

% TO CHANGE

T = 4; % Total duration
D1 = 0.4; % Grip deadtime
D2 = 0.4; % Release deatime
d1 = 0.8; % TO duration
d2 = 0.8; % TRANSITION 1 duration
d3 = 0.8; % TRANSITION 2 duration
d4 = 0.8; % RETURN duration
A1 = 0.5; % TO distance
A2 = 0.3; % Release distance

N = 400; % Number of 'POINTS'

filename = 'Profile_x';

% END TO CHANGE



% Calculate each section number of points
N1 = int16(N/T*d1);
N2 = int16(N/T*D1);
N3 = int16(N/T*d2);
N4 = int16(N/T*d3);
N5 = int16(N/T*D2);
N6 = N-N1-N2-N3-N4-N5;

% Determine end of phase 1 and 2 time
l1 = 0+d1;
l2 = l1+D1;
l3 = l2+d3;
l4 = l3+d4;
l5 = l4+D2;

% linearly spread space for all 3 phases
n1 = linspace(0,d1,N1);
n2 = linspace(l1,l2,N2);
n3 = linspace(l2,l3,N3);
n4 = linspace(l3,l4,N4);
n5 = linspace(l4,l5,N5);
n6 = linspace(l5,T,N6);

% Phase 1, reshape to fit 1xn
y1 = (-A1*cos(1/d1*pi*n1) + A1)/2;
y1 = reshape(y1, [N1,1]);

% Phase 2, only get 1st column
y2 = A1*ones(N2);
y2 = y2(:,1);

% Phase 3, reshape to fit 1xn
y3 = (A1*cos(1/d2*pi*(n3-l2)) + A1)/2;
y3 = reshape(y3, [N3,1]);

% Phase 4, reshape to fit 1xn
y4 = (-A2*cos(1/d3*pi*(n4-l3)) + A2)/2;
y4 = reshape(y4, [N4,1]);

% Phase 5, only get 1st column
y5 = A2*ones(N5);
y5 = y5(:,1);

% Phase 6, reshape to fit 1xn
y6 = (A2*cos(1/d4*pi*(n6-l5)) + A2)/2;
y6 = reshape(y6, [N6,1]);

% join x and y axis 
y = [y1; y2; y3; y4; y5; y6];
n = [reshape(n1, [N1,1]); reshape(n2, [N2,1]); reshape(n3, [N3,1]); reshape(n4, [N4,1]); reshape(n5, [N5,1]); reshape(n6, [N6,1])];
x_profile = [n y];

% Export
save(filename,'x_profile');

% Visualise plots
scatter(n1,y1);
hold on;
scatter(n2,y2);
scatter(n3,y3);
scatter(n4,y4);
scatter(n5,y5);
scatter(n6,y6);

scatter(grip_profile(:,1),grip_profile(:,2))
scatter(z_profile(:,1),z_profile(:,2))
scatter(y_profile(:,1),y_profile(:,2))


