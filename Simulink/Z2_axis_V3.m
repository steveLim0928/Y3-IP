function z1_profile = Z1_axis_V3 (time, T, N, A,Ts)

N = N+1;
N1 = int16(N/T*time(1));+1
N2 = int16(N/T*time(2));
N3 = int16(N/T*time(3));
N4 = int16(N/T*time(4));
N5 = int16(N/T*time(5));
N6 = int16(N/T*(time(6)+time(7)));

N8 = int16(N/T*time(8));
N9 = int16(N/T*time(9));
N10 = N - N1-N2-N3-N4-N5-N6-N8-N9;

l1 = time(1);
l2 = l1+time(2);
l3 = l2+time(3);
l4 = l3+time(4);
l5 = l4+time(5);
l6 = l5+time(6)+time(7);

l8 = l6+time(8);
l9 = l8+time(9);

n1 = linspace(0,l1,N1);
n2 = linspace(l1+Ts,l2,N2);
n3 = linspace(l2+Ts,l3,N3);
n4 = linspace(l3+Ts,l4,N4);
n5 = linspace(l4+Ts,l5,N5);
n6 = linspace(l5+Ts,l6,N6);

n8 = linspace(l6,l8,N8);
n9 = linspace(l8,l9,N9);
n10 = linspace(l9,T,N10);

C1 = A(1)-A(1)*0.5;
C2 = A(1)*0.5-A(2);
E1 = A(1)+A(1)*0.5;
E2 = A(1)*0.5+A(2);

y1 = zeros(N1);
y1 = y1(:,1);

% Phase 1, reshape to fit 1xn
y2 = (-A(1)*cos(1/time(2)*pi*(n2-l1)) + A(1))/2;
y2 = reshape(y2, [N2,1]);

% Phase 2, only get 1st column
y3 = A(1)*ones(N3);
y3 = y3(:,1);

% Phase 3, reshape to fit 1xn
y4 = (C1*cos(1/time(4)*pi*(n4-l3)) + E1)/2;
y4 = reshape(y4, [N4,1]);

y5 = A(1)*0.5*ones(N5);
y5 = y5(:,1);

y6 = (C2*cos(1/(time(6)+time(7))*pi*(n6-l5)) + E2)/2;
y6 = reshape(y6, [N6,1]);



% Phase 5, only get 1st column
y8 = A(2)*ones(N8);
y8 = y8(:,1);

% Phase 6, reshape to fit 1xn
y9 = (A(2)*cos(1/time(9)*pi*(n9-l8)) + A(2))/2;
y9 = reshape(y9, [N9,1]);

y10 = zeros(N10);
y10 = y10(:,1);

% join x and y axis 
y = [y1; y2; y3; y4; y5; y6; y8; y9; y10];
n = [reshape(n1, [N1,1]); reshape(n2, [N2,1]); reshape(n3, [N3,1]); reshape(n4, [N4,1]); reshape(n5, [N5,1]); reshape(n6, [N6,1]); reshape(n8, [N8,1]); reshape(n9, [N9,1]); reshape(n10, [N10,1])];
y(1) = [];
n(1) = [];
z1_profile = [n y];

% Visualise plots
% scatter(n1,y1);
% hold on;
% scatter(n2,y2);
% scatter(n3,y3);
% scatter(n4,y4);
% scatter(n5,y5);
% scatter(n6,y6);
% scatter(n8,y8);
% scatter(n9,y9);
% scatter(n10,y10);
end
