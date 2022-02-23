function g1_profile = G1_axis_V3 (time, T, N, A)

N1 = int16(N/T*time(1));
N2 = int16(N/T*time(2));
N3 = int16(N/T*time(3));
N4 = int16(N/T*time(4));
N5 = int16(N/T*time(5));
N6 = int16(N/T*time(6));
N7 = N-N1-N2-N3-N4-N5-N6;

l1 = 0+time(1);
l2 = l1+time(2);
l3 = l2+time(3);
l4 = l3+time(4);
l5 = l4+time(5);
l6 = l5+time(6);

n1 = linspace(0,l1,N1);
n2 = linspace(l1,l2,N2);
n3 = linspace(l2,l3,N3);
n4 = linspace(l3,l4,N4);
n5 = linspace(l4,l5,N5);
n6 = linspace(l5,l6,N6);
n7 = linspace(l6,T,N7);

C = A(1)-A(2);
E = A(1)+A(2);

y1 = zeros(N1);
y1 = y1(:,1);

% Phase 1, reshape to fit 1xn
y2 = (-A(1)*cos(1/time(2)*pi*(n2-l1)) + A(1))/2;
y2 = reshape(y2, [N2,1]);

% Phase 2, only get 1st column
y3 = A(1)*ones(N3);
y3 = y3(:,1);

% Phase 3, reshape to fit 1xn
y4 = (C*cos(1/time(4)*pi*(n4-l3)) + E)/2;
y4 = reshape(y4, [N4,1]);

% Phase 5, only get 1st column
y5 = A(2)*ones(N5);
y5 = y5(:,1);

% Phase 6, reshape to fit 1xn
y6 = (A(2)*cos(1/time(6)*pi*(n6-l5)) + A(2))/2;
y6 = reshape(y6, [N6,1]);

y7 = zeros(N7);
y7 = y7(:,1);

% join x and y axis 
y = [y1; y2; y3; y4; y5; y6; y7];
n = [reshape(n1, [N1,1]); reshape(n2, [N2,1]); reshape(n3, [N3,1]); reshape(n4, [N4,1]); reshape(n5, [N5,1]); reshape(n6, [N6,1]); reshape(n7, [N7,1])];
g1_profile = [n y];

% Visualise plots
scatter(n1,y1);
hold on;
scatter(n2,y2);
scatter(n3,y3);
scatter(n4,y4);
scatter(n5,y5);
scatter(n6,y6);
scatter(n7,y7);

end