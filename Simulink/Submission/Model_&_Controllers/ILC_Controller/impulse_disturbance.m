function disturbance = impulse_disturbance (time, T, N, A, Ts)

N = N+1;
N1 = int16(N/T*time(1))+1;
N2 = int16(N/T*time(2));
N3 = int16(N/T*time(3));
N4 = int16(N/T*time(4));
N5 = N-N1-N2-N3-N4;

l1 = 0+time(1);
l2 = l1+time(2);
l3 = l2+time(3);
l4 = l3+time(4);

n1 = linspace(0,time(1),N1);
n2 = linspace(l1+Ts,l2,N2);
n3 = linspace(l2+Ts,l3,N3);
n4 = linspace(l3+Ts,l4,N4);
n5 = linspace(l4+Ts,T,N5);


y1 = zeros(N1);
y1 = y1(:,1);

% Phase 1, reshape to fit 1xn
y2 = (-A*cos(1/time(2)*pi*(n2-l1)) + A)/2;
y2 = reshape(y2, [N2,1]);

% Phase 2, only get 1st column
y3 = A(1)*ones(N3);
y3 = y3(:,1);

% Phase 3, reshape to fit 1xn
y4 = (A*cos(1/time(4)*pi*(n4-l3)) + A)/2;
y4 = reshape(y4, [N4,1]);

% Phase 6, reshape to fit 1xn
y5 = zeros(N5);
y5 = y5(:,1);

% join x and y axis 
y = [y1; y2; y3; y4; y5];
n = [reshape(n1, [N1,1]); reshape(n2, [N2,1]); reshape(n3, [N3,1]); reshape(n4, [N4,1]); reshape(n5, [N5,1])];
y(1) = [];
n(1) = [];
disturbance = [n y];

% Visualise plots
% scatter(n1,y1);
% hold on;
% scatter(n2,y2);
% scatter(n3,y3);
% scatter(n4,y4);
% scatter(n5,y5);
end