clear all
fs = 10;

% defining the sinc filter
sincNum = sin(pi*[-fs:1/fs:fs]); % numerator of the sinc function
sincDen = (pi*[-fs:1/fs:fs]); % denominator of the sinc function
sincDenZero = find(abs(sincDen) < 10^-10);
sincOp = sincNum./sincDen;
sincOp(sincDenZero) = 1; % sin(pix/(pix) =1 for x =0

alpha = 0;
cosNum = cos(alpha*pi*[-fs:1/fs:fs]);
cosDen = (1-(2*alpha*[-fs:1/fs:fs]).^2);
cosDenZero = find(abs(cosDen)<10^-10);
cosOp = cosNum./cosDen;
cosOp(cosDenZero) = pi/4;
gt_alpha0 = sincOp.*cosOp;
GF_alpha0 = fft(gt_alpha0,1024);

alpha = 0.5;
cosNum = cos(alpha*pi*[-fs:1/fs:fs]);
cosDen = (1-(2*alpha*[-fs:1/fs:fs]).^2);
cosDenZero = find(abs(cosDen)<10^-10);
cosOp = cosNum./cosDen;
cosOp(cosDenZero) = pi/4;
gt_alpha5 = sincOp.*cosOp;
GF_alpha5 = fft(gt_alpha5,1024);

alpha = 1;
cosNum = cos(alpha*pi*[-fs:1/fs:fs]);
cosDen = (1-(2*alpha*[-fs:1/fs:fs]).^2);
cosDenZero = find(abs(cosDen)<10^-10);
cosOp = cosNum./cosDen;
cosOp(cosDenZero) = pi/4;
gt_alpha1 = sincOp.*cosOp;
GF_alpha1 = fft(gt_alpha1,1024);

close all
figure
plot([-fs:1/fs:fs],[gt_alpha0],'b','LineWidth',2)
hold on
plot([-fs:1/fs:fs],[gt_alpha5],'m','LineWidth',2)
plot([-fs:1/fs:fs],[gt_alpha1],'c','LineWidth',2)
legend('alpha=0','alpha=0.5','alpha=1');
grid on
xlabel('time, t')
ylabel('amplitude, g(t)')
title('Time domain waveform of raised cosine pulse shaping filters')

figure
plot([-512:511]/1024*fs, abs(fftshift(GF_alpha0)),'b','LineWidth',2);
hold on
plot([-512:511]/1024*fs, abs(fftshift(GF_alpha5)),'m','LineWidth',2);

plot([-512:511]/1024*fs, abs(fftshift(GF_alpha1)),'c','LineWidth',2);
legend('alpha=0','alpha=0.5','alpha=1');
axis([-2 2 0 14])
grid on
xlabel('frequency, f')
ylabel('amplitude, |G(f)|')
title('Frequency domain representation of raised cosine pulse shaping filters')