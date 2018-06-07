clear all; clc; close all;

%a = importdata('capture.txt');
a = -csvread('capture.txt');

s=2;
v=(a(s:s-1:end)-a(1:s-1:end-(s-1)))/(s-1);

figure(1)
subplot(2,1,1)
plot(a)
subplot(2,1,2)
plot(v)

n=500;
for ii=1:length(a)-n
    aa(ii)=mean(a(ii:ii+n));
end
for ii=1:length(v)-n
    vv(ii)=mean(v(ii:ii+n));
end

figure(2)
subplot(2,1,1)
plot(aa)
subplot(2,1,2)
plot(vv)


% sc=(pi/180); % give v=[rad/s], if sc=1 then v=[deg/s]
% v=sc*(a(2:end,2)-a(1:end-1,2))/.005;
% 
% v= [v' v(end)]';
% a=[a v];
% 
% for i=1:5
%     
% c=a(10:end,i);
% 
% figure(i)
% plot(c)
% 
% % n=500;
% % for ii=1:length(c)-n
% %     b(ii)=mean(c(ii:ii+n));
% % end
% % figure(i+10)
% % plot(b)
%     
% end
%     
    