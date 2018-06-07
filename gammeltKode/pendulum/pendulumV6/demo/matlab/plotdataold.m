clear all; clc; close all;

%a = importdata('capture.txt');
a = csvread('capture.txt');

plot(a)

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
% n=500;
% for ii=1:length(c)-n
%     b(ii)=mean(c(ii:ii+n));
% end
% figure(i+10)
% plot(b)
%     
% end
%     
%     