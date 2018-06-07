clear all; clc; close all;

%a = importdata('capture.txt');
a = csvread('capture.txt');
a1=a(:,1);
a2=a(:,2);
a3=a(:,3);

for i=1:3
    
c=a(:,i);

figure(i)
plot(c)

n=500;
for ii=1:length(c)-n
    b(ii)=mean(c(ii:ii+n));
end
figure(i+10)
plot(b)
    
end
    
    