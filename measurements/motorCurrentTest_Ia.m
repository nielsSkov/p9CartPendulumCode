%change path to directory containing the project files
cd ~/syncDrive/uni/9thSem/project/p9CartPendulumCode/measurements/

clear all; close all; clc                                                  %#ok<CLALL>

%set LaTeX as default interpreter
set( groot, 'defaultAxesTickLabelInterpreter',      'latex' );
set( groot, 'defaultLegendInterpreter',             'latex' );
set( groot, 'defaultTextInterpreter',               'latex' );
set( groot, 'defaultColorbarTickLabelInterpreter',  'latex' );
set( groot, 'defaultPolaraxesTickLabelInterpreter', 'latex' );
set( groot, 'defaultTextarrowshapeInterpreter',     'latex' );
set( groot, 'defaultTextboxshapeInterpreter',       'latex' );

%------Ia = 4A motor test--------------------------------------------------

data = csvread('Ia_4A_1.csv',2,0);

t_4A = data(:,1);
A_4A = data(:,2)*10; %scaled to obtain unit [A]  .. convertion: 100 mV/A

%range of data between .1 and .5 s
range1 = ( .1 < t_4A ) & ( t_4A < .5 );

%range of data between .6 and .9 s
range2 = ( .6 < t_4A ) & ( t_4A < .9 );

rms1 = -rms( A_4A(range1) )*ones(size(t_4A));
rms2 = rms( A_4A(range2) )*ones(size(t_4A));

plot(t_4A,A_4A)
hold on
plot(t_4A,rms1)
plot(t_4A,rms2)

xlim( [ min(t_4A) max(t_4A) ] )
grid on, grid minor
xlabel('$t$ [s]')
ylabel('$I_a$ [A]')

%------reverse Ia = 4A motor test------------------------------------------

data = csvread('Ia_4A_2.csv',2,0);

t_4A = data(:,1);
A_4A = data(:,2)*10; %scaled to obtain unit [A]  .. convertion: 100 mV/A

%range of data between .1 and .5 s
range1 = ( .1 < t_4A ) & ( t_4A < .5 );

%range of data between .6 and .9 s
range2 = ( .6 < t_4A ) & ( t_4A < .9 );

rms1 = -rms( A_4A(range1) )*ones(size(t_4A));
rms2 = rms( A_4A(range2) )*ones(size(t_4A));

figure
plot(t_4A,A_4A)
hold on
plot(t_4A,rms1)
plot(t_4A,rms2)

xlim( [ min(t_4A) max(t_4A) ] )
grid on, grid minor
xlabel('$t$ [s]')
ylabel('$I_a$ [A]')


%------reverse Ia = 4A motor test------------------------------------------

dataUa = csvread('Ia_4A_and_Ua_1.csv',2,0);
dataIa = csvread('Ia_4A_and_Ua_2.csv',2,0);

t  = dataIa(:,1);
Ia = dataIa(:,2)*10; %scaled to obtain unit [A]  .. convertion: 100 mV/A
Ua = dataUa(:,2);

%range of data between .1 and .5 s
%range1 = ( .1 < t_4A ) & ( t_4A < .5 );

%range of data between .6 and .9 s
%range2 = ( .6 < t_4A ) & ( t_4A < .9 );

%rms1 = -rms( A_4A(range1) )*ones(size(t_4A));
%rms2 = rms( A_4A(range2) )*ones(size(t_4A));

figure
plot(t,Ia)
grid on, grid minor
xlabel('$t$ [s]')
ylabel('$I_a$ [V]')
xlim( [ -.1 1.1 ] )

figure
plot(t,Ua)

%xlim( [ min(t_4A) max(t_4A) ] )
grid on, grid minor
xlabel('$t$ [s]')
ylabel('$U_a$ [A]')
xlim( [ -.1 1.1 ] )


Ra = Ua./Ia;

figure
plot(t,Ra)

%xlim( [ min(t_4A) max(t_4A) ] )
grid on, grid minor
xlabel('$t$ [s]')
ylabel('$R_a$ [A]')
xlim( [ -.1 1.1 ] )