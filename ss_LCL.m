%% Simulation properties
Tsim = 0.5;
step_time =5e-7;
%% LCL filter parameters

Lt = 1e-3; 
rt = 0.01*0;
Lg = 0.1e-3;
rg = 0.01*0;
Cf = 50e-6;
Vcc=400;
Rf = 0;
fs = 20e3;
Vpac = 180;

fres = sqrt((Lg+Lt)/(Cf*Lg*Lt))/(2*pi);

%% State Space plant model

A = [-(rt+Rf)/Lt -1/Lt Rf/Lt;
      1/Cf  0 -1/Cf;
      Rf/Lg 1/Lg -(rg+Rf)/Lg];

B = [Vcc/(2*Lt);
     0;
     0];

E = [0;
      0;
    -1/Lg;];

C = [0 0 1];

D = 0;

sys = ss(A,B,C,D)

%% Control Structure

% Controller parameters 
xi = 1.0;
wn = 2*pi*60;
w0 = wn*0.01;
% Integral + Ressonant controller
% Ac = [0 0 0;
%       0 0 1;
%       0 -wn*wn -2*xi*(wn*0.01)];
% 
% Bc = [1;
%       0;
%       1];
Ac = [0 1;
      -wn*wn -2*xi*(wn*0.01)];

Bc = [0;
      1];



%% Augmented System

[A_L, A_C]=size(A);
[Ac_L, Ac_C]=size(Ac);

[B_L, B_C]=size(B);
[Bc_L, Bc_C]=size(Bc);
Bw = E;
[Bw_L, Bw_C]=size(Bw);

Aa = [A zeros(A_L,Ac_C);
     -Bc*C Ac]
Ba = [B;
      zeros(Ac_C, B_C)];
Bwa = [Bw;
      zeros(Ac_C, B_C)];

Br = [zeros(A_C,1); Bc];

Ca = [C zeros(1,length(Ac))];

%% Feedback gains

%  Bessel polynomials
r = 2*pi*fs/10;% max radius that narrows in relation to 1 decade below fsw
ts = 0.5e-3;% settling time
alfa = 4/ts;
theta = 45;
sigma = alfa; %377*2;

p = [-3.948+j*13.553 -3.948-j*13.553 -6.040+j*5.601 -6.040-j*5.601 -9.394]/ts; %ITAE
p = [-4.11+6.314*j -4.11-6.314*j -5.927+3.081*j -5.927-3.081*j -6.448+0*j]/ts; %BESSEL
K = -place(Aa,Ba,p); % Negative due to the model

% Qctr = blkdiag(1e2,1e2,1e2,1e9,1e9);
% Rctr = 1e5*eye(1);
% Kctr = -lqr(Aa,Ba,Qctr,Rctr);
% %L = lqr(A,B,blkdiag(1e2,1e2,1e2),1e-4)';
% K = Kctr

%% eig(A+(Bu*K))
K1 = [K(1) K(2) K(3)];
K2 = [K(4) K(5)];

%% Closed loop poles of the controller
sys_cl_ctr = ss(Aa+Ba*K,Br,Ca,0);
sysd_cl_ctr = ss(Aa+Ba*K,Bwa,Ca,0);

[p_cl_ctr,z_cl_ctr] = pzmap(sys_cl_ctr) % Lists the positions of every closed loop P/Z

%% Controlability and Observability verification

Co = ctrb(A, B);

unco = length(A) - rank(Co);

if unco == 0
    disp('The system is controllable.');
else
    disp('The system is not controllable.');
end


Ob = obsv(A, C);

unob = length(A) - rank(Ob);

if unob == 0
    disp('The system is observable.');
else
    disp('The system is not observable.');
end

%% 
Ares = [0 1; -wn^2 -2*xi*w0]
Bres = [0; 1]
Ae = [A (Bres*C)'; 0 0 0 0 1; 0 0 -1 -wn^2 -2*xi*w0]
Be = [B; zeros(2,1)]
Ce = [C 0 1];
L = lqr(Ae,Be,blkdiag(1e5,1e5,1e5,1e5,1e5),1e-6)';