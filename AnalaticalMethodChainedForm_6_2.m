%We will be writing the code for the new Analytical solution to calculate
%the trajectory of a mobile robot in the presence of moving obstacles 
%Paper by Zhihua Qu and Jing Wang
%All units are SI units
%Intersection crossing scenario 

%Set timing variables
t0 = 0;%start maneuvre time
Ts = 2.5; %sampling rate
T = 40; %manuevure time
K_ = (T/Ts); %no of samples

%Car like Robot Parameters
R = 1; %sensor range
l = 0.8; %distance between two wheel axile centers
rho = 0.2; %wheel radius 

% Creating arrays to collect all the simulation data 
X =     double.empty(K_+1,0);
Y =     double.empty(K_+1,0);
THETA = double.empty(K_+1,0);
PHAI =  double.empty(K_+1,0);
U1 =    double.empty(K_,0);
U2 =    double.empty(K_,0);
V1 =    double.empty(K_,0);
V2 =    double.empty(K_,0);
A6 =    double.empty(K_+1,0);
Z1 =    double.empty(K_+1,0);
Z2 =    double.empty(K_+1,0);
Z3 =    double.empty(K_+1,0);
Z4 =    double.empty(K_+1,0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%Boundary Conditions%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%(x,y) mid point between front and rear axle centers
%theta vheichle orientation
%phai steering angle 
x_0 = 0;
y_0 = 10;
theta_0 = 0;
phai_0 = 0;
q_0 = [x_0; y_0; theta_0; phai_0]; 

X(1) = x_0;
Y(1) = y_0;
THETA(1) = theta_0;
PHAI(1) = phai_0;

%System inputs for the firt iteration
% U1 the angular velocity of the driving wheels
% U2 the steering rate of the front wheels

U1(1)= 1000; %1000 rad/sec our assumption
U2(1)=0; 

%final position of car like robot
x_f = 50;
y_f = 10;
theta_f = 0;
phai_f = 0;
q_f = [x_f; y_f; theta_f; phai_f];

X(K_+1) = x_f;
Y(K_+1) = y_f;
PHAI(K_+1)= phai_f ;
THETA(K_+1)= theta_f;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%Chained Form boundary conditions%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

z1_0 = x_0-(l/2)*(cos(theta_0));
z2_0 = tan(phai_0)/(l*cos(theta_0).^3);
z3_0 = tan(theta_0);
z4_0 = y_0-(l/2)*(sin(theta_0));
z_0 = [z1_0;z2_0; z3_0; z4_0];

Z1(1) = z1_0;
Z2(1) = z2_0;
Z3(1) = z3_0;
Z4(1) = z4_0;
V1(1) = U1(1)*rho*cos(theta_0);
V2(1) = (U2(1) + (3*sin(theta_0)/l*power(cos(theta_0),2))*power(sin(phai_0),2)*V1(1))/(l*power(cos(theta_0),3)*power(cos(phai_0),2));
    
z1_f = x_f-(l/2)*(cos(theta_f));
z2_f = tan(phai_f)/(l*cos(theta_f).^3);
z3_f = tan(theta_f);
z4_f = y_f-(l/2)*(sin(theta_f));   
z_f = [z1_f;z2_f; z3_f; z4_f];

Z1(K_+1) = z1_f;
Z2(K_+1) = z2_f;
Z3(K_+1) = z3_f;
Z4(K_+1) = z4_f;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%Set The Moving Obstacles%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

n0 = 5; %number of moving obstacles 
ri = 0.5; %radius of obstacles

%start position of obstacles
y_obs0 = 8; x_obs0 = 7; O_obs0 = [x_obs0;y_obs0]; 
X_obs0 = double.empty(K_,0);
Y_obs0 = double.empty(K_,0);

y_obs1 = 15; x_obs1 = 20; O_obs1 = [x_obs1;y_obs1];
X_obs1 = double.empty(K_,0);
Y_obs1 = double.empty(K_,0);

y_obs2 = 10; x_obs2 = 38; O_obs2 = [x_obs2;y_obs2];
X_obs2 = double.empty(K_,0);
Y_obs2 = double.empty(K_,0);

y_obs3 = 9; x_obs3 = 0; O_obs3 = [x_obs3;y_obs3];
X_obs3 = double.empty(K_,0);
Y_obs3 = double.empty(K_,0);

y_obs4 = 11; x_obs4 = 0; O_obs4 = [x_obs4;y_obs4];
X_obs4 = double.empty(K_,0);
Y_obs4 = double.empty(K_,0);

X_obs0(1) = x_obs0;
Y_obs0(1) = y_obs0;
X_obs1(1) = x_obs1;
Y_obs1(1) = y_obs1;
X_obs2(1) = x_obs2;
Y_obs2(1) = y_obs2;
X_obs3(1) = x_obs3;
Y_obs3(1) = y_obs3;
X_obs4(1) = x_obs4;
Y_obs4(1) = y_obs4;

%velocity vector for obstacles
Vy_obs0 = [ 0.5   0.4  0.6  0.6  0.5  0.4  0.6  0.6  0.6  0.6  0.6  0.6  0.6  0.6  0.6  0.6];
Vx_obs0 = [ 0.4   0.3  0.1  0.2  0.2  0.2  0.2  0.2  0.2  0.2  0.2  0.2  0.2  0.2  0.2  0.2];
Vy_obs1 = [-0.5  -0.4 -0.6 -0.6 -0.5 -0.4  0.6  0.6  0.6  0.6  0.6  0.6  0.6  0.6  0.6  0.6];
Vx_obs1=  [ 0     0.1  0.1  0.1  0.1  0.2  0.2  0.2  0.1  0.1  0.2  0.2  0.1  0.1  0.2  0.2];
Vy_obs2 = [-0.5  -0.4 -0.6 -0.6 -0.5 -0.4  0.6  0.6  0.6  0.6  0.6  0.6  0.6  0.6  0.6  0.6];
Vx_obs2 = [-0.1  -0.1 -0.2 -0.3  0.1  0.1  0.1  0.1 -0.1 -0.1 -0.1  0.1  0.1  0.1 -0.1  0.1];
Vy_obs3 = [0      0    0    0    0    0    0    0    0    0    0    0    0    0    0    0  ];
Vx_obs3 = [1      2    1    2    1    2    1    2    1    2    1    2    1    2    1    2  ];
Vy_obs4 = [0      0    0    0    0    0    0    0    0    0    0    0    0    0    0    0  ];
Vx_obs4 = [1      2    1    2    1    2    1    2    1    2    1    2    1    2    1    2  ];

%calculating obstacles' positions for the whole simulation
 for K = 1:K_
      x_obs0_k = x_obs0 + Ts * sum(Vx_obs0(1:K));
      x_obs1_k = x_obs1 + Ts * sum(Vx_obs1(1:K));
      x_obs2_k = x_obs2 + Ts * sum(Vx_obs2(1:K));
      x_obs3_k = x_obs3 + Ts * sum(Vx_obs3(1:K));
      x_obs4_k = x_obs4 + Ts * sum(Vx_obs4(1:K));
      
      y_obs0_k = y_obs0 + Ts * sum(Vy_obs0(1:K));   
      y_obs1_k = y_obs1 + Ts * sum(Vy_obs1(1:K));
      y_obs2_k = y_obs2 + Ts * sum(Vy_obs2(1:K));
      y_obs3_k = y_obs3 + Ts * sum(Vy_obs3(1:K));
      y_obs4_k = y_obs4 + Ts * sum(Vy_obs4(1:K));
      
      X_obs0(K+1) = x_obs0_k;
      Y_obs0(K+1) = y_obs0_k;
      
      X_obs1(K+1) = x_obs1_k;
      Y_obs1(K+1) = y_obs1_k;
      
      X_obs2(K+1) = x_obs2_k;
      Y_obs2(K+1) = y_obs2_k;
      
      X_obs3(K+1) = x_obs3_k;
      Y_obs3(K+1) = y_obs3_k;
      
      X_obs4(K+1) = x_obs4_k;
      Y_obs4(K+1) = y_obs4_k;
      
 end
X_obs = [X_obs0; X_obs1; X_obs2; X_obs3; X_obs4];
Y_obs = [Y_obs0; Y_obs1; Y_obs2; Y_obs3; Y_obs4];
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%Calculating the Trajectory for the%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%Car like robot%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (x_0 - (l/2)*(sin(theta_0)) ~= x_f - (l/2)*(sin(theta_f))) && (abs(theta_0-theta_f)< pi)
    for K = 1 : K_-1
        
     t=t0+(K+1)*Ts ;
      
     Z1(K+1) = Z1(1) + ((K)/K_)*(Z1(K_+1) - Z1(1)) ;
     Z4(K+1) = Z4(1) + ((K)/K_)*(Z4(K_+1) - Z4(1)) ; %Assumed 
      
     Z2(K+1) = Z2(K) + V2(K)*Ts ;
     Z3(K+1) = Z3(K) + ((Z1(K_+1) - Z1(1))/K_) * V2(K) * Ts ;
      
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     %%%%%%%%%%%%%%%%%%%%%%%%%%%calculating for a6%%%%%%%%%%%%%%%%%%%%%%%%
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
     z1_t = Z1(K+1) + (Z1(K_+1) - Z1(1))*(t-t0-K*Ts)/(T);
      
     f_z1_t = [1        z1_t        z1_t.^2     z1_t.^3    z1_t.^4      z1_t.^5];
      
     Y_k =    [Z4(K+1);    Z3(K+1);       Z2(K+1);       Y(K_)-0.5*sin(THETA(K_+1)) ;      tan(THETA(K_+1));        0];
      
     A_k =    [Z1(K+1).^6; 6*Z1(K+1).^5;  30*Z1(K+1).^4; Z1(K_+1).^6;   6*Z1(K_+1).^5;   30*Z1(K_+1).^4];
      
     B_k =    [1        Z1(K+1)        Z1(K+1).^2     Z1(K+1).^3      Z1(K+1).^4        Z1(K+1).^5; 
               0        1              2*Z1(K+1)      3*Z1(K+1).^2    4*Z1(K+1).^3      5*Z1(K+1).^4; 
               0        0              2              6*Z1(K+1)       12*Z1(K+1).^2     20*Z1(K+1).^3;
               1        Z1(K_+1)       Z1(K_+1).^2    Z1(K_+1).^3     Z1(K_+1).^4       Z1(K_+1).^5;
               0        1              2*Z1(K_+1)     3*Z1(K_+1).^2   4*Z1(K_+1).^3     5*Z1(K_+1).^4; 
               0        0              2              6*Z1(K_+1)      12*Z1(K_+1).^2    20*Z1(K_+1).^3];
     g2 = ((z1_t.^6 - f_z1_t / B_k) *(A_k)).^2;
       
     for j=1:n0
    % if ((X_obs0(K+1) > (z1_t - Vx_obs0(K)*t - ri - R)) && (X_obs0(K+1) < (z1_t - Vx_obs0(K)*t + 0.5*l + ri + R)))
        g1 = 2*(((z1_t.^6 - f_z1_t) / B_k)*A_k)*((f_z1_t / B_k) * Y_k - Y_obs(j,(K+1))); %ignore warning
        g0 = power(((f_z1_t /B_k)*Y_k - Y_obs0(K+1)),2) - power((z1_t - X_obs(j,(K+1))),2) - power((ri + R + 0.5*l),2);
        A6(2*j-1) = (-g1 + sqrt(power(g1,2)-4*g2*g0))/(2*g2);
        A6(2*j) = (-g1 - sqrt(power(g1,2)-4*g2*g0))/(2*g2);
     end
     
      a6 = min(A6);
      if (isreal(a6) == 0) 
        a6 = abs(a6);
      end
      a = B_k \ (Y_k - A_k*a6);
      a = a.';
     
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %%%%%%%%%%%%%%%%%%find steering inputs in chained form%%%%%%%%%%%%%%
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       
      V1(K+1) = (Z1(K_) - Z1(1))/T;
      V2(K+1) = 6*V1(K+1)*(a(4) + 4*a(5)*Z1(K+1) + 10*a(6)*Z1(K+1).^2 + 20*a6*Z1(K+1).^3) + 24*(t-t0-K*Ts)*V1(K+1).^2*(a(5) + 5*a(6)*Z1(K+1) + 15*a6*Z1(K+1).^2) + 60*power((t-t0-K*Ts),2)*V1(K+1).^3*(a(6) + 6*a6*Z1(K+1)) + 120*a6*V1(K+1).^4*power((t-t0-K*Ts),3);
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %%%%%%%%%%%%%%%%%%%%%%find boundry conditions%%%%%%%%%%%%%%%%%%%%%%%
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      THETA(K+1) = atan(Z3(K+1));
      X(K+1) = Z1(K+1) + (l/2)*(cos(THETA(K+1)));
      Y(K+1) = Z4(K+1) + (l/2)*(sin(THETA(K+1)));
      PHAI(K+1) = atan(Z2(K+1)*l*power(cos(THETA(K+1)),3));
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %%%%%%%%%%%%%%%%%%%%%%find steering inputs%%%%%%%%%%%%%%%%%%%%%%%%%%
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       
      U1(K+1) = (V1(K+1))/(rho*cos(THETA(K+1)));
      U2(K+1) = ((-3*sin(THETA(K+1))*power(sin(PHAI(K+1)),3)*V1(K+1))/(l*power(cos(THETA(K+1)),2)))+(l*power(cos(THETA(K+1)),3)*power(cos(PHAI(K+1)),2)*V2(K+1));
      
    end
end
X %prinitng values of X
Y %printing values of Y 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%Plotting the Obstacles%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

plot(X_obs0,Y_obs0,'r--o') %plotting obs 1
hold on                  %plotting the rest on the same graph
plot(X_obs1,Y_obs1,'g--o') %plotting obs 2
plot(X_obs2,Y_obs2,'y--o') %plotting obs 3
plot(X_obs3,Y_obs3,'m--o') %plotting obs 4
plot(X_obs4,Y_obs4,'k--o') %plotting obs 5

%%%%%%%%%%%%%%%%%%%%%%%%%Writing point number on each point%%%%%%%%%%%%%%%%
for K = 1 : K_+1
    t2 = text(X_obs0(K),Y_obs0(K),num2str(K-1));
    t3 = text(X_obs1(K),Y_obs1(K),num2str(K-1));
    t4 = text(X_obs2(K),Y_obs2(K),num2str(K-1));
    t5 = text(X_obs3(K),Y_obs3(K),num2str(K-1));
    t6 = text(X_obs4(K),Y_obs4(K),num2str(K-1));
end

%%%%%%%%%%%%%%%%%%%Plotting Path%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plot(X,Y,'b->')
%%%%%%%%%%%%%%%%%%writing number on each point%%%%%%%%%%%%%%%%
for K = 1 : K_+1
t = text(X(K),Y(K),num2str(K-1));
end
legend('Obs 1','Obs 2','Obs 3','Obs 4','Obs 5','Mobile Robot') %Show legend
title('Trajectory Planning') %graph title
xlabel('X') %x-axis title
ylabel('Y') %y-axis title
str = {'Sampling Time ='  num2str(Ts), 'Intersection Crossing Scenario'};
text(5,25,str)
hold off
%s = [X(6),Y(6);X_obs0(6),Y_obs0(6)];
%d = pdist(s)