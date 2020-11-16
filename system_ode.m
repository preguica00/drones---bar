function dydt = system_ode(t,y,u)
[H,Ts,drone1_info, drone2_info, bar_info] = system_info;
[m_drone,m_bar,m_sys,g, C_barra] = parameters;
 %% Unpack the state and input vectors


%states drone 1
phi_d1= y(1);
theta_d1= y(2);
yaw_d1= y(3);
wx_d1= y(4);
wy_d1= y(5);
wz_d1= y(6);

%states drone 2
phi_d2= y(7);
theta_d2= y(8);
yaw_d2= y(9);
wx_d2= y(10);
wy_d2= y(11);
wz_d2= y(12);

%states bar
x_bar= y(13);
y_bar= y(14);
z_bar = y(15);
dot_xbar= y(16);
dot_ybar= y(17);
dot_zbar= y(18);

phi_bar= y(19);
theta_bar= y(20);
yaw_bar= y(21);
wx_bar= y(22);
wy_bar= y(23);
wz_bar= y(24);

[Ixx,Iyy,Izz,a1,a2,a3,Ixx_sys,Iyy_sys,Izz_sys] = momentsofinertia(yaw_bar);
%control variables drone 1
T1_d1  = u(1);
taux_d1 = u(2);
tauy_d1 = u(3);
tauz_d1 = u(4);

%control variables drone 2
T2_d2  = u(5);
taux_d2 = u(6);
tauy_d2 = u(7);
tauz_d2 = u(8);


%%

line1_T1 = ( cos(yaw_d1)*sin(theta_d1)*cos(phi_d1) + sin(yaw_d1)*sin(phi_d1) )*T1_d1;
line2_T1 = ( sin(yaw_d1)*sin(theta_d1)*cos(phi_d1) - cos(yaw_d1)*sin(phi_d1) )*T1_d1;
line3_T1 = cos(theta_d1)*cos(phi_d1)*T1_d1;

%-----------------------------

line1_T2 = ( cos(yaw_d2)*sin(theta_d2)*cos(phi_d2) + sin(yaw_d2)*sin(phi_d2) )*T2_d2;
line2_T2 =  ( sin(yaw_d2)*sin(theta_d2)*cos(phi_d2) - cos(yaw_d2)*sin(phi_d2) )*T2_d2;
line3_T2 = cos(theta_d2)*cos(phi_d2)*T2_d2;

%Total force bar To=[line1;line2;line3]

line1 = line1_T1 + line1_T2;
line2 = line2_T1 + line2_T2;
line3 = line3_T1 + line3_T2;

%Total force bar To in inertial frame T^I_O=[TO_x;TO_y;TO_z]
TO_x   = line1;
TO_y = line2;
TO_z = line3;

%--------------------------------
% gamma_1 = cos(yaw_d1)*sin(theta_d1)*cos(phi_d1) + sin(yaw_d1)*sin(phi_d1);
% Pi_1 = sin(yaw_d1)*sin(theta_d1)*cos(phi_d1) - cos(yaw_d1)*sin(phi_d1);
% 
% gamma_2 = cos(yaw_d2)*sin(theta_d2)*cos(phi_d2) + sin(yaw_d2)*sin(phi_d2);
% Pi_2 = sin(yaw_d2)*sin(theta_d2)*cos(phi_d2) - cos(yaw_d2)*sin(phi_d2);

line11_tauO = 0; 
line21_tauO = -C_barra*cos(theta_d1)*cos(phi_d1)*T1_d1; 
line31_tauO = (-cos(yaw_bar-yaw_d1)*sin(phi_d1)+sin(yaw_d1-yaw_bar)*sin(theta_d1)*cos(phi_d1)*T1_d1)*C_barra; 


line12_tauO = 0; 
line22_tauO = -C_barra*cos(theta_d2)*cos(phi_d2)*T2_d2; 
line32_tauO = (-cos(yaw_bar-yaw_d2)*sin(phi_d2)+sin(yaw_d2-yaw_bar)*sin(theta_d2)*cos(phi_d1)*T2_d2)*C_barra; 


%tauO = [tauO_x;tauO_y;tauO_z]
tauO_x = line11_tauO-line12_tauO;
tauO_y = line21_tauO-line22_tauO;
tauO_z = line31_tauO-line32_tauO;
%%

%%Equations of motion
x_acceleration_bar = (TO_x)/(m_sys);
y_acceleration_bar= TO_y/(m_sys);
z_acceleration_bar = (-m_sys*g + TO_z) /(m_sys);

%derivadas velocidades angulares barra
% dot_wxO = tauO_x *Ixx_sys^(-1) + wy_bar*wz_bar*a1_sys;
% dot_wyO = tauO_y *Iyy_sys^(-1) + wx_bar*wz_bar*a2_sys;
% wx_bar =0;
% wy_bar=0;
% dot_wzO = tauO_z *Izz_sys^(-1) + wx_bar*wy_bar*a3_sys;
dot_wxO = tauO_x *Ixx_sys^(-1);
dot_wyO = tauO_y *Iyy_sys^(-1);
dot_wzO = tauO_z *Izz_sys^(-1);

%derivadas velocidades angulares drone 1
dot_wx1 = taux_d1* Ixx^(-1) + wy_d1*wz_d1*a1;
dot_wy1= tauy_d1* Iyy^(-1) + wx_d1*wz_d1*a2;
dot_wz1= tauz_d1* Izz^(-1) + wx_d1*wy_d1*a3;

%derivadas velocidades angulares drone 2
dot_wx2 = taux_d2* Ixx^(-1) + wy_d2*wz_d2*a1;
dot_wy2= tauy_d2* Iyy^(-1) + wx_d2*wz_d2*a2;
dot_wz2= tauz_d2* Izz^(-1) + wx_d2*wy_d2*a3;

%%
drone1 = [wx_d1;wy_d1;wz_d1;dot_wx1;dot_wy1;dot_wz1];
drone2 = [wx_d2;wy_d2;wz_d2;dot_wx2;dot_wy2;dot_wz2];
bar_trans = [dot_xbar;dot_ybar;dot_zbar;x_acceleration_bar;y_acceleration_bar;z_acceleration_bar];
bar_rot=[wx_bar;wy_bar;wz_bar;dot_wxO;dot_wyO;dot_wzO];

dydt=[drone1;drone2;bar_trans;bar_rot];

end