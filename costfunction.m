function cost = costfunction(y, H,k)

[H,Ts,drone1_info, drone2_info, bar_info] = system_info;
%% Unpacking drones
% drone 1 
id1_T1 = drone1_info(1,:);
id1_taux1 = drone1_info(2,:);
id1_tauy1 = drone1_info(3,:);
id1_tauz1 = drone1_info(4,:);

id1_phi1 = drone1_info(5,:);
id1_theta1 = drone1_info(6,:);
id1_yaw1 = drone1_info(7,:);

id1_wx1 = drone1_info(8,:);
id1_wy1 = drone1_info(9,:);
id1_wz1 = drone1_info(10,:);


% drone 2
id2_T2 = drone2_info(1,:);
id2_taux2 = drone2_info(2,:);
id2_tauy2 = drone2_info(3,:);
id2_tauz2 = drone2_info(4,:);

id2_phi2 = drone2_info(5,:);
id2_theta2 = drone2_info(6,:);
id2_yaw2 = drone2_info(7,:);

id2_wx2 = drone2_info(8,:);
id2_wy2 = drone2_info(9,:);
id2_wz2 = drone2_info(10,:);

% bar
xb = bar_info(1,:);
yb = bar_info(2,:);
zb = bar_info(3,:);
dot_xb = bar_info(4,:);
dot_yb = bar_info(5,:);
dot_zb = bar_info(6,:);

phi_b = bar_info(7,:);
theta_b = bar_info(8,:);
yaw_b = bar_info(9,:);
wx_b = bar_info(10,:);
wy_b = bar_info(11,:);
wz_b = bar_info(12,:);


%% Assigning drone variables
% drone 1

T1 = y(id1_T1);
tau_x1 = y(id1_taux1);
tau_y1 = y(id1_tauy1);
tau_z1 = y(id1_tauz1);

phi_d1 = y(id1_phi1);
theta_d1 = y(id1_theta1);
yaw_d1 = y(id1_yaw1);
wx_d1 = y(id1_wx1);
wy_d1 = y(id1_wy1);
wz_d1 = y(id1_wz1);

% drone 2

T2 = y(id2_T2);
tau_x2 = y(id2_taux2);
tau_y2 = y(id2_tauy2);
tau_z2 = y(id2_tauz2);

phi_d2 = y(id2_phi2);
theta_d2 = y(id2_theta2);
yaw_d2 = y(id2_yaw2);
wx_d2 = y(id2_wx2);
wy_d2 = y(id2_wy2);
wz_d2 = y(id2_wz2);


% bar
x_bar = y(xb);
y_bar = y(yb);
z_bar = y(zb);
dot_xbar = y(dot_xb);
dot_ybar = y(dot_yb);
dot_zbar = y(dot_zb);

phi_bar = y(phi_b);
theta_bar = y(theta_b);
yaw_bar = y(yaw_b);

wx_bar = y(wx_b);
wy_bar = y(wy_b);
wz_bar = y(wz_b);


%%
vector_drone1_U = [T1;tau_x1;tau_y1;tau_z1];
vector_drone1_states= [3*phi_d1;3*theta_d1;yaw_d1;wx_d1;wy_d1;wz_d1];
vector_drone2_U = [T2;tau_x2;tau_y2;tau_z2];
vector_drone2_states= [3*phi_d2;3*theta_d2;yaw_d2;wx_d2;wy_d2;wz_d2];

vector_bar_states = [x_bar;y_bar;10*z_bar;dot_xbar;dot_ybar;10*dot_zbar;10*phi_bar;10*theta_bar;yaw_bar;10*wx_bar;10*wy_bar;wz_bar];

vector_variables=[vector_drone1_U;vector_drone1_states;vector_drone2_U;vector_drone2_states;vector_bar_states];


ref_d1= [10*ones(H,1);zeros(3*H,1);zeros(6*H,1)];
ref_d2= [10*ones(H,1);zeros(3*H,1);zeros(6*H,1)];

ref_b= [cos(0.1*pi*k)*ones(H,1);sin(0.1*pi*k)*ones(H,1);zeros(6*H,1);-0*ones(H,1);zeros(3*H,1)];

ref_sys=[ref_d1;ref_d2;ref_b];


cost = sum(vecnorm(vector_variables-ref_sys).^2);
end