  function  [command, optimum, predicted_trajectory] = optimizetrajectory(current_state, optimum,k)


%% Initial conditions 
    
    % drone 1
    phi_init_d1 = current_state(1);
    theta_init_d1 = current_state(2);
    yaw_init_d1 = current_state(3);

    wx_init_d1 = current_state(4);
    wy_init_d1 = current_state(5);
    wz_init_d1 = current_state(6);

    % drone 2
    phi_init_d2 = current_state(7);
    theta_init_d2 = current_state(8);
    yaw_init_d2 = current_state(9);

    wx_init_d2 = current_state(10);
    wy_init_d2 = current_state(11);
    wz_init_d2 = current_state(12);

    
    %bar
    x_init_bar= current_state(13);
    y_init_bar = current_state(14);
    z_init_bar = current_state(15);
    dotx_init_bar = current_state(16);
    doty_init_bar = current_state(17);
    dotz_init_bar = current_state(18);

    phi_init_bar = current_state(19);
    theta_init_bar = current_state(20);
    yaw_init_bar = current_state(21);
    wx_init_bar = current_state(22);
    wy_init_bar = current_state(23);
    wz_init_bar = current_state(24);
    
    
    
    init_vector_d1 = [phi_init_d1; theta_init_d1; yaw_init_d1; wx_init_d1;wy_init_d1;wz_init_d1];
    init_vector_d2 = [phi_init_d2; theta_init_d2; yaw_init_d2; wx_init_d2;wy_init_d2;wz_init_d2];

    init_vector_bar_trans = [x_init_bar;y_init_bar;z_init_bar;dotx_init_bar;doty_init_bar;dotz_init_bar];
    init_vector_bar_rot= [phi_init_bar;theta_init_bar;yaw_init_bar;wx_init_bar;wy_init_bar;wz_init_bar];
    
    init_vector = [init_vector_d1;init_vector_d2;init_vector_bar_trans;init_vector_bar_rot];
    
  %% optimizer
  
  [H,Ts,drone1_info, drone2_info, bar_info] = system_info;
     
    %initial conditions
    if isempty(optimum)
        optimum = [10*ones(H,1);zeros(9*H,1);10*ones(H,1);zeros(21*H,1)];
    end

    opts = optimoptions('fmincon','Algorithm','sqp','TolFun',0.001,'MaxIter',100000,'MaxFunEvals',100000);

%  
%     lb =[-5*ones(H,1);0*ones(H,1);-Inf*ones(2*H,1);-5*ones(H,1);0*ones(H,1);-Inf*ones(8*H,1)];
%     ub=[5*ones(H,1);30*ones(H,1);Inf*ones(2*H,1);5*ones(H,1);30*ones(H,1); Inf*ones(8*H,1)];

    lb=[];
    ub=[];
    [optimum, ~] = fmincon(@(y)costfunction(y, H,k), optimum,[],[],[],[],lb,ub,@(y)discretization(y,init_vector),opts);
 
    

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
   %% optimal control
   % drone 1
   T1_opt  = optimum(id1_T1);
   tau_x1_opt  = optimum(id1_taux1);
   tau_y1_opt  = optimum(id1_tauy1);
   tau_z1_opt  = optimum(id1_tauz1);
   
   phi_d1_opt  = optimum(id1_phi1);
   theta_d1_opt  = optimum(id1_theta1);
   yaw_d1_opt  = optimum(id1_yaw1);
   wx_d1_opt  = optimum(id1_wx1);
   wy_d1_opt  = optimum(id1_wy1);
   wz_d1_opt  = optimum(id1_wz1);

   
   % drone 2
   T2_opt = optimum(id2_T2);
   tau_x2_opt  = optimum(id2_taux2);
   tau_y2_opt  = optimum(id2_tauy2);
   tau_z2_opt  = optimum(id2_tauz2);
   
   phi_d2_opt  = optimum(id2_phi2);
   theta_d2_opt  = optimum(id2_theta2);
   yaw_d2_opt  = optimum(id2_yaw2);
   wx_d2_opt  = optimum(id2_wx2);
   wy_d2_opt  = optimum(id2_wy2);
   wz_d2_opt  = optimum(id2_wz2);
   
   % bar
   
   x_bar_opt  = optimum(xb);
   y_bar_opt  = optimum(yb);
   z_bar_opt  = optimum(zb);
   dot_xbar_opt  = optimum(dot_xb);
   dot_ybar_opt  = optimum(dot_yb);
   dot_zbar_opt  = optimum(dot_zb);

   phi_bar_opt  = optimum(phi_b);
   theta_bar_opt  = optimum(theta_b);
   yaw_bar_opt  = optimum(yaw_b);
   wx_bar_opt  = optimum(wx_b);
   wy_bar_opt  = optimum(wy_b);
   wz_bar_opt  = optimum(wz_b);
   
    
   command_d1=[T1_opt(1), tau_x1_opt(1),tau_y1_opt(1), tau_z1_opt(1)];
   command_d2=[T2_opt(1),tau_x2_opt(1),tau_y2_opt(1),tau_z2_opt(1)];
   command =[command_d1,command_d2];

   predicted_d1=[phi_d1_opt,theta_d1_opt,yaw_d1_opt,wx_d1_opt,wy_d1_opt,wz_d1_opt];
   predicted_d2=[phi_d2_opt,theta_d2_opt,yaw_d2_opt,wx_d2_opt,wy_d2_opt,wz_d2_opt];
   predicted_b=[x_bar_opt,y_bar_opt,z_bar_opt,dot_xbar_opt,dot_ybar_opt,dot_zbar_opt];
   predicted_b_rot=[phi_bar_opt,theta_bar_opt,yaw_bar_opt,wx_bar_opt,wy_bar_opt,wz_bar_opt];
   predicted_trajectory = [predicted_d1,predicted_d2,predicted_b,predicted_b_rot];

   
  
  end


