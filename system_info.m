function [H,Ts,drone1_info, drone2_info, bar_info] = system_info
    
    H=10;
    Ts = 0.2;
    
    %drone 1
    id1_T1 = 1:H;
    id1_taux1 = (1:H) + H;
    id1_tauy1= (1:H) + 2*H;
    id1_tauz1 = (1:H) + 3*H;
    
    id1_phi1= (1:H) + 4*H;
    id1_theta1 = (1:H) + 5*H;
    id1_yaw1= (1:H) + 6*H;
    
    id1_wx1 = (1:H) + 7*H;
    id1_wy1= (1:H) + 8*H;
    id1_wz1 = (1:H) + 9*H;
    
    %drone 2
    id2_T2= (1:H) + 10*H;
    id2_taux2 = (1:H) + 11*H;
    id2_tauy2= (1:H) + 12*H;
    id2_tauz2 = (1:H) + 13*H;
    
    id2_phi2= (1:H) + 14*H;
    id2_theta2 = (1:H) + 15*H;
    id2_yaw2= (1:H) + 16*H;
    
    id2_wx2 = (1:H) + 17*H;
    id2_wy2= (1:H) + 18*H;
    id2_wz2 = (1:H) + 19*H;
    
    %bar
    xb = (1:H) + 20*H;
    yb = (1:H) + 21*H;
    zb = (1:H) + 22*H;
    dot_xb= (1:H) + 23*H;
    dot_yb = (1:H) + 24*H;
    dot_zb= (1:H) + 25*H;

    phi_b = (1:H) + 26*H;
    theta_b = (1:H) + 27*H;
    yaw_b = (1:H) + 28*H;

    wx_b = (1:H) + 29*H;
    wy_b = (1:H) + 30*H;
    wz_b = (1:H) + 31*H;

    
    
    drone1_U=[id1_T1;id1_taux1;id1_tauy1;id1_tauz1]; 
    drone1_states=[id1_phi1;id1_theta1;id1_yaw1;id1_wx1;id1_wy1;id1_wz1];   
    drone1_info = [drone1_U;drone1_states];
    
    drone2_U=[id2_T2;id2_taux2;id2_tauy2;id2_tauz2];
    drone2_states=[id2_phi2;id2_theta2;id2_yaw2;id2_wx2;id2_wy2;id2_wz2];
    drone2_info=[drone2_U;drone2_states];
    
    bar_info=[xb;yb;zb;dot_xb;dot_yb;dot_zb;phi_b;theta_b;yaw_b;wx_b;wy_b;wz_b];
     
    
end

