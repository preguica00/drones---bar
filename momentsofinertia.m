function [Ixx,Iyy,Izz,a1,a2,a3,Ixx_sys,Iyy_sys,Izz_sys] = momentsofinertia(yaw_bar)

[m_drone,m_bar,m_sys,g, C_barra] = parameters;
%moments of inertia drones
Ixx = 0.0119;
Iyy=0.0119;
Izz=0.0223;

a1 = (Iyy-Izz)/Ixx;
a2 = (Izz-Ixx)/Iyy;
a3 =  (Ixx-Iyy)/Izz;

%moments of inertia bar
Ixxb = 0;
Iyyb = (m_bar*C_barra^2)/12;
Izzb = (m_bar*C_barra^2)/12;

%moments of inertia of system
Ixx_sys= 2*Ixx+2*m_drone*( ((C_barra^2)/4)*(cos(yaw_bar)).^2) + Ixxb;
Iyy_sys=2*Iyy+2*m_drone*( ((C_barra^2)/4)*(sin(yaw_bar)).^2) + Iyyb;
Izz_sys=2*Izz+2*m_drone*( ((C_barra^2)/4)*(sin(yaw_bar)).^2 + ((C_barra^2)/4)*(cos(yaw_bar)).^2) + Izzb;



end