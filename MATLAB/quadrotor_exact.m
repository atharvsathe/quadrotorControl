% Tim McNamara
% 18-776 Project Part 1

function dxdt = quadrotor_exact(t, x, K)
    m = 0.6;
    Jx = 0.0092;
    Jy = 0.0091;
    Jz = 0.0101;
    g = 9.81;

    px_dot = x(1);
%   px =     x(2);
    py_dot = x(3);
%   py =     x(4);
    pz_dot = x(5);
%   pz =     x(6);
    p =      x(7);
    phi =    x(8);
    q =      x(9); 
    theta =  x(10);
    r =      x(11);
    psi =    x(12);
    
    u = -K*x;
    u(1) = u(1) + m*g;
    
    u_clipped(1) = max([min([u(1), 16]),0]);
    u_clipped(2) = max([min([u(2), .432]),-.432]);
    u_clipped(3) = max([min([u(3), .432]),-.432]);
    u_clipped(4) = max([min([u(4), .1]),-.1]);
    
%     F = u(1) + m*g;
%     tau_phi = u(2);
%     tau_theta = u(3);
%     tau_psi = u(4);

    F = u_clipped(1);
    tau_phi = u_clipped(2);
    tau_theta = u_clipped(3);
    tau_psi = u_clipped(4);
    
    az = -F/m;

    dxdt =  [cos(phi)*sin(theta)*az;
             px_dot;
             -sin(phi)*az;
             py_dot;
             g + cos(phi)*cos(theta)*az;
             pz_dot;             
             1/Jx*tau_phi;
             p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
             1/Jy*tau_theta;
             q*cos(phi) - r*sin(phi);
             1/Jz*tau_psi;
             q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta)];
end