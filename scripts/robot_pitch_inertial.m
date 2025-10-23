m_wood = 0.132;
m_battery = 0.103;
m_reg = 0.01;
m_ard = 0.01;
m_stm = 0.117;

m_top = m_wood+m_battery+m_reg+m_ard;
m_bottom = m_wood+m_stm;
m_plate = (m_top+m_bottom)/2;

a = 0.164;
b = 0.106;
t = 0.007;
H = 0.105 - 2*t;
rrod = 0.003;
Lrod = H;
m_rod = 0.03; % 28-29g
z_axle = -0.05;

[I_y_center,I_y_axle,M_tot,z_COM] = robot_pitch_inertia(a,b,t,H, m_plate, rrod, Lrod, m_rod, z_axle)

function [I_y_center,I_y_axle,M_tot,z_COM] = robot_pitch_inertia(a,b,t,H, m_plate, rrod, Lrod, m_rod, z_axle)
% compute_body_inertia  Compute inertia about pitch axis (y) for two plates + 4 rods
%
% Inputs:
%   a        - plate length (forward) [m]
%   b        - plate width  (lateral) [m]   (not used for I_y except in center mass if needed)
%   t        - plate thickness [m]
%   H        - distance between plate centers (top z - bottom z) [m]
%   m_plate  - mass of one plate [kg]  (assumed identical top & bottom)
%   rrod    - rod radius [m]
%   Lrod    - rod length [m]  (rod axis along z; typically Lrod = H)
%   m_rod   - mass of one rod [kg]
%   z_axle  - (optional) z-coordinate of wheel axle relative to O (midpoint). If omitted or
%             empty, no axle-reflection is made.
%
% Outputs (struct):
%   I_y_center  - inertia about y-axis through O (midpoint) [kg*m^2]
%   I_y_axle    - inertia about y-axis through z = z_axle (if z_axle provided)
%   M_tot       - total mass [kg]
%   z_COM       - z coordinate of COM relative to O (m)
%
% Example:
%   out = compute_body_inertia(0.12, 0.08, 0.01, 0.10, 0.2, 0.003, 0.10, 0.05, -0.06);

% validate inputs
if nargin < 9
    z_axle = [];
end

% total mass
M_tot = 2*m_plate + 4*m_rod;

% center of mass z (symmetric if masses equal)
% top plate at +H/2, bottom at -H/2, rods center at z=0 (assumed)
z_COM = ( m_plate*(+H/2) + m_plate*(-H/2) + 4*m_rod*0 ) / M_tot;

% plate inertia about its own center (axis parallel to y)
I_plate_cent_y = (1/12) * m_plate * (a^2 + t^2);  % kg*m^2

% plate about O (parallel axis)
I_plate_aboutO = I_plate_cent_y + m_plate*(H/2)^2;

% two plates total
I_plates_total = 2 * I_plate_aboutO;

% rod inertia about its center, perpendicular to rod axis
I_rod_cent_perp = (1/12) * m_rod * (3*rrod^2 + Lrod^2);

% rod offset in x direction (distance from body center)
x_offset = a/2;

% each rod about O (parallel axis)
I_rod_aboutO_each = I_rod_cent_perp + m_rod * x_offset^2;

% sum for 4 rods
I_rods_total = 4 * I_rod_aboutO_each;

% total inertia about y through O
I_y_center = I_plates_total + I_rods_total;

if ~isempty(z_axle)
    % parallel axis to axle at z = z_axle (distance from O)
    d = z_axle - 0;   % O at z=0
    I_y_axle = I_y_center + M_tot * d^2;
else
    I_y_axle = [];
end

end
