function [Ixx_total, Iyy_total, Izz_total] = robot_all_inertia(...
    m_plate, a, b, t, ...   % wooden plates: mass, length (x), width (y), thickness (z)
    m_rod, r_rod, L_rod, ...% steel rods: mass, radius, length
    h)                      % vertical distance between plates

% -----------------------------------------------------------
% Wooden plates (two identical, one top, one bottom)
% -----------------------------------------------------------
% Plate inertias about centroid:
Ixx_plate_c = (1/12)*m_plate*(b^2 + t^2);
Iyy_plate_c = (1/12)*m_plate*(a^2 + t^2);
Izz_plate_c = (1/12)*m_plate*(a^2 + b^2);

% Parallel axis shift for top/bottom plates
dz = h/2;  % distance from robot COM plane
Ixx_plate = Ixx_plate_c + m_plate*dz^2;
Iyy_plate = Iyy_plate_c + m_plate*dz^2;
Izz_plate = Izz_plate_c;  % shift along z-axis is 0

% Total plates contribution
Ixx_plates = 2*Ixx_plate;
Iyy_plates = 2*Iyy_plate;
Izz_plates = 2*Izz_plate;

% -----------------------------------------------------------
% Steel rods (4 vertical at the corners)
% -----------------------------------------------------------
% Each rod modeled as a slender cylinder aligned with z-axis
Izz_rod_c = (1/2)*m_rod*r_rod^2;                     % about its own z-axis
Ixx_rod_c = (1/12)*m_rod*(3*r_rod^2 + L_rod^2);      % through centroid
Iyy_rod_c = Ixx_rod_c;                               % symmetry

% Distances of rods from center (corners of rectangle a x b)
dx = a/2;  % along x
dy = b/2;  % along y

% Parallel axis terms
d_sq = dx^2 + dy^2;
Ixx_rod = Ixx_rod_c + m_rod*dy^2;
Iyy_rod = Iyy_rod_c + m_rod*dx^2;
Izz_rod = Izz_rod_c + m_rod*d_sq;

% Total rods contribution (4 rods)
Ixx_rods = 4*Ixx_rod;
Iyy_rods = 4*Iyy_rod;
Izz_rods = 4*Izz_rod;

% -----------------------------------------------------------
% Total inertia
% -----------------------------------------------------------
Ixx_total = Ixx_plates + Ixx_rods;
Iyy_total = Iyy_plates + Iyy_rods;
Izz_total = Izz_plates + Izz_rods;

end
