from typing import NamedTuple
import math
import numpy as np
# Based on the work of Bill Nadir
# from https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-851-satellite-engineering-fall-2003/projects/portfolio_nadir2.pdf

class sc_params(NamedTuple, np):
    dim:  np.array  # [m] length, width, depth
    CG:   np.array  # [m] center of gravity offset from geometric center
    mass: float # [kg] (minus ACS system)
    q_refl:  int   # Surface reflectance q
    life: int   # [years] Vehicle lifespan

class orbit_params(NamedTuple):
    a:     float # [m] semi-major axis
    e:     float # eccentricity
    i:     float # [rad] inclination
    Om:    float # [rad] argument of periapsis (angle from ascending node to periapsis)
    Omega: float # [rad] longitude of ascending node (angle between x and asc. node)


class acsSizer():

    # Planet properties
    mu = 3.986e14   # [m^3/s^2], Earth gravity constant
    r_pol = 6357000 # [m], Polar radius
    r_equ = 6378000 # [m], Equitorial radius
    # Constants
    c = 3*10**8  # [m/s] Speed of light 
    Io_s = 1367; # Solar constant W/m^2

    def __init__(self, sc, orbit):
        self.sc = sc
        self.orbit = orbit


    def calc_solar_torque(self):
        # Ts = torque_solar(veh.dim, veh.CG, 0, veh.mat); 
        # Calculate solar radiation pressure torque

        # Find surface area of the largest face of orbit
        A = self.sc.dim
        A_s = A[1]*A[2] # get surface area
        if ( A[1]*A[3] > A_s):
            A_s = A[1]*A[2]
        if ( A[2]*A[3] > A_s):
            A_s = A[2]*A[3]
        F = (self.Io_s/self.c) * A_s * (1 + self.sc.q_refl) * math.cos(0) # set i = 0 worst case scenario
        # now calculate torque
        # use CG to find maximum worst case moment arm
        T_solar = F * max(abs(self.sc.CG))
        return T_solar
    
    def calc_aero_torque(self, alt, V):
        # alt [m]
        # V airspeed [m/s]
        # Aero Constants
        T = -131.21 + 0.00299*alt # [deg C] Atmospheric Temperature
        p = 2.488*(((T + 273.1)/216.6)**-11.388) # [KPa] Atmospheric Pressure
        rho = p / (0.2869**(T + 273.1)) # [kg/m^3]
        C_D = 2.2 # drag coefficient of cube shaped SC

        # Assume the center of pressure is at the center of the face of one side
        # of the cube which is facing direclty into the atmosphere = allow for max drag
        # Here the surface areas of the sides of the S/C are determined
        # max([x*z y*z x*y])
        A = self.sc.dim
        area_1 = A[1] * A[3]
        area_2 = A[2] * A[3]
        area_3 = A[1] * A[2]
        max_area = max(np.array( [area_1, area_2, area_3] ))
        F = 0.5 * rho * C_D * (max_area**2) * (V**2)
        T_aero = F * max(abs(self.sc.CG))
        return T_aero

    def calc_gravity_torque(self):
        




    
        






