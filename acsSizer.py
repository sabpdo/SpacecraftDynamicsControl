from functools import total_ordering
from typing import NamedTuple
import math
import numpy as np
from matplotlib import pyplot as plt
# Based on the work of Bill Nadir
# from https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-851-satellite-engineering-fall-2003/projects/portfolio_nadir2.pdf

class sc_params(NamedTuple):
    dim:  np.array  # [m] length, width, depth
    CG:   np.array  # [m] center of gravity offset from geometric center
    I:    np.array  # [kg*m^2 x, y, z] mass moment of inertia 
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
    mu = 6.836529e15   # [m^3/s^2], Neptune gravity constant
    r_pol = 24621997.59 # [m], Polar radius [CHANGE]
    r_equ = 24621997.59 # [m], Equitorial radius [CHANGE]
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
        A_s = A[0]*A[1] # get surface area
        #print(A_s)
        if ( A[0]*A[2] > A_s):
            A_s = A[0]*A[1]
        if ( A[1]*A[2] > A_s):
            A_s = A[1]*A[2]
        F = (self.Io_s/self.c) * A_s * (1 + self.sc.q_refl) * math.cos(0) # set i = 0 worst case scenario
        # now calculate torque
        # use CG to find maximum worst case moment arm
        T_solar = F * max(abs(self.sc.CG))
        return T_solar
    
    def calc_gravity_torque(self, r):
        # Get the max moment of inertia
        Imax = max(np.diag(self.sc.I))
        # Min moment of inertia
        Imin = min(np.diag(self.sc.I))
        # Angle deviation from vert
        theta = 45 * math.pi/180 # [rad] worst case angle chosen
        T_grav = (3 * self.mu * math.sin(2*theta) * (Imax - Imin)) / (2 * r**3)
        return T_grav

    def calc_magnetic_torque(self, lat, r, re):
        # Neptune magnetic field (approx as dipole)
        B = (1)**(0.5) * 0.3/((r/re)**3) # [guass]
        B_t = B* (1*(10**-4)) # [tesla], [N/(A*m)]
        # Vehicle residual dipole (worst case)
        D = 1 # [A*m^2]
        T_mag = B_t * D # [Nm]
        return T_mag

    def oe2rv(self, nu):
        # Credit: Christopher D. Hall
        # % http://www.aoe.vt.edu/~cdhall/
        # oe2rv.m Orbital Elements to r,v
        # oe = [a e i Om om nu]
        # r,v expressed in IJK frame

        # a = semi-major axis
        # e = eccentricity
        # i = inclination
        # Om = argument of periapsis
        # Omega = right ascension of the ascending node (longitude of ascending node)
        # nu = true anomaly (at epoch). ***(location on orbit)***
        a = self.orbit.a
        e = self.orbit.e
        i = self.orbit.i
        Om = self.orbit.Om
        Omega = self.orbit.Omega

        p = a * (1 - e*e)
        r = p / (1 + e*math.cos(nu))
        rv1 = r * math.cos(nu)
        rv2 = r * math.sin(nu)
        rv = np.array([rv1, rv2, 0])
        vv = math.sqrt(self.mu/p) * np.array([-math.sin(nu), e + math.cos(nu), 0])

        # Rotate
        c0 = math.cos(Om)
        s0 = math.sin(Om)
        co = math.cos(Omega)
        so = math.sin(Omega)
        ci = math.cos(i)
        si = math.sin(i)
        R = np.array([[c0*co-s0*so*ci, -c0*so-s0*co*ci, s0*si], [s0*co+c0*so*ci, -s0*so+c0*co*ci, -c0*si], [so*si, co*si, ci]])
        ri = np.transpose(np.dot(R, rv))
        vi = np.transpose(np.dot(R, vv))
        return ri, vi

    def prop_system_spec(self, H, sat_rate):
        # H: maximum stored momentum in any one momentum wheel
        #   (the saturation point of a momentum wheel) [N*m*s]
        # sat_rate: the rate of saturation of a momentum wheel
        #   (used to determine how often the momentum wheel needs dumped)
        #   [days/saturation]
        Isp = 1000 # Hydrazine (monoprop) with a conservative specific impulse (200 sec)
        g = 11.15
        t = 1 # impulse time

        #Based off guide
        H = 20*10e-3

        # Specify thruster location
        # [x, y, z]
        A = self.sc.dim
        # Location of the six required thrusters
        thruster_1 = np.array([0, A[1]/2, A[2]/2])
        thruster_2 = np.array([0, A[1]/2, -A[2]/2])
        thruster_3 = np.array([A[0]/2, 0, A[2]/2])
        thruster_4 = np.array([-A[0]/2, 0, A[2]/2])
        thruster_5 = np.array([A[0]/2, -A[1]/2, 0])
        thruster_6 = np.array([-A[0]/2, -A[1]/2, 0])

        # Moment arms from CG
        # X thrusters spin about X-axis, moment arm is in Y direction
        # Y thrusters spin about Y-axis, moment arm is in Z direction
        # Z thrusters spin about Z-axis, moment arm is in X direction
        moment_arms_1 = abs( self.sc.CG[1] - thruster_1[1])
        moment_arms_2 = abs( self.sc.CG[1] - thruster_2[1])
        #print(A[1])
        moment_arms_3 = abs( self.sc.CG[2] - thruster_3[2])
        moment_arms_4 = abs( self.sc.CG[2] - thruster_4[2])
        moment_arms_5 = abs( self.sc.CG[0] - thruster_5[0])
        moment_arms_6 = abs( self.sc.CG[0] - thruster_6[0])

        moment_arms = np.array([moment_arms_1, moment_arms_2, moment_arms_3, moment_arms_4, moment_arms_5, moment_arms_6])
        
        # Assume worst case distance from thruster to CG (shortest)
        # Requires larger thrust to impart the required torque on the S/C
        worst_moment_arm = 32.5 #min(moment_arms)
        #print(moment_arms)
        # Thrust required to dump momentum per pulse
        F = H / (worst_moment_arm * t)
        # Required prop mass for this prop system 
        total_pulses = (3 * self.sc.life * 365.25) / sat_rate # total thr pulses required over lifetime
        m_prop = (F * total_pulses * t) / (Isp * g) # [kg]
        # total prop system mass assuming 85% of the prop system mass is propellant (SMAD, p 660)
        p_mass = m_prop / 2.8 # mass in kg
        return F, p_mass

    def run_sizer(self):

        # Calculate time for one orbit
        t = 2 * math.pi * math.sqrt(self.orbit.a**3 / self.mu) # [sec]
        # Calculate solar radiation torques
        Ts = self.calc_solar_torque()
        #print(Ts)
        ang_step = math.pi/50 # [rad]
        start = 0
        stop = 2 * math.pi

        sim_range = np.arange(start, stop + ang_step, ang_step)
        R = list()
        V = list()
        V_MAG = list()
        time = list()
        LAT = list()
        ALT = list()
        TA = list()
        TM = list()
        TG = list()
        TS = list()
        T = list()
        ii = 0
        
        for ang in sim_range:
            # Get orbiit position and velocity
            r, v = self.oe2rv(ang)
            
            v_mag = np.linalg.norm(v) # [m/s] calculate vel magnitude
            R.append(r)
            V.append(v)
            V_MAG.append(v_mag)
            E = math.acos( (self.orbit.e + math.cos(ang)) / (1 + self.orbit.e * math.cos(ang))) # Eccentric anomly
            time_to_ang = math.sqrt(self.orbit.a**3 / self.mu) * (E - self.orbit.e * math.sin(E)) 
            time.append(time_to_ang)
            # Post process orbit elevation (latitude)
            aa = math.sqrt( r[0]**2 + r[1]**2)
            lat = math.atan2( r[2], aa) # [rad]
            LAT.append(lat)

            # Post process altitude
            r_planet = (self.r_pol * self.r_equ) / math.sqrt( self.r_pol**2 * math.cos(lat)**2 + self.r_equ**2 * math.sin(lat)**2) # [m]

            # Calculate Planet radius assuming: oblate, sphereoid
            alt = np.linalg.norm(r) - r_planet # [m] Subtract planet radius from vehicle position vector mag
            ALT.append(alt)

            # Calculate magnetic torques
            Tm = self.calc_magnetic_torque(lat, np.linalg.norm(r), r_planet)
            #print(Tm)
            TM.append(Tm)

            # Calculate Gravity Torques
            Tg = self.calc_gravity_torque(np.linalg.norm(r))
            TG.append(Tg)
            #print(Ts)
            TS.append(Ts)
            # Sum all disturance torque
            T_total = Ts + Tm + Tg # [Nm]
            T.append(T_total)

            ii = ii + 1 # increment counter
        # Post process time values
        max_t = math.ceil(len(time) / 2) - 1
        
        #print(time[max_t-1])
        for jj in np.arange(max_t, len(time)):
            time[jj] = (2 * time[max_t]) - time[jj]
        # Integrate max torques around orbit and find total ang mom
        ang_mom = np.trapz(np.array(T), np.array(time))
        ang_mom_cyc = 0.8 * ang_mom # [Nms] cyclical ang momentum per orbit
   
        ang_mom_sec = ang_mom - ang_mom_cyc # [Nms] secular ang moment per orbit

        # Size ACS thrusters for secular momentum dumping
        orb_sat = 1 # [orbits/saturation]
        day_sat = (orb_sat * t) / 86400 # [days/saturation] {THIS IS WHERE YOU NEED TO EDIT THE SATURATION}
        thrust, t_mass = self.prop_system_spec(ang_mom_sec, day_sat)

        # Generate polar plot
        fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
        ax.plot(np.transpose(sim_range), T)
        ax.plot(np.transpose(sim_range), TS)
        ax.plot(np.transpose(sim_range), TM)
        ax.plot(np.transpose(sim_range), TG)
        ax.grid(True)
        ax.legend(['total', 'solar', 'magnetic', 'gravity'])
        ax.set_title("Disturbance Torques", va='bottom')
        plt.show()

        wheel_cap = (abs(ang_mom_cyc))
        
        return ang_mom_cyc, ang_mom_sec, t_mass, thrust, wheel_cap


if __name__ == "__main__":
    # Run test case
    veh_dim = np.array([65, 65, 1]) # length width depth
    veh_CG  = np.array([0, 0, 0])  # [m] center of gravity offset from geometric center
    veh_mass = 60 # [kg] (minus ACS system)
    veh_life = 7 # [years] Vehicle lifespan

    mu = 6.836529e15
    R = 27e6
    boom_mass = 6
    boom_length = 32.5
    additional_boom_end_mass = 0.234 * 2

    # mass moment of inertia
    Izz = (4 * ((boom_mass * boom_length ** 2 / 3) + (additional_boom_end_mass * boom_length ** 2)))
    Iyy = Izz /2
    Ixx = Izz / 2
    veh_I = np.diag([Ixx, Iyy, Izz])
    print(f"I: {veh_I}")
    veh = sc_params(veh_dim, veh_CG, veh_I, veh_mass, 0.63, 4)

    # Orbit params
    OE_a = 2e8+2.5e7 # [m] semi major axis
    OE_e = 0.0 # eccentricity
    OE_i = 45 * math.pi/180 # [rad] inclination
    OE_Om = 0 # [rad] argument of periapsis
    OE_Omega = 0 # [rad] longitude of ascending node
    orbit = orbit_params(OE_a, OE_e, OE_i, OE_Om, OE_Omega)

    # Setup acsSizer
    sizer_tool = acsSizer(veh, orbit)

    ang_mom_cyc, ang_mom_sec, t_mass, thrust, wheel_cap = sizer_tool.run_sizer()
    print("Angular Cyclic Momentum (Nms): ", ang_mom_cyc)
    print("Angular Secular Momentum (Nms): ", ang_mom_sec)
    print("Total Propellant Mass (kg): ", t_mass)
    print("Thrust required to dump momentum per pulse (N): ", thrust)
    print("Wheel Capacity (Nms): ", wheel_cap)







        


    
        






