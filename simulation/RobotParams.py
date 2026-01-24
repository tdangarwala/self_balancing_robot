from dataclasses import dataclass

@dataclass
class RobotParams:
    mb: float = 2  # mass of the pendulum (kg)
    mw: float = 0.2  # mass of the wheel (kg)
    l: float = 0.3   # length of the pendulum (m)
    g: float = 9.81  # acceleration due to gravity (m/s^2)
    ib: float = (1/3) * mb * l**2  # moment of inertia of the pendulum (kg*m^2)
    iw: float = 0.002  # moment of inertia of the wheel (kg*m^2)
    rw: float = 0.05  # radius of the wheel (m)