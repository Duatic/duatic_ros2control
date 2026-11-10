# Timestamp
builtin_interfaces/Time timestamp

# The configured drive name
string name
# Raw drive status word
uint32 statusword
# Current joint position [rad]
float64 joint_position
# Current joint velocity [rad/s]
float64 joint_velocity
# Current joint torque [Nm]
float64 joint_effort

# Current motor position [rad]
float64 motor_position
# Current motor velocity [rad/s]
float64 motor_velocity

# Currently commanded joint position [rad]
float64 joint_position_commanded
# Currently commanded joint velocity [rad/s]
float64 joint_velocity_commanded
# Currently commanded joint torque [Nm]
float64 joint_effort_commanded
# Currently commanded motor currents [A]
# In d-q coordinates
float64 current_q_commanded
float64 current_d_commanded

# Current system temperature [°C]
float32 temperature_system
# Current phase A temperature [°C]
float32 temperature_phase_a
# Current phase B temperature [°C]
float32 temperature_phase_b
# Current phase C temperature [°C]
float32 temperature_phase_c

# Current bus voltage [V]
float32 bus_voltage

# Motor currents [A]
# In d-q coordinates
float64 current_q
float64 current_d
# Phase currents
float64 current_phase_a
float64 current_phase_b
float64 current_phase_c
# Phase voltages
float64 voltage_phase_a
float64 voltage_phase_b
float64 voltage_phase_c

# True once motor motion proved the brake pin is out. Always true for drives without a brake.
bool brake_released
