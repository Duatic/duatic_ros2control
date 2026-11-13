# Timestamp
builtin_interfaces/Time timestamp

# The configured drive name
string name
# The id of the drive on the bus
int32 id
# Current joint position [rad]
float64 joint_position
# Current joint velocity [rad/s]
float64 joint_velocity
# Current joint torque [Nm]
float64 joint_effort

# Currently commanded joint position [rad]
float64 joint_position_commanded
# Currently commanded joint velocity [rad/s]
float64 joint_velocity_commanded
# Currently commanded joint torque [Nm]
float64 joint_effort_commanded
# Currently commanded motor current (q-axis) [A]
float64 motor_current_commanded


#TODO add commanded values?
#TODO refactor into using JointState message?

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

# Drive state machine states, the protocol ids of rsl_drive_sdk
uint16 DRIVE_STATE_NA=0
uint16 DRIVE_STATE_COLDSTART=1
uint16 DRIVE_STATE_WARMSTART=2
uint16 DRIVE_STATE_CONFIGURE=3
uint16 DRIVE_STATE_CALIBRATE=4
uint16 DRIVE_STATE_STANDBY=5
uint16 DRIVE_STATE_MOTOROP=6
uint16 DRIVE_STATE_CONTROLOP=7
uint16 DRIVE_STATE_ERROR=8
uint16 DRIVE_STATE_FATAL=9
uint16 DRIVE_STATE_MOTORPREOP=10
uint16 DRIVE_STATE_DEVICE_MISSING=11
# Current drive state machine state, one of DRIVE_STATE_*
uint16 drive_state
# True while the latest reading from the drive is older than its communication timeout
bool communication_timeout
# True once a drive error froze the hardware system this drive belongs to.
# The latches below stay set until the hardware component is reloaded, in practice when the controller manager restarts.
# The two cause latches only name drives that failed in the cycle that froze the system, not failures after it.
bool error_active
# True once this drive was in Error or Fatal state and so froze its hardware system
bool fault_latched
# True once this drive had a communication timeout and so froze its hardware system
bool communication_timeout_latched
