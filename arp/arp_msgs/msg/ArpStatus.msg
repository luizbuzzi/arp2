# 10Hz status message for informational/diagnostics purposes
Header header

#---------------------
# Control information

string[] controller_name

uint8 FAULT_OVERHEAT=1
uint8 FAULT_OVERVOLTAGE=2
uint8 FAULT_UNDERVOLTAGE=4
uint8 FAULT_SHORT_CIRCUIT=8
uint8 FAULT_EMERGENCY_STOP=16
uint8 FAULT_SEPEX_EXCITATION_FAULT=32
uint8 FAULT_MOSFET_FAILURE=64
uint8 FAULT_STARTUP_CONFIG_FAULT=128
uint8[] fault

uint8 STATUS_SERIAL_MODE=1
uint8 STATUS_PULSE_MODE=2
uint8 STATUS_ANALOG_MODE=4
uint8 STATUS_POWER_STAGE_OFF=8
uint8 STATUS_STALL_DETECTED=16
uint8 STATUS_AT_LIMIT=32
uint8 STATUS_MICROBASIC_SCRIPT_RUNNING=128
uint8[] status

# Temperature of main logic chip (C)
float32[] ic_temperature

# Internal supply and reference voltage (V)
float32[] internal_voltage
float32[] adc_voltage

#---------------------
# Channel information

string[] channel_name

# Current flowing in the motors (A)
float32[] motor_current

# Output stage, as a proportion of full (-1..1)
float32[] motor_power

# Commanded and measured speed of the motors (rad/s)
float32[] commanded_velocity

# Electrical power supply to the driver (V, A)
float32[] supply_voltage
float32[] supply_current

#Channel status
uint8 CHANNEL_STATUS_AMPS_LIMIT=1
uint8 CHANNEL_STATUS_MOTOR_STALLED=2
uint8 CHANNEL_STATUS_LOOP_ERROR=4
uint8 CHANNEL_STATUS_SAFETY_STOP=8
uint8 CHANNEL_STATUS_FORWARD_LIMIT=16
uint8 CHANNEL_STATUS_REVERSE_LIMIT=32
uint8 CHANNEL_STATUS_AMPS_TRIGGER=64
uint8[] channel_status

# Measured temperatures (C)
# Motor temp is processed from a thermal sensor connected to analog input 1.
float32[] motor_temperature
# Channel temp is the temperature of the FETs. This is reported by the controller.
float32[] channel_temperature
