# Message of DSDM actuator sensor feedback
Header header
float32[] theta     # vector of shaft positions  [output,M1,M2] [rad]
float32[] w         # vector of shaft velocities [output,M1,M2] [rad/sec]
float32[] y_raw     # vector of shaft positions  [output,M1,M2] [ticks]
float32[] w_raw     # vector of shaft velocities [output,M1,M2] [ticks/period]
float32   a         # actuator position [rad or meter]
float32   da        # actuator velocity [rad/sec or meter/sec]
