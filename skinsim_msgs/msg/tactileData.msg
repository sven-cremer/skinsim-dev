# Tactile data

float64 time            # timestamp in the header is the acquisition time of 
                        # the tactile data
                         
int64 patchID
int64[] sensorID

float64[] force
float64[] force_noisy
float64 total_force

# For PPS sensor
float64[] capacitance
float64[] capacitance_noisy
