################################
### Tyler Ard                ###
### Argonne National Lab     ###
### Vehicle Mobility Systems ###
### tard(at)anl(dot)gov      ###
################################

### Enums
def enum(**enums):
    return type('Enum', (), enums)

# Different object types and statuses
VEHTYPE = enum(SAS=-3, EXT=-2, NONE=-1, MOBIL=0, CAV=1, PCC=2, CYCLE=3) # Enums for type of vehicle as assigned in SIM matrix

### Settings
# Simulation settings
RANDSEED = 44 # Random seed used to fix the pseudo random generation. None to use system time

TEND = 205.0 # time til simulation ends automatically [s]

# Vehicle parameter settings
VEHWIDTH = 1.90 # assumed vehicle width [m]
VEHLENGTH = 3.25 # assumed vehicle length [m]

# Connectivity settings
CONN_RANGE = 450 # [m] Reliable connectivity range based on Chicago testing
SENSOR_RANGE = 100 # [m] Reliable sensor detection range

# Prediction settings
USING_PRED = False # If using prediction provided from cycle
USING_PREVIEW = True # If using a direct preview of upcoming cycle

# Traffic control setting
USING_ONLINE_MPC = False # If using online MPC to track front vehicle
USING_NEURAL_NETWORK = True # If using neural network controller to track front vehicle
USING_LOOKUP_TABLE = False # If using lookup table to track front vehicle

### Error checking
assert not (USING_PRED and USING_PREVIEW), 'Cannot use both PRED and PREVIEW settings.'

if USING_PRED:
    print('Using externally-provided prediction.')
elif USING_PREVIEW:
    print('Using future preview of cycle motion.')

assert not (USING_PRED), 'USING PRED Not implemented.'