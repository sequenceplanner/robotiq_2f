# Whether to use the high level or low level mapping
# The gripper will use the default settings:
# Speed: 100%
# Force: 50%
# true => high level will be used
# false => low level will be used
bool use_high_level true

# High level output mapping
string command  # [open, close]

# Low level output mapping
uint8 r_act    # action request
uint8 r_gto    # go to request
uint8 r_atr    # automatic release routine
uint8 r_ard    # automatic release direction
uint8 r_pr     # position request
uint8 r_sp     # speed
uint8 r_fr     # force
