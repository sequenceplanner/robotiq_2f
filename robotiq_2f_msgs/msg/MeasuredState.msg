# High level input mapping
string measured # [opened, closed, gripping, fault, unknown]

# Low level output mapping
uint8 g_act    # activation status
uint8 g_gto    # action status
uint8 g_sta    # gripper status
uint8 g_obj    # object detection status
uint8 g_flt    # fault status
uint8 g_pr     # position request echo
uint8 g_po     # position
uint8 g_cu     # current