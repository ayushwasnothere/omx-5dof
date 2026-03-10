# A single object to add to the planning scene.

string id                           # unique collision object ID
string type                         # "box", "cylinder", "sphere"
geometry_msgs/PoseStamped pose      # position and frame
float64[] dimensions                # box: [dx,dy,dz], cylinder: [h,r], sphere: [r]
bool use_camera_frame               # if true, pose position is in camera frame
