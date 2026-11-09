# Per-foot output of the Spot joint-torque GRF contact classifier.
uint32 foot_index
string foot_name
bool valid                 # GRF reconstruction succeeded for this sample
float64 grf_z              # reconstructed vertical GRF [N]
float64 classifier_force   # force value fed to the contact classifier [N]
bool classified_contact
