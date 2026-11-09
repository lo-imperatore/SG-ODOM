# Per-foot output of the ANYmal contact classifier, alongside the
# robot-reported contact for comparison.
uint32 foot_index
string foot_name
bool has_estimated_grf
float64 estimated_grf_z        # reconstructed vertical GRF [N]
bool classified_contact
bool has_measured_contact
float64 measured_wrench_z      # robot-reported contact wrench z [N]
uint8 measured_contact_state   # anymal_msgs/Contact state code
