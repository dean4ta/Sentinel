# Published by speed_step_test at each command transition: entering a
# new step or the start of the sine phase. Bag receive timestamps +
# encoder values give average speed per step.
int32 index        # step index; -1 = sine phase
float32 target_mps # target speed m/s (0.0 when sine)
bool sine          # true when this is the sine phase start
