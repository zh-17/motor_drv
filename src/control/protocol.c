#include "protocol.h"
#include "motor.h"

static cmd_t cmd;
static fbk_t fbk;

void motor_feedback(motor_t * self)
{
	fbk.status.enabled = motor_enabled(self);
	fbk.status.fault = (motor_fault(self) != 0);
	fbk.status.saturation = motor_saturation(self);
}
void motor_command(motor_t* self)
{
	if (!cmd.ctl_word.enable)
	{
		motor_disable(self);
		return;
	}
	if (!motor_enabled(self))
	{
		if (cmd.ctl_word.fault_clear)
			motor_clear_error(self);
		if (cmd.ctl_word.enable)
			motor_enable(self);
	}
}