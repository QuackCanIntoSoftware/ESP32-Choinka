
struct PwmConfig
{
	mcpwm_unit_t 		unit;
	mcpwm_io_signals_t 	channel;
	mcpwm_timer_t		timer;
	mcpwm_operator_t 	operator;
};