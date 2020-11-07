#include "restart_strategy.h"

#include "../plugin.h"

RestartStrategy::RestartStrategy()
	: internal_sequence_count(1L) {}

RestartStrategy::RestartStrategy(long sequence_start_value)
	: internal_sequence_count(sequence_start_value)
{
	// Assert sequence_start_value > 0
}

RestartStrategy::~RestartStrategy()
{}

void RestartStrategy::reset_sequence()
{
	internal_sequence_count = 1L;
}

static PluginTypePlugin<RestartStrategy> _type_plugin(
    "RestartStrategy",
    // TODO: Replace empty string by synopsis for the wiki page.
    "");