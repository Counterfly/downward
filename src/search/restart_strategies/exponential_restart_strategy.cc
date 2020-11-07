#include "exponential_restart_strategy.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;

namespace exponential_restart_strategy {
	ExponentialRestartStrategy::ExponentialRestartStrategy()
		: RestartStrategy()
	{}

	ExponentialRestartStrategy::ExponentialRestartStrategy(long sequence_start_value)
		: RestartStrategy(sequence_start_value)
	{}

	ExponentialRestartStrategy::~ExponentialRestartStrategy()
	{}

	uint64_t ExponentialRestartStrategy::next_sequence_value()
	{
		return sequence(internal_sequence_count++);
	}

	#define SEQUENCE_BITS 64 // number of positive bits in uint64_t
	uint64_t ExponentialRestartStrategy::sequence(long sequence_number)
	{
		--sequence_number;
		if (sequence_number > SEQUENCE_BITS) {
			// Otherwise overflow
			sequence_number = SEQUENCE_BITS;
		}

		return 1L << sequence_number;
	}

	static shared_ptr<RestartStrategy> _parse(OptionParser &parser) {
		parser.document_synopsis("Exponential Restart Strategy", "");

		//Options opts = parser.parse();
		if (parser.dry_run())
			return 0;
		else
			return make_shared<ExponentialRestartStrategy>(); // currently no options
	}

	static Plugin<RestartStrategy> _plugin("exp", _parse);
}