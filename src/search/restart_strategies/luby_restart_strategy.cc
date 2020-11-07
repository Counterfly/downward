#include "luby_restart_strategy.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;

namespace luby_restart_strategy {
	LubyRestartStrategy::LubyRestartStrategy()
		: RestartStrategy()
	{}

	LubyRestartStrategy::LubyRestartStrategy(long sequence_start_value)
		: RestartStrategy(sequence_start_value)
	{}

	LubyRestartStrategy::~LubyRestartStrategy()
	{}

	uint64_t LubyRestartStrategy::next_sequence_value()
	{
		return sequence(internal_sequence_count++);
	}


	uint64_t LubyRestartStrategy::sequence(long sequence_number)
	{
		long focus = 2L;
		while (sequence_number > (focus - 1)) {
			focus = focus << 1;
		}

		if (sequence_number == (focus - 1)) {
			return focus >> 1;
		}
		else {
			return sequence(sequence_number - (focus >> 1) + 1);
		}
	}

	static shared_ptr<RestartStrategy> _parse(OptionParser &parser) {
		parser.document_synopsis("Luby Restart Strategy", "");

		// Options opts = parser.parse();
		if (parser.dry_run())
			return 0;
		else
			return make_shared<LubyRestartStrategy>(); // currently no options
	}

	static Plugin<RestartStrategy> _plugin("luby", _parse);
}