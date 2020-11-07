#ifndef EXPONENTIAL_RESTART_STRATEGY_H
#define EXPONENTIAL_RESTART_STRATEGY_H

#include "../restart_strategy.h"
#include <cstdint>

namespace exponential_restart_strategy {
	class ExponentialRestartStrategy : public RestartStrategy {
	public:
		ExponentialRestartStrategy();
		ExponentialRestartStrategy(long sequence_start_value);
		~ExponentialRestartStrategy();

	public:
		virtual std::uint64_t next_sequence_value() override;
		virtual std::uint64_t sequence(long sequence_number) override;
	};
}

#endif
