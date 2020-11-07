#ifndef RESTART_STRATEGY_H
#define RESTART_STRATEGY_H

// #include <cstddef>
#include <cstdint>


class RestartStrategy {

public:
	RestartStrategy();
	RestartStrategy(long sequence_start_value);
	virtual ~RestartStrategy();

public:
	virtual std::uint64_t next_sequence_value() = 0;
	virtual std::uint64_t sequence(long sequence_number) = 0;
	virtual void reset_sequence();

protected:
	long internal_sequence_count;
};
#endif
