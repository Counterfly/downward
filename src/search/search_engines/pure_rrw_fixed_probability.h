#ifndef PURE_RRW_FP_H
#define PURE_RRW_FP_H


#include "../evaluation_context.h"
#include "../heuristic.h"
#include "../search_engine.h"
#include "../search_statistics.h"
#include "../utils/countdown_timer.h"
#include "../utils/rng.h"
#include "../utils/rng_options.h"

#include "../open_list.h"
#include "../state_id.h"

#include <map>
#include <memory> 
#include <set>
#include <utility>
#include <vector>

namespace options {
class OptionParser;
class Options;
}
class RestartStrategy;

typedef std::pair<StateID, std::pair<int, OperatorID >> OpenListEntryEHC;

enum class PreferredUsage {
	DO_NOTHING,	/* using preferred operators but don't want to influence hill-climbing (maybe using in localminima*/
    PRUNE_BY_PREFERRED,
    RANK_PREFERRED_FIRST
};
inline std::ostream& operator<<(std::ostream& out, const PreferredUsage value){
    const char* s = 0;
    if (value == PreferredUsage::DO_NOTHING) {
    	s = "DO NOTHING";
    }
    else if (value == PreferredUsage::PRUNE_BY_PREFERRED) {
    	s = "PRUNE_BY_PREFERRED";
    }
    else if (value == PreferredUsage::RANK_PREFERRED_FIRST) {
    	s = "RANK_BY_PREFERRED_FIRST";
    }
    else {
    	s = "DNE, something went wrong";
    }

    return out << s;
}

/*
  Enforced hill-climbing with deferred evaluation.

  TODO: We should test if this lazy implementation really has any benefits over
  an eager one. We hypothesize that both versions need to evaluate and store
  the same states anyways.
*/
namespace pure_rrw_fp {
    class PureRRWFixedProb : public SearchEngine {
        void get_biased_successors(
            EvaluationContext &eval_context,
            ordered_set::OrderedSet<OperatorID> &ops) const;	// Used only in RW section
        void expand(EvaluationContext &eval_context, int d);
        void reach_state(
            const GlobalState &parent, const OperatorID &op,
            const GlobalState &state);
        SearchStatus ehc();

        std::shared_ptr<Evaluator> scaling_heuristic;
        unsigned int scaling_factor;
        std::vector<std::shared_ptr<Evaluator>> preferred_operator_evaluators;
        bool use_preferred;

        double restart_probability;     // [0,1], fixed probability restart rate
        double probability_preferred;   // Probability of selecting a preferred operator in successor generation (to create non-uniform distributions)
        int num_restarts;

        EvaluationContext current_eval_context;

        // Statistics
        std::map<int, std::pair<int, int>> d_counts;
        int last_num_expanded;

        utils::CountdownTimer* timer;
    protected:
        std::shared_ptr<utils::RandomNumberGenerator> rng;
        virtual void initialize() override;
        virtual SearchStatus step() override;

        // So that we can extract proper plan.
        //  ow, using the parent pointer won't work for RWs as a state may be revisited and create an inf loop
        virtual bool check_goal_and_set_plan(const GlobalState &state) override;

    public:
        explicit PureRRWFixedProb(const options::Options &opts);
        virtual ~PureRRWFixedProb() override;

        virtual void print_statistics() const override;

        virtual void search() override;

        virtual void save_plan_if_necessary() override;

    private:
        long luby_sequence(long sequence_number);
    };
}

#endif
