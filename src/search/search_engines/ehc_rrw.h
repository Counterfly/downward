#ifndef ENFORCED_HILL_CLIMBING_SEARCH_RW_H
#define ENFORCED_HILL_CLIMBING_SEARCH_RW_H

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

namespace enforced_hill_climbing_restarting_random_walk {
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
  This is mostly enforced_hill_climbing_search.h with some deviations.
*/
class EnforcedHillClimbingSearchRRW : public SearchEngine {
    std::unique_ptr<EdgeOpenList> open_list;

    std::shared_ptr<Evaluator> evaluator;
    std::vector<std::shared_ptr<Evaluator>> preferred_operator_evaluators;
    std::set<Evaluator *> path_dependent_evaluators;
    bool use_preferred;
    PreferredUsage preferred_usage;

    EvaluationContext current_eval_context;
    int current_phase_start_g;

    // Statistics
    std::map<int, std::pair<int, int>> d_counts;
    int num_ehc_phases;
    int last_num_expanded;

    void insert_successor_into_open_list(
        const EvaluationContext &eval_context,
        int parent_g,
        OperatorID op_id,
        bool preferred);
    
    void get_biased_successors(EvaluationContext &eval_context, ordered_set::OrderedSet<OperatorID> &ops);
    
    void expand(EvaluationContext &eval_context);
    void reach_state(
        const GlobalState &parent, OperatorID op_id,
        const GlobalState &state);
    SearchStatus ehc();

    std::shared_ptr<RestartStrategy> restart_strategy;
    double probability_preferred;		// Probability of selecting a preferred operator in successor generation (to create non-uniform distributions)

    utils::CountdownTimer* timer;
protected:
    std::shared_ptr<utils::RandomNumberGenerator> rng;
    virtual void initialize() override;
    virtual SearchStatus step() override;
    // So that we can extract proper plan.
    //  ow, using the parent pointer won't work for RWs as a state may be revisited and create an inf loop
    virtual bool check_goal_and_set_plan(const GlobalState &state) override;

public:
    explicit EnforcedHillClimbingSearchRRW(const options::Options &opts);
    virtual ~EnforcedHillClimbingSearchRRW() override;

    virtual void print_statistics() const override;

    virtual void search() override;
};
}

#endif
