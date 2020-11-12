#include "pure_rrw_scaled.h"

#include "../restart_strategy.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../algorithms/ordered_set.h"
#include "../evaluators/const_evaluator.h"
#include "../evaluators/g_evaluator.h"
#include "../evaluators/pref_evaluator.h"
#include "../operator_id.h"
#include "../task_utils/successor_generator.h"
#include "../task_utils/task_properties.h"
#include "../utils/logging.h"
#include "../utils/rng.h"
#include "../utils/rng_options.h"
#include <cstdint>
#include <unordered_set>

#include <sys/time.h>
#define UNUSED(expr) do { (void)(expr); } while (0)
using namespace std;

namespace pure_rrw_scaled {

	PureRRWScaled::PureRRWScaled(
		const Options &opts)
		: SearchEngine(opts),
		initial_heuristic(opts.get<shared_ptr<Evaluator>>("h")),
		// initial_heuristic_value(-1),
		preferred_operator_evaluators(opts.get_list<shared_ptr<Evaluator>>("preferred")),
		restart_strategy(opts.get<shared_ptr<RestartStrategy>>("restart")),
		probability_preferred(opts.get<double>("pref_prob")),
		current_eval_context(state_registry.get_initial_state(), &statistics),
		last_num_expanded(-1),
      	rng(utils::parse_rng_from_options(opts)) {
		this->use_preferred = preferred_operator_evaluators.size() > 0;

		utils::g_log << "----" << endl;
		utils::g_log << "--------" << endl;
		utils::g_log << "----------------" << endl;
		utils::g_log << "--------------------------------" << endl;
		utils::g_log << "----------------" << endl;
		utils::g_log << "--------" << endl;
		utils::g_log << "----" << endl;
		utils::g_log << "rng-random_seed: " << opts.get<int>("random_seed") << endl;
		utils::g_log << "Pref_Prob (as double) = " << probability_preferred << endl;
		utils::g_log << "Use Preferred =  " << use_preferred << endl;
	}

	PureRRWScaled::~PureRRWScaled() {
	}

	void PureRRWScaled::reach_state(
		const GlobalState &parent, const OperatorID &op, const GlobalState &state) {
		UNUSED(parent);
		UNUSED(op);
		UNUSED(state);
	}

	void PureRRWScaled::save_plan_if_necessary() {
		if (found_solution()) {
			plan_manager.save_plan(plan, task_proxy);
		}
	}

	void PureRRWScaled::initialize() {
		utils::g_log << "Conducting enforced hill-climbing search, (real) bound = "
			<< bound << endl;

		// Re-initialize
		solution_found = false;
		plan.clear();
		d_counts.clear();

		SearchNode node = search_space.get_node(current_eval_context.get_state());
		node.open_initial();

		initial_heuristic_value = current_eval_context.get_evaluator_value(this->initial_heuristic.get());
		assert(initial_heuristic_value > 0);
	}

void PureRRWScaled::get_biased_successors(EvaluationContext &eval_context, ordered_set::OrderedSet<OperatorID> &ops) const {
	unordered_set<OperatorID, utils::operator_id_hasher::OperatorIDHasher> pref_ops;
    unordered_set<OperatorID, utils::operator_id_hasher::OperatorIDHasher> non_pref_ops;
    // if there are no preferred_operator_evaluators 
    // then pref_ops will be empty, and non_pref_ops should contain ops
    successor_generator.get_partitioned_operators(
        eval_context,
        preferred_operator_evaluators,
        pref_ops,
        non_pref_ops
    );
	// utils::g_log << "Pref Ops.size v Non-Pref Ops.size: " << pref_ops.size() << " v " << non_pref_ops.size() << endl;

    // favouring to update stats after random number is generated (and now outside of this function bc of an old const battle)
    // b/c, in theory, we wouldn't need to generate the unselected operator set
    // statistics.inc_expanded();
    // statistics.inc_generated_ops(pref_ops.size() + non_pref_ops.size());
    if (use_preferred && probability_preferred > 0 && pref_ops.size() > 0) {
        // Both operator types exist, randomly choose between the two sets
        double r = (*rng)();
        // utils::g_log << "randomed...." << r << endl;
        if (r < probability_preferred) {
            // utils::g_log << "randoming among preferred" << endl;
            for (OperatorID op : pref_ops) {
                ops.insert(op);
            }
        }
        else {
            // utils::g_log << "randoming among non_pref" << endl;
            for (OperatorID op : non_pref_ops) {
                ops.insert(op);
            }
        }
    }
    else {
        for (OperatorID op : pref_ops) {
            ops.insert(op);
        }
        for (OperatorID op : non_pref_ops) {
            ops.insert(op);
        }
        // utils::g_log << "randoming among both pref and non-pref ops with ops.size = " << ops.size() << endl;
    }

    // Randomize ops
    // ops.shuffle(*rng); -- this didn't work
}

void PureRRWScaled::expand(EvaluationContext &eval_context, int d) {
	UNUSED(eval_context);
	UNUSED(d);
}

bool PureRRWScaled::check_goal_and_set_plan(const GlobalState &state) {
	if (task_properties::is_goal_state(task_proxy, state)) {
		utils::g_log << "Solution found!" << endl;
		set_plan(plan);
		return true;
	}
	return false;
}

void PureRRWScaled::search() {
	initialize();
	status = IN_PROGRESS;
	timer = new utils::CountdownTimer(this->max_time);	// Use as private instance variable so don't have to pass it through function
	while (status == IN_PROGRESS) {
		status = step();
		if (timer->is_expired()) {
			// Timer is done through system (python scripts check call.py, limits.py)
			// Timer will always be inf....stupid
			utils::g_log << "Time limit reached. Abort search." << endl;
			status = TIMEOUT;
			break;
		}
	}

	// utils::g_log << "Actual search time: " << *timer << " [t=" << g_timer << "]" << endl;
	utils::g_log << "Actual search time: " << timer->get_elapsed_time() << endl;
	delete timer;
}


SearchStatus PureRRWScaled::step() {
	last_num_expanded = statistics.get_expanded();
	search_progress.check_progress(current_eval_context);

	if (check_goal_and_set_plan(current_eval_context.get_state())) {
		return SOLVED;
	}

	if (initial_heuristic_value < 1) {
		exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
	}
	//expand(current_eval_context, 0);
	return ehc();
}

SearchStatus PureRRWScaled::ehc() {
	// NOT ACTUALLY EHC, just a pure RW
	// open list is empty..perform RWs or return no solution after enough trials
	EvaluationContext eval_context = current_eval_context;

	uint64_t MAX_TIMESTEP = 1;
	MAX_TIMESTEP <<= 63;

	while (!check_goal_and_set_plan(eval_context.get_state()))
	{

		uint64_t restart_length = restart_strategy->next_sequence_value();
		// utils::g_log << "restart length = " << restart_length << ", " << endl;
		if (restart_length < (MAX_TIMESTEP / initial_heuristic_value)) {
			restart_length = restart_length * initial_heuristic_value;
		}
		else {
			restart_length = MAX_TIMESTEP;
		}

		utils::g_log << "new restart length = " << restart_length << endl;
		uint64_t timestep = 0;
		eval_context = current_eval_context;
		plan.clear();
		while (timestep < restart_length && !check_goal_and_set_plan(eval_context.get_state()))
		{
			++timestep;
			if (timer->is_expired()) {
				// This will never happen, timer handled by system
				utils::g_log << "timeout reached during a RW" << endl;
				return TIMEOUT;
			}
			//utils::g_log << eval_context.get_state().get_id() << " -> " << endl;
			ordered_set::OrderedSet<OperatorID> ops;
			get_biased_successors(eval_context, ops);
			// for (OperatorID op : ops ) {
			// 	utils::g_log << op.get_index() << ", ";
			// }
			// utils::g_log << "  Ops.size = " << ops.size() << endl;
			if (ops.size() == 0) {
				utils::g_log << "Pruned all operators -- resetting current state to EHC root" << endl;
				eval_context = current_eval_context;
				plan.clear(); // TODO: this wasn't in old
			}
			else {
				// randomly select op
				int random_op_id_index = (*rng)(ops.size());
				OperatorID random_op_id = ops[random_op_id_index];
				OperatorProxy random_op = task_proxy.get_operators()[random_op_id_index];
				GlobalState state = state_registry.get_successor_state(eval_context.get_state(), random_op);

				// utils::g_log << "Randomly chosen op is " << random_op_id_index << " acheived new state " << state.get_id().get_value() << endl;
				//reach_state(eval_context.get_state(), random_op, state);

				eval_context = EvaluationContext(state, &statistics);	// Eval Context of successor
				statistics.inc_evaluated_states();	// Evaluating random state
				statistics.inc_expanded();	// Expanding current state
				statistics.inc_generated();	// Only generating one (random) state
				statistics.inc_generated_ops(ops.size());
				// should inc_expanded_states() or inc_generated_states()?

				plan.push_back(random_op_id);
			}
		}
	}

	current_eval_context = eval_context;
	return SOLVED;
	//utils::g_log << "No solution - FAILED" << endl;
	//return FAILED;
}

void PureRRWScaled::print_statistics() const {
	statistics.print_detailed_statistics();
	utils::g_log << "Termination Status: " << search_status::getStringFromSearchStatus(status) << "." << endl;

	// utils::g_log << "For last execution: " << endl;
	for (auto count : d_counts) {
		int depth = count.first;
		int phases = count.second.first;
		assert(phases != 0);
		int total_expansions = count.second.second;
		utils::g_log << "EHC phases of depth " << depth << ": " << phases
			<< " - Avg. Expansions: "
			<< static_cast<double>(total_expansions) / phases << endl;
	}
}

static shared_ptr<SearchEngine> _parse(OptionParser &parser) {
	parser.document_synopsis("Single RW, restarting RWs with sequence-based restarts. If h is supplied then its value from the initial state is multiplied to the restarting sequence.  Uniform probability among operators", "");
	parser.add_option<shared_ptr<Evaluator>>("h", "heuristic. default is to no scale (scale by 1)", "const(1)");
	parser.add_list_option<shared_ptr<Evaluator>>(
			"preferred",
			"use preferred operators of these heuristics",
			"[]");
	parser.add_option<shared_ptr<RestartStrategy>>("restart", "restart strategy (luby [default] or exponential)", "luby");
	parser.add_option<double>("pref_prob", "probability of selecting a preferred operator (when there are some) in local minima (to create distributions. 0 means zero probability of selecting a preferred operator. -1 means do not add any bias (treat pref and unpref the same). Valid input is [0,100])", "-1");
	utils::add_rng_options(parser);

	SearchEngine::add_options_to_parser(parser);
	Options opts = parser.parse();

	if (parser.dry_run())
		return nullptr;
	else
		return make_shared<PureRRWScaled>(opts);
}

static Plugin<SearchEngine> _plugin("pure_rrw_scaled", _parse);
}